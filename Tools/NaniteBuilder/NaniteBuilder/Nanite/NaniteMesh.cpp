#include "NaniteMesh.h"

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <array>
#include <cassert>

#include "../Utils/Utils.h"	
#include "NaniteBuilder.h"
#include "QEM/QEMSimplification.h"

namespace nanite
{
	bool NaniteMesh::Build(const Mesh& originMesh, int leafTriThreshold)
	{
		const int MAX_DEPTH = 32;
		mLevels.reserve(MAX_DEPTH);

		mLevels.push_back(Level());
		mLevels[0].Mesh = originMesh;
		int nparts = static_cast<int>(std::ceilf(mLevels[0].Mesh.Triangles.size() / (float)leafTriThreshold));
		if (nparts < 1)
			return false;

		int edgeCut = NaniteBuilder::SplitMeshIntoClusters(nparts, &mLevels[0].Mesh, &mLevels[0].Clusters);

		for (int depth = 0; depth < MAX_DEPTH - 1; ++depth)
		{
			Level& srcLevel = mLevels[depth];
			Mesh& srcMesh = srcLevel.Mesh;
			std::vector<Cluster>& srcClusters = srcLevel.Clusters;

			mLevels.push_back(Level());
			Level& dstLevel = mLevels[1 + depth];
			Mesh& dstMesh = dstLevel.Mesh;

			std::vector<Cluster> tmpClusters;
			for (int ci = 0; ci < srcClusters.size(); ++ci)
			{
				const Cluster& srcCluster = srcClusters[ci];
				std::vector<Triangle> slicedTriangles(srcMesh.Triangles.begin() + srcCluster.StartIndex, srcMesh.Triangles.begin() + srcCluster.StartIndex + srcCluster.NumTriangles);

				auto [simplifiedVertices, simplifiedTriangles]
					= qem::SimplifyMesh(static_cast<int>(slicedTriangles.size()) / 2, slicedTriangles, srcMesh.Vertices);
				int offset = static_cast<int>(dstMesh.Vertices.size());
				for (Triangle& tri : simplifiedTriangles)
				{
					tri.i0 += offset;
					tri.i1 += offset;
					tri.i2 += offset;
				}

				if (simplifiedTriangles.size() == 0)
					continue;

				Cluster tmpCluster;
				tmpCluster.StartIndex = static_cast<int>(dstMesh.Triangles.size());
				tmpCluster.NumTriangles = static_cast<int>(simplifiedTriangles.size());
				tmpClusters.push_back(tmpCluster);

				dstMesh.Vertices.insert(dstMesh.Vertices.end(), simplifiedVertices.begin(), simplifiedVertices.end());
				dstMesh.Triangles.insert(dstMesh.Triangles.end(), simplifiedTriangles.begin(), simplifiedTriangles.end());
			}

			mergeDuplicatedVertices(&dstMesh);

			std::vector<Cluster> dstClusters;
			for (const Cluster& tmpCluster : tmpClusters)
			{
				std::vector<Cluster> subClusters;
				NaniteBuilder::SplitMeshIntoClusters(
					2, tmpCluster.StartIndex, tmpCluster.NumTriangles,
					dstMesh.Vertices, &dstMesh.Triangles, &subClusters);
				assert(subClusters.size() == 2);
				dstClusters.insert(dstClusters.end(), subClusters.begin(), subClusters.end());
			}
			int objval = NaniteBuilder::MergeClusters(dstClusters, &dstMesh, &dstLevel.Clusters);

			if (objval < 1 || dstLevel.Clusters.size() >= srcLevel.Clusters.size())
				break;
		}

		Level& tailLevel = mLevels[mLevels.size() - 1];
		Cluster wholeCluster;
		wholeCluster.StartIndex = 0;
		wholeCluster.NumTriangles = static_cast<int>(tailLevel.Mesh.Triangles.size());
		wholeCluster.Bounds = ComputeBoundingBox(tailLevel.Mesh.Vertices);
		tailLevel.Clusters.clear();
		tailLevel.Clusters.emplace_back(wholeCluster);

		return true;
	}

	void NaniteMesh::PaintColorByCluster()
	{
		for (Level& level : mLevels)
		{
			for (int i = 0; i < level.Clusters.size(); ++i)
			{
				const Cluster& cluster = level.Clusters[i];
				FVector3 color = HSVtoRGB(std::fmod(i / 16.f, 1.f), 1.f, 1.f);
				for (int j = cluster.StartIndex; j < cluster.StartIndex + cluster.NumTriangles; ++j)
				{
					level.Mesh.Triangles[j].Color = color;
				}
			}
		}
	}

	void NaniteMesh::mergeDuplicatedVertices(Mesh* mesh)
	{
		std::unordered_map<FVector3, uint32_t, FVector3Hasher> uniqueVertexMap;
		std::vector<FVector3> outVertices;

		auto update = [&](uint32_t& i, const FVector3& v)
			{
				auto it = uniqueVertexMap.find(v);
				if (it != uniqueVertexMap.end())
				{
					i = it->second;
				}
				else
				{
					uint32_t newIndex = static_cast<uint32_t>(outVertices.size());
					outVertices.emplace_back(v);
					uniqueVertexMap[v] = newIndex;
					i = newIndex;
				}
			};

		for (Triangle& triangle : mesh->Triangles)
		{
			update(triangle.i0, mesh->Vertices[triangle.i0]);
			update(triangle.i1, mesh->Vertices[triangle.i1]);
			update(triangle.i2, mesh->Vertices[triangle.i2]);
		}

		mesh->Vertices = std::move(outVertices);
	}

	void NaniteMesh::mergeDuplicatedTriangles(Mesh* mesh)
	{
		struct SortedTriangleKey 
		{
			std::array<uint32_t, 3> Indices;
			SortedTriangleKey(uint32_t a, uint32_t b, uint32_t c) 
			{
				Indices = { a, b, c };
				std::sort(Indices.begin(), Indices.end());
			}
			bool operator==(const SortedTriangleKey& other) const 
			{
				return Indices == other.Indices;
			}
		};

		struct SortedTriangleHash 
		{
			size_t operator()(const SortedTriangleKey& key) const 
			{
				return std::hash<uint32_t>()(key.Indices[0]) ^
					(std::hash<uint32_t>()(key.Indices[1]) << 1) ^
					(std::hash<uint32_t>()(key.Indices[2]) << 2);
			}
		};

		std::unordered_set<SortedTriangleKey, SortedTriangleHash> seen;
		std::vector<Triangle> unique;

		for (const Triangle& tri : mesh->Triangles)
		{
			SortedTriangleKey key(tri.i0, tri.i1, tri.i2);
			if (seen.find(key) == seen.end())
			{
				seen.insert(key);
				unique.push_back(tri);
			}
		}

		mesh->Triangles = std::move(unique);
	}
}