#include "NaniteBuilder.h"

#include <cassert>

namespace nanite
{
	int NaniteBuilder::SplitMeshIntoClusters(int nparts, Mesh* inoutMesh, std::vector<Cluster>* outClusters)
	{
		return SplitMeshIntoClusters(nparts, inoutMesh->Vertices, &(inoutMesh->Triangles), outClusters);
	}

	int NaniteBuilder::SplitMeshIntoClusters(
		int nparts,
		const std::vector<FVector3> vertices,
		std::vector<Triangle>* inoutTriangles,
		std::vector<Cluster>* outClusters)
	{
		return SplitMeshIntoClusters(
			nparts, 0, static_cast<int>(inoutTriangles->size()), 
			vertices, inoutTriangles, outClusters);
	}

	int NaniteBuilder::SplitMeshIntoClusters(
		int nparts, int start, int count,
		const std::vector<FVector3> vertices,
		std::vector<Triangle>* inoutTriangles,
		std::vector<Cluster>* outClusters)
	{
		std::vector<Triangle>& triangles = *inoutTriangles;

		// build triangle graph
		std::unordered_map<Edge, std::vector<idx_t>> edgeToTriangles;
		edgeToTriangles.reserve(count * 3);
		for (int triIdx = start; triIdx < start + count; ++triIdx)
		{
			const Triangle& triangle = triangles[triIdx];
			uint32_t i0 = triangle.i0;
			uint32_t i1 = triangle.i1;
			uint32_t i2 = triangle.i2;
			edgeToTriangles[Edge(i0, i1)].push_back(static_cast<idx_t>(triIdx));
			edgeToTriangles[Edge(i1, i2)].push_back(static_cast<idx_t>(triIdx));
			edgeToTriangles[Edge(i2, i0)].push_back(static_cast<idx_t>(triIdx));
		}

		// partition the triangle graph using METIS
		std::vector<std::set<idx_t>> triangleAdj(count);
		for (const auto& [edge, tris] : edgeToTriangles)
		{
			if (tris.size() == 2)
			{
				idx_t t0 = tris[0] - start;
				idx_t t1 = tris[1] - start;
				triangleAdj[t0].insert(t1);
				triangleAdj[t1].insert(t0);
			}
		}

		std::vector<idx_t> xadj;
		std::vector<idx_t> adjncy;

		xadj.push_back(0);
		for (const auto& neighbors : triangleAdj)
		{
			for (auto adjTri : neighbors)
			{
				adjncy.push_back(adjTri);
			}
			xadj.push_back(static_cast<idx_t>(adjncy.size()));
		}

		idx_t ncon = 1;

		idx_t* vwgt = nullptr;
		idx_t* vsize = nullptr;
		idx_t* adjwgt = nullptr;

		std::vector<real_t> tpwgts(nparts, 1.0f / nparts);
		real_t ubvec = { 1.08f };

		idx_t options[METIS_NOPTIONS];
		METIS_SetDefaultOptions(options);
		options[METIS_OPTION_NUMBERING] = 0;

		idx_t objval;
		std::vector<idx_t> partOut;
		partOut.resize(count);

		int result = METIS_PartGraphKway(
			static_cast<idx_t*>(&count), &ncon, xadj.data(), adjncy.data(),
			vwgt, vsize, adjwgt, &nparts,
			tpwgts.data(), &ubvec, options, &objval, partOut.data());

		if (!(result == METIS_OK))
		{
			std::cerr << "METIS partitioning failed with error code " << result << "\n";
			return -1;
		}

		// reorder and cluster triangles based on partitions
		std::vector<std::vector<Triangle>> reorederBuffer(nparts);
		for (int i = 0; i < count; ++i)
		{
			reorederBuffer[partOut[i]].push_back(triangles[start + i]);
		}

		triangles.erase(triangles.begin() + start, triangles.begin() + start + count);
		outClusters->clear();
		outClusters->resize(nparts);
		int insertIndex = start;
		for (int i = 0; i < nparts; ++i)
		{
			triangles.insert(triangles.begin() + insertIndex, reorederBuffer[i].begin(), reorederBuffer[i].end());
			(*outClusters)[i].StartIndex = static_cast<int>(insertIndex);
			(*outClusters)[i].NumTriangles = static_cast<int>(reorederBuffer[i].size());
			(*outClusters)[i].Bounds = computeBoundingBox(vertices, triangles, (*outClusters)[i]);
			insertIndex += static_cast<int>(reorederBuffer[i].size());
		}

		return static_cast<int>(objval);
	}

	int NaniteBuilder::MergeClusters(const std::vector<Cluster>& clusters, Mesh* inoutMesh, std::vector<Cluster>* outClusters)
	{
		const std::vector<FVector3>& vertices = inoutMesh->Vertices;
		std::vector<Triangle>& triangles = inoutMesh->Triangles;

		int numTriangles = static_cast<int>(triangles.size());
		int numClusters = static_cast<int>(clusters.size());
		int mergeGroupSize = std::max(3, std::min(numTriangles / 128, 4));
		int numGroups = static_cast<int>(std::ceilf(static_cast<float>(numClusters) / mergeGroupSize));
		if (numGroups < 2) return -1;

		std::unordered_map<Edge, std::vector<idx_t>> edgeToClusters;
		edgeToClusters.reserve(numTriangles * 3);
		for (int clusterIdx = 0; clusterIdx < clusters.size(); ++clusterIdx)
		{
			const Cluster& cluster = clusters[clusterIdx];
			for (int triIdx = cluster.StartIndex; triIdx < cluster.StartIndex + cluster.NumTriangles; ++triIdx)
			{
				const Triangle& triangle = triangles[triIdx];
				uint32_t i0 = triangle.i0;
				uint32_t i1 = triangle.i1;
				uint32_t i2 = triangle.i2;
				edgeToClusters[Edge(i0, i1)].push_back(static_cast<idx_t>(clusterIdx));
				edgeToClusters[Edge(i1, i2)].push_back(static_cast<idx_t>(clusterIdx));
				edgeToClusters[Edge(i2, i0)].push_back(static_cast<idx_t>(clusterIdx));
			}
		}

		// partition the triangle graph using METIS
		std::vector<std::set<idx_t>> clusterAdj(numClusters);
		for (const auto& [edge, clusterList] : edgeToClusters)
		{
			for (size_t i = 0; i < clusterList.size(); ++i)
			{
				for (size_t j = i + 1; j < clusterList.size(); ++j)
				{
					idx_t c0 = clusterList[i];
					idx_t c1 = clusterList[j];
					if (c0 != c1)
					{
						clusterAdj[c0].insert(c1);
						clusterAdj[c1].insert(c0);
					}
				}
			}
		}

		std::vector<idx_t> xadj;
		std::vector<idx_t> adjncy;

		xadj.push_back(0);
		for (const auto& neighbors : clusterAdj)
		{
			for (auto adjTri : neighbors)
			{
				adjncy.push_back(adjTri);
			}
			xadj.push_back(static_cast<idx_t>(adjncy.size()));
		}

		idx_t ncon = 1;

		idx_t* vwgt = nullptr;
		idx_t* vsize = nullptr;
		idx_t* adjwgt = nullptr;

		std::vector<real_t> tpwgts(numGroups, 1.0f / numGroups);
		real_t ubvec = { 2.0f };

		idx_t options[METIS_NOPTIONS];
		METIS_SetDefaultOptions(options);
		options[METIS_OPTION_NUMBERING] = 0;

		idx_t objval;
		std::vector<idx_t> partOut;
		partOut.resize(numClusters);

		int result = METIS_PartGraphKway(
			static_cast<idx_t*>(&numClusters), &ncon, xadj.data(), adjncy.data(),
			vwgt, vsize, adjwgt, &numGroups,
			tpwgts.data(), &ubvec, options, &objval, partOut.data());

		if (!(result == METIS_OK))
		{
			std::cerr << "METIS partitioning failed with error code " << result << "\n";
			return -1;
		}

		// reorder and cluster triangles based on partitions
		std::vector<std::vector<Triangle>> reorederBuffer(numGroups);
		for (int i = 0; i < numClusters; ++i)
		{
			const Cluster& cluster = clusters[i];
			reorederBuffer[partOut[i]].insert(reorederBuffer[partOut[i]].end(),
				triangles.begin() + cluster.StartIndex, triangles.begin() + cluster.StartIndex + cluster.NumTriangles);
		}

		triangles.clear();
		outClusters->clear();
		outClusters->resize(numGroups);
		for (int i = 0; i < numGroups; ++i)
		{
			triangles.insert(triangles.end(), reorederBuffer[i].begin(), reorederBuffer[i].end());
			(*outClusters)[i].StartIndex = static_cast<int>(triangles.size() - reorederBuffer[i].size());
			(*outClusters)[i].NumTriangles = static_cast<int>(reorederBuffer[i].size());
			(*outClusters)[i].Bounds = computeBoundingBox(vertices, triangles, (*outClusters)[i]);
		}

		return static_cast<int>(objval);
	}

	AABB NaniteBuilder::computeBoundingBox(
		const std::vector<FVector3>& vertices,
		const std::vector<Triangle>& triangles,
		const Cluster& cluster)
	{
		AABB bbox;
		for (int i = cluster.StartIndex; i < cluster.StartIndex + cluster.NumTriangles; ++i)
		{
			const Triangle& triangle = triangles[i];
			bbox.Encapsulate(vertices[triangle.i0]);
			bbox.Encapsulate(vertices[triangle.i1]);
			bbox.Encapsulate(vertices[triangle.i2]);
		}
		return bbox;
	}
}