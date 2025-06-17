#include "NaniteMesh.h"

#include <iostream>
#include <cassert>

#include "../Utils/Utils.h"
#include "../MeshProcessing/MeshClustering.h"
#include "../MeshProcessing/MeshSimplifier.h"

namespace nanite
{
	NaniteMesh::NaniteMesh()
		: mName("")
	{

	}

	NaniteMesh::NaniteMesh(const std::string name)
		: mName(name)
	{

	}

	bool NaniteMesh::Build(const Mesh& originMesh, int leafTriThreshold)
	{
		mLODMeshes.emplace_back(originMesh);

		// Cluster the LOD0 mesh
		std::vector<Cluster> leafClusters = ClusterMesh(originMesh, leafTriThreshold);
		std::vector<NaniteNode>& leafNodes = mNodes.emplace_back();
		for (const Cluster& leafCluster : leafClusters)
		{
			leafNodes.emplace_back(leafCluster);
		}

		const int MAX_CLUSTERS_PER_GROUP = 4;
		while (true)
		{
			Mesh& srcMesh = *mLODMeshes.rbegin();
			std::vector<NaniteNode>& parentNodes = mNodes.emplace_back();
			std::vector<NaniteNode>& childNodes = mNodes[mNodes.size() - 2];

			// Collect child clusters
			std::vector<Cluster> childClusters;
			for (int nodeIdx = 0; nodeIdx < childNodes.size(); ++nodeIdx)
			{
				childClusters.emplace_back(childNodes[nodeIdx].GetClusterData());
			}
			// Make group of N clusters
			std::vector<std::vector<int>> clusterGroups = GroupClusters(srcMesh, childClusters, MAX_CLUSTERS_PER_GROUP);

			// Simplify the groups of clusters
			std::vector<Mesh> simplifiedMeshes;
			simplifiedMeshes.reserve(clusterGroups.size());
			for (const std::vector<int>& clusterIndices : clusterGroups)
			{
				Mesh groupMesh;
				groupMesh.Vertices = srcMesh.Vertices;
				for (int clusterIndex : clusterIndices)
				{
					for (int triIdx : childClusters[clusterIndex].Triangles)
					{
						auto [i0, i1, i2] = srcMesh.GetTriangleIndices(triIdx);
						groupMesh.Indices.emplace_back(i0);
						groupMesh.Indices.emplace_back(i1);
						groupMesh.Indices.emplace_back(i2);
						groupMesh.Normals.emplace_back(srcMesh.Normals[triIdx]);
						groupMesh.Colors.emplace_back(srcMesh.Colors[triIdx]);
					}
				}

				assert(groupMesh.IsManifold());
				int targetTriCount = groupMesh.NumTriangles() / 2;
				int outNumValidTriCount = -1;
				simplifiedMeshes.emplace_back(SimplifyMesh(groupMesh, targetTriCount, &outNumValidTriCount, false));
			}

			// Create the integrated simplified meshes and
			// Separate each simplified mesh into two clusters
			Mesh& integratedMesh = mLODMeshes.emplace_back();
			std::unordered_map<uint32_t, uint32_t> indexMap;
			// Remove invalid vertices and remap indices
			for (uint32_t i = 0; i < simplifiedMeshes[0].Vertices.size(); ++i)
			{
				bool bValid = true;
				for (const Mesh& simplifiedMesh : simplifiedMeshes)
				{
					const FVector3& v = simplifiedMesh.Vertices[i];
					if (v == INVALID_VERTEX)
					{
						bValid = false;
						break;
					}
				}
				if (bValid)
				{
					integratedMesh.Vertices.emplace_back(simplifiedMeshes[0].Vertices[i]);
					indexMap[i] = static_cast<uint32_t>(integratedMesh.Vertices.size() - 1);
				}
			}

			std::vector<std::vector<Cluster>> parentClusters;
			for (const Mesh& simplifiedMesh : simplifiedMeshes)
			{
				int startNumTriangles = integratedMesh.NumTriangles();
				// Fill the triangle informations
				for (uint32_t triIdx = 0; triIdx < static_cast<uint32_t>(simplifiedMesh.NumTriangles()); ++triIdx)
				{
					auto indices = simplifiedMesh.GetTriangleIndices(triIdx);
					if (indices == INVALID_TRIANGLE) continue;
					auto [i0, i1, i2] = indices;
					integratedMesh.Indices.emplace_back(indexMap[i0]);
					integratedMesh.Indices.emplace_back(indexMap[i1]);
					integratedMesh.Indices.emplace_back(indexMap[i2]);
					integratedMesh.Normals.emplace_back(simplifiedMesh.Normals[triIdx]);
					integratedMesh.Colors.emplace_back(simplifiedMesh.Colors[triIdx]);
				}
				int endNumTriangles = integratedMesh.NumTriangles();
				// Separate simplified mesh into two clusters
				std::vector<Cluster> subClusters = ClusterMesh(
					integratedMesh.CreateSubMesh(startNumTriangles, endNumTriangles, false),
					leafTriThreshold, 2);
				// Set parent clusters buffer
				for (Cluster& cluster : subClusters)
				{
					cluster.Mesh = &integratedMesh;
					for (int& triIdx : cluster.Triangles) triIdx += startNumTriangles;
				}
				parentClusters.emplace_back(std::move(subClusters));
			}

			integratedMesh.ComputeNormals();

			// If there's only one group or
			// If lod mesh is non-manifold
			// exit the loop
			if (clusterGroups.size() == 1 || !integratedMesh.IsManifold())
			{
				Mesh& rootMesh = integratedMesh;
				Cluster rootCluster;
				rootCluster.Mesh = &rootMesh;
				for (int triIdx = 0; triIdx < rootMesh.NumTriangles(); ++triIdx)
				{
					rootCluster.Triangles.emplace_back(triIdx);
					auto [v0, v1, v2] = rootMesh.GetTriangleVertices(triIdx);
					rootCluster.Bounds.Encapsulate(v0);
					rootCluster.Bounds.Encapsulate(v1);
					rootCluster.Bounds.Encapsulate(v2);
				}
				NaniteNode& rootNode = parentNodes.emplace_back(rootCluster);
				for (NaniteNode& childNode : childNodes) rootNode.AddChild(&childNode);

				break;
			}

			// Create a new level of parent nodes in the hierarchy
			for (int parentIndex = 0; parentIndex < clusterGroups.size(); ++parentIndex)
			{
				// The order child clusters corresponds to the clusters held by each child nodes.
				// so clusterGroups also represents the child node indices of each group.
				// and group is separated to two clusters, which are the parents.
				std::vector<int> childIndices = clusterGroups[parentIndex];
				for (const Cluster& parentCluster : parentClusters[parentIndex])
				{
					NaniteNode& parentNode = parentNodes.emplace_back(parentCluster);
					for (int childIndex : childIndices)
					{
						parentNode.AddChild(&childNodes[childIndex]);
					}
				}
			}
		}

		return true;
	}

	void NaniteMesh::PaintByCluster()
	{
		for (int level = 0; level < GetLODDepth(); ++level)
		{
			Mesh& mesh = mLODMeshes[level];
			std::vector<Cluster> clusters;
			for (const NaniteNode& node : mNodes[level])
			{
				clusters.emplace_back(node.GetClusterData());
			}

			utils::PaintMeshByCluster(&mesh, clusters , 12);
		}
	}

	bool NaniteMesh::Save(const std::string& path) const
	{
		const NaniteNode& root = GetRootNode();
		return true;
	}
}