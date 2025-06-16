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
				groupMesh.RemoveUnusedVertices();
				simplifiedMeshes.emplace_back(SimplifyMesh(groupMesh, groupMesh.NumTriangles() / 2));
			}

			// Create the integrated simplified meshes and
			// Separate each simplified mesh into two clusters
			Mesh& integratedMesh = mLODMeshes.emplace_back();
			std::vector<std::vector<Cluster>> parentClusters;
			for (Mesh& simplifiedMesh : simplifiedMeshes)
			{
				// Separate simplified mesh into two clusters
				std::vector<Cluster> subClusters = ClusterMesh(simplifiedMesh, leafTriThreshold);
				for (Cluster& cluster : subClusters)
				{
					cluster.Mesh = &integratedMesh;
					for (int& triIdx : cluster.Triangles) triIdx += integratedMesh.NumTriangles();
				}
				parentClusters.emplace_back(std::move(subClusters));
				// Merge the simplified meshes
				for (uint32_t& i : simplifiedMesh.Indices) i += integratedMesh.NumVertices();
				integratedMesh.Vertices.insert(integratedMesh.Vertices.end(), simplifiedMesh.Vertices.begin(), simplifiedMesh.Vertices.end());
				integratedMesh.Indices.insert(integratedMesh.Indices.end(), simplifiedMesh.Indices.begin(), simplifiedMesh.Indices.end());
				integratedMesh.Normals.insert(integratedMesh.Normals.end(), simplifiedMesh.Normals.begin(), simplifiedMesh.Normals.end());
				integratedMesh.Colors.insert(integratedMesh.Colors.end(), simplifiedMesh.Colors.begin(), simplifiedMesh.Colors.end());
			}
			integratedMesh.MergeDuplicatedVertices();

			// If there's only one group, exit the loop
			if (clusterGroups.size() == 1)
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
}