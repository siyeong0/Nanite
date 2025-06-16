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
		mLODMeshes.rbegin()->Name = mName + "_LOD0";
		Mesh& srcMesh = *mLODMeshes.rbegin();

		// Cluster the LOD0 mesh
		std::vector<Cluster> leafClusters = ClusterMesh(srcMesh, leafTriThreshold);
		std::vector<NaniteNode> leafNodes(leafClusters.size());
		for (int i = 0; i < leafClusters.size(); ++i)
		{
			leafNodes[i].SetClusterData(leafClusters[i]);
		}

		// Make group of N clusters
		const int maxClustersPerGroup = 4;
		std::vector<std::vector<int>> groups = GroupClusters(srcMesh, leafClusters, maxClustersPerGroup);

		// Simplify the groups of clusters
		std::vector<Mesh> simplifiedMeshes;
		simplifiedMeshes.reserve(groups.size());
		for (const std::vector<int>& group : groups)
		{
			Mesh groupMesh;
			groupMesh.Vertices = srcMesh.Vertices;
			for (int clusterIndex : group)
			{
				for (int triIdx : leafClusters[clusterIndex].Triangles)
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

		// Create the integrated simplified mesh and
		// partition each each mesh into two clusters
		Mesh& integratedMesh = mLODMeshes.emplace_back();
		//std::vector<Cluster> parentClusters;
		for (Mesh& simplifiedMesh : simplifiedMeshes)
		{
			// Create two clusters for each simplified mesh
			//std::vector<Cluster> clusters = ClusterMesh(simplifiedMesh, leafTriThreshold);
			//for (Cluster& cluster : clusters)
			//{
			//	cluster.Mesh = &integratedMesh;
			//	for (int& triIdx : cluster.Triangles)
			//	{
			//		triIdx += integratedMesh.NumTriangles();
			//	}
			//}
			//parentClusters.insert(parentClusters.end(), clusters.begin(), clusters.end());
			// Merge the simplified meshes
			for (uint32_t& i : simplifiedMesh.Indices) i += integratedMesh.NumVertices();
			integratedMesh.Vertices.insert(integratedMesh.Vertices.end(), simplifiedMesh.Vertices.begin(), simplifiedMesh.Vertices.end());
			integratedMesh.Indices.insert(integratedMesh.Indices.end(), simplifiedMesh.Indices.begin(), simplifiedMesh.Indices.end());
			integratedMesh.Normals.insert(integratedMesh.Normals.end(), simplifiedMesh.Normals.begin(), simplifiedMesh.Normals.end());
			integratedMesh.Colors.insert(integratedMesh.Colors.end(), simplifiedMesh.Colors.begin(), simplifiedMesh.Colors.end());
		}
		integratedMesh.MergeDuplicatedVertices();

		mLODMeshes.rbegin()->Name = mName + "_LOD1";

		return false;
	}
}