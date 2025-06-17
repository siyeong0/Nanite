#include "MeshClustering.h"

#include <iostream>
#include <cassert>
#include <vector>
#include <ranges>
#include <set>
#include <unordered_map>

#include <metis.h>

#include "../Utils/Utils.h"

namespace nanite
{
	static std::vector<int> partGraph(
		const std::vector<std::set<idx_t>>& adjacencyList,
		int numNodes, int numParts, float imbalanceRatio)
	{
		idx_t nvtxs = static_cast<idx_t>(numNodes); // Number of nodes
		idx_t ncon = 1; // Number of weights in each node
		std::vector<idx_t> xadj; // Start index of each node
		std::vector<idx_t> adjncy; // Adjacent list
		std::vector<real_t> tpwgts(numParts, 1.0f / numParts); // Target weight of each partition
		real_t ubvec = { imbalanceRatio }; // Allowed imbalance ratio
		idx_t options[METIS_NOPTIONS]; // Options
		METIS_SetDefaultOptions(options);
		options[METIS_OPTION_NUMBERING] = 0; // Index starts with 0

		// Fill buffers
		xadj.reserve(numNodes + 1);
		adjncy.reserve(adjacencyList.size() * 3);

		xadj.emplace_back(0); // Starts with 0
		for (int triIdx = 0; triIdx < adjacencyList.size(); ++triIdx)
		{
			const auto& neighbors = adjacencyList[triIdx];
			for (auto adjLink : neighbors)
			{
				adjncy.emplace_back(adjLink); // Index
			}
			xadj.emplace_back(static_cast<idx_t>(adjncy.size()));
		}

		// METIS part graph
		idx_t objvalOut; // cost of cutting
		std::vector<idx_t> partOut(numNodes); // Partition data

		int result = METIS_PartGraphKway(
			&nvtxs,
			&ncon,
			xadj.data(),
			adjncy.data(),
			nullptr, // Weight of each node. NULL means all nodes have the same weight
			nullptr, // Memory size of each node. NULL means all nodes have the same size
			nullptr, // Weight of each edge. NULL means all edges have the same weight
			&numParts,
			tpwgts.data(),
			&ubvec,
			options,
			&objvalOut, partOut.data());

		std::vector<int> resultParts;
		if (!(result == METIS_OK))
		{
			std::cerr << "METIS partitioning failed with error code " << result << "\n";
			return resultParts;
		}
		resultParts.reserve(partOut.size());

		for (int i = 0; i < partOut.size(); ++i)
		{
			resultParts.emplace_back(static_cast<int>(partOut[i]));
		}

		return resultParts;
	}

	std::vector<Cluster> PartMesh(const Mesh& mesh, int numParts, float imbalanceRatio)
	{
		const int numTriangles = mesh.NumTriangles();

		// Create a map to store edges and the triangles they belong to
		std::unordered_map<Edge, std::vector<idx_t>> edgeToTriangelesMap;
		edgeToTriangelesMap.reserve(utils::NextPrime(2 * (numTriangles * 3) + 1));
		for (int triIdx = 0; triIdx < mesh.NumTriangles(); ++triIdx)
		{
			auto [e0, e1, e2] = mesh.GetTriangleEdges(triIdx);
			edgeToTriangelesMap[e0].emplace_back(static_cast<idx_t>(triIdx));
			edgeToTriangelesMap[e1].emplace_back(static_cast<idx_t>(triIdx));
			edgeToTriangelesMap[e2].emplace_back(static_cast<idx_t>(triIdx));
		}

		// Build the adjacency list between triangles sharing edges
		std::vector<std::set<idx_t>> triangsAdjacencyList(numTriangles);
		for (const auto& [edge, triIdxs] : edgeToTriangelesMap)
		{
			for (size_t i = 0; i < triIdxs.size(); ++i)
			{
				for (size_t j = i + 1; j < triIdxs.size(); ++j)
				{
					idx_t t0 = triIdxs[i];
					idx_t t1 = triIdxs[j];
					triangsAdjacencyList[t0].insert(t1);
					triangsAdjacencyList[t1].insert(t0);
				}
			}
		}

		// Partton the graph of triangles
		std::vector<int> parts = partGraph(triangsAdjacencyList, numTriangles, numParts, imbalanceRatio);

		// Create clusters based on the partitioning
		std::vector<Cluster> resultClusters(numParts);
		for (int triIdx = 0; triIdx < mesh.NumTriangles(); ++triIdx)
		{
			Cluster& resultCluster = resultClusters[parts[triIdx]];
			resultCluster.Mesh = &mesh;
			resultCluster.Triangles.emplace_back(triIdx);
			auto [v0, v1, v2] = mesh.GetTriangleVertices(triIdx);
			resultCluster.Bounds.Encapsulate(v0);
			resultCluster.Bounds.Encapsulate(v1);
			resultCluster.Bounds.Encapsulate(v2);
		}

		return resultClusters;
	}

	std::vector<Cluster> PartCluster(const Cluster& cluster, int numParts, float imbalanceRatio)
	{
		assert(numParts > 0);
		std::vector<Cluster> resultClusters;
		if (numParts == 1)
		{
			// Returns a input cluster
			resultClusters.emplace_back(cluster);
			return resultClusters;
		}

		const int numTriangles = static_cast<int>(cluster.Triangles.size());
		// Create a map to store edges and the triangles they belong to
		std::unordered_map<Edge, std::vector<idx_t>> edgeToTriangelesMap;
		edgeToTriangelesMap.reserve(utils::NextPrime(2 * (numTriangles * 3) + 1));
		for (int triIdx : cluster.Triangles)
		{
			auto [e0, e1, e2] = cluster.Mesh->GetTriangleEdges(triIdx);
			edgeToTriangelesMap[e0].emplace_back(static_cast<idx_t>(triIdx));
			edgeToTriangelesMap[e1].emplace_back(static_cast<idx_t>(triIdx));
			edgeToTriangelesMap[e2].emplace_back(static_cast<idx_t>(triIdx));
		}

		// Build the adjacency list between triangles sharing edges
		// It uses a map to keep track of the index of each triangle in the adjacency list
		std::unordered_map<int, int> triangleToIndexMap;
		std::vector<std::set<idx_t>> triangsAdjacencyList;
		for (const auto& [edge, triIdxs] : edgeToTriangelesMap)
		{
			for (size_t i = 0; i < triIdxs.size(); ++i)
			{
				for (size_t j = i + 1; j < triIdxs.size(); ++j)
				{
					idx_t t0 = triIdxs[i];
					idx_t t1 = triIdxs[j];

					if (triangleToIndexMap.find(t0) == triangleToIndexMap.end())
					{
						int index = static_cast<int>(triangsAdjacencyList.size());
						triangleToIndexMap[t0] = index;
						triangsAdjacencyList.emplace_back();
					}
					if (triangleToIndexMap.find(t1) == triangleToIndexMap.end())
					{
						int index = static_cast<int>(triangsAdjacencyList.size());
						triangleToIndexMap[t1] = index;
						triangsAdjacencyList.emplace_back();
					}

					triangsAdjacencyList[triangleToIndexMap[t0]].insert(triangleToIndexMap[t1]);
					triangsAdjacencyList[triangleToIndexMap[t1]].insert(triangleToIndexMap[t0]);
				}
			}
		}

		// Partton the graph of triangles
		std::vector<int> parts = partGraph(triangsAdjacencyList, numTriangles, numParts, imbalanceRatio);

		// Create clusters based on the partitioning
		resultClusters.resize(numParts);
		for (int triIdx : cluster.Triangles)
		{
			Cluster& resultCluster = resultClusters[parts[triangleToIndexMap[triIdx]]];
			resultCluster.Mesh = cluster.Mesh;
			resultCluster.Triangles.emplace_back(triIdx);
			auto [v0, v1, v2] = cluster.Mesh->GetTriangleVertices(triIdx);
			resultCluster.Bounds.Encapsulate(v0);
			resultCluster.Bounds.Encapsulate(v1);
			resultCluster.Bounds.Encapsulate(v2);
		}

		return resultClusters;
	}

	std::vector<Cluster> ClusterMesh(const Mesh& mesh, int maxNumTrianglesInCluster, int maxNumCluster)
	{
		float IMBALACNE_RATIO = 1.2f; // Allow 20% imbalance
		int numPartitions = static_cast<int>(std::ceilf((float)mesh.NumTriangles() / maxNumTrianglesInCluster) * IMBALACNE_RATIO);
		if (maxNumCluster >= 2)
		{
			numPartitions = std::max(numPartitions, maxNumCluster);
		}
		std::vector<Cluster> resultClusters;

		if (numPartitions < 2)
		{
			// Returns a single cluster containing all triangles
			std::ranges::iota_view range{ 0, mesh.NumTriangles() };
			resultClusters.emplace_back(Cluster{ &mesh, {range.begin(), range.end()}, utils::ComputeBoundingBox(mesh.Vertices) });
			return resultClusters;
		}

		// Part the mesh into clusters
		std::vector<Cluster> clusters = nanite::PartMesh(mesh, numPartitions, IMBALACNE_RATIO);

		// If the number of triangles in a cluster is more than the maximum allowed,
		// partition the cluster into smaller clusters
		resultClusters.reserve(static_cast<size_t>(numPartitions * 1.2f));
		for (const Cluster& cluster : clusters)
		{
			if (cluster.Triangles.size() < maxNumTrianglesInCluster)
			{
				resultClusters.emplace_back(cluster);
			}
			else
			{
				int numSubPartitions = static_cast<int>(std::ceilf((float)cluster.Triangles.size() / maxNumTrianglesInCluster) * IMBALACNE_RATIO);
				std::vector<Cluster> subClusters = PartCluster(cluster, numSubPartitions, IMBALACNE_RATIO);
				resultClusters.insert(resultClusters.end(), subClusters.begin(), subClusters.end());
			}
		}

		// TODO: Print to dbg stream
		size_t maxClusterSize = 0;
		size_t minClusterSize = std::numeric_limits<size_t>::max();
		float maxAABBVolume = 0.0f;
		float minAABBVolume = std::numeric_limits<float>::max();
		for (const Cluster& cluster : resultClusters)
		{
			maxClusterSize = std::max(maxClusterSize, cluster.Triangles.size());
			minClusterSize = std::min(minClusterSize, cluster.Triangles.size());
			float aabbVolume = cluster.Bounds.Volume();
			maxAABBVolume = std::max(maxAABBVolume, aabbVolume);
			minAABBVolume = std::min(minAABBVolume, aabbVolume);
		}

		//std::cout << "Clustering results:\n"
		//	<< "  Number of clusters: " << resultClusters.size() << "\n"
		//	<< "  Max cluster size: " << maxClusterSize << "\n"
		//	<< "  Min cluster size: " << minClusterSize << "\n"
		//	<< "  Max AABB volume: " << maxAABBVolume << "\n"
		//	<< "  Min AABB volume: " << minAABBVolume << "\n\n";

		return resultClusters;
	}

	std::vector<std::vector<int>> GroupClusters(const Mesh& mesh, const std::vector<Cluster>& clusters, int maxNumClustersPerGroup)
	{
		const int numClusters = static_cast<int>(clusters.size());
		float IMBALACNE_RATIO = 1.0f; // Forbid imbalance
		int numPartitions = static_cast<int>(std::ceilf((float)numClusters / maxNumClustersPerGroup) * IMBALACNE_RATIO);
		std::vector<std::vector<int>> resultGroups;

		if (numPartitions == 1)
		{
			std::vector<int> all;
			for (int i = 0; i < static_cast<int>(clusters.size()); ++i) all.emplace_back(i);
			resultGroups.emplace_back(all);
			return resultGroups;
		}

		// Create a map to store edges and the clusters they belong to
		std::unordered_map<Edge, std::vector<idx_t>> edgeToClustersMap;
		edgeToClustersMap.reserve(utils::NextPrime(2 * (mesh.NumTriangles() * 3) + 1));
		for (int clusterIdx = 0; clusterIdx < numClusters; ++clusterIdx)
		{
			const Cluster& cluster = clusters[clusterIdx];
			for (int triIdx : cluster.Triangles)
			{
				auto [e0, e1, e2] = mesh.GetTriangleEdges(triIdx);
				edgeToClustersMap[e0].emplace_back(static_cast<idx_t>(clusterIdx));
				edgeToClustersMap[e1].emplace_back(static_cast<idx_t>(clusterIdx));
				edgeToClustersMap[e2].emplace_back(static_cast<idx_t>(clusterIdx));
			}
		}

		// Build the adjacency list between clusters sharing edges
		std::vector<std::set<idx_t>> clusterAdjacencyList(numClusters);
		for (const auto& [edge, clusterIdxs] : edgeToClustersMap)
		{
			// TODO: assertion failed
			// assert(clusterIdxs.size() <= 2); // Each edge can only belong to two clusters
			for (size_t i = 0; i < clusterIdxs.size(); ++i)
			{
				for (size_t j = i + 1; j < clusterIdxs.size(); ++j)
				{
					idx_t c0 = clusterIdxs[i];
					idx_t c1 = clusterIdxs[j];
					clusterAdjacencyList[c0].emplace(c1);
					clusterAdjacencyList[c1].emplace(c0);
				}
			}
		}

		// Part the graph of triangles
		std::vector<int> parts = partGraph(clusterAdjacencyList, numClusters, numPartitions, IMBALACNE_RATIO);

		// Create groups based on the partitioning
		resultGroups.resize(numPartitions);
		for (int clusterIdx = 0; clusterIdx < numClusters; ++clusterIdx)
		{
			resultGroups[parts[clusterIdx]].emplace_back(clusterIdx);
		}

		return resultGroups;
	}
}