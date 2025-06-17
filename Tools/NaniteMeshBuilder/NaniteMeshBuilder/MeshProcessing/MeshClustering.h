#pragma once

#include "../Topology/Mesh.h"
#include "../Topology/Cluster.h"

namespace nanite
{
	std::vector<Cluster> PartMesh(const Mesh& mesh, int numParts, float imbalanceRatio);
	std::vector<Cluster> PartCluster(const Cluster& cluster, int numParts, float imbalanceRatio);
	std::vector<Cluster> ClusterMesh(const Mesh& mesh, int maxNumTrianglesInCluster, int maxNumCluster = -1);
	std::vector<std::vector<int>> GroupClusters(const Mesh& mesh, const std::vector<Cluster>& clusters, int maxNumClustersPerGroup);
}