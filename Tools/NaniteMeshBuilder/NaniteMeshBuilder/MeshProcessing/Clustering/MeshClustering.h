#pragma once

#include "../../Topology/Mesh.h"
#include "../../Topology/Cluster.h"

namespace nanite
{
	std::vector<int> PartMesh(const Mesh& mesh, int numParts);
	std::vector<Cluster> ClusterAndReorderMesh(Mesh* mesh, int numClusters);
	std::vector<Cluster> ClusterAndReorderMesh (Mesh* mesh, const std::vector<int>& parts = {}, int numClusters = -1);
}