#pragma once

#include "../../Topology/Mesh.h"
#include "../../Topology/Cluster.h"

namespace nanite
{
	std::vector<Cluster> ClusterMesh(const Mesh& mesh, int maxNumTrianglesInCluster);
}