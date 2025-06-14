#include "NaniteMesh.h"

#include <iostream>
#include <cassert>

#include "../Utils/Utils.h"
#include "../MeshProcessing/MeshClustering.h"
#include "../MeshProcessing/MeshSimplifier.h"

namespace nanite
{
	bool NaniteMesh::Build(const Mesh& originMesh, int leafTriThreshold)
	{
		Mesh srcMesh(originMesh);
		int numLeafClusters = static_cast<int>(std::ceilf((float)originMesh.NumTriangles() / leafTriThreshold));
		std::vector<Cluster> clusters = ClusterMesh(srcMesh, numLeafClusters);



		return false;
	}
}