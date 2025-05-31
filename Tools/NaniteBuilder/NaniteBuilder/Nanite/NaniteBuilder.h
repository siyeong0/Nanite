#pragma once
#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>

#include <metis.h>

#include "../Math/Math.h"
#include "../Topology/Edge.h"
#include "../Topology/Cluster.h"
#include "NaniteMesh.h"

namespace nanite
{
	class NaniteBuilder
	{
	public:
		static int SplitMeshIntoClusters(int nparts, Mesh* inoutMesh, std::vector<Cluster>* outClusters);
		static int MergeClusters(const std::vector<Cluster>& clusters, Mesh* inoutMesh, std::vector<Cluster>* outClusters);

	private:
		static AABB computeBoundingBox(
			const std::vector<FVector3>& vertices,
			const std::vector<Triangle>& triangles,
			const Cluster& cluster);
	};
}