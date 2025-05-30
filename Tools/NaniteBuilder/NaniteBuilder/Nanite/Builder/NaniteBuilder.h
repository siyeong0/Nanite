#pragma once
#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>

#include <metis.h>

#include "../../Math/Math.h"
#include "../NaniteMesh/NaniteMesh.h"
#include "Edge.h"
#include "../NaniteMesh/Geometry/Cluster.h"

namespace nanite
{
	class NaniteBuilder
	{
	public:
		static int BuildGraph(NaniteMesh& mesh, int nparts, std::vector<Cluster>& outClusters);

	private:
		static AABB ComputeBoundingBox(
			const std::vector<FVector3>& vertices,
			const std::vector<Triangle>& triangles,
			int start, int count);
	};
}