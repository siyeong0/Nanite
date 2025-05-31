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
		NaniteBuilder(const std::vector<FVector3>& vertices, std::vector<Triangle>& triangles);
		~NaniteBuilder() = default;


		int BuildGraph(int nparts,
			std::vector<Triangle>& triangles,
			std::vector<Cluster>& outClusters);
		int MergeClusters(
			const std::vector<Cluster>& clusters,
			const std::vector<Triangle>& triangles,
			std::vector<idx_t>& outMergedClusters);

	private:
		static AABB computeBoundingBox(
			const std::vector<FVector3>& vertices,
			const std::vector<Triangle>& triangles,
			const Cluster& cluster);

	private:
		const std::vector<FVector3>& mVertices;
	};
}