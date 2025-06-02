#pragma once
#include <vector>

#include <metis.h>

#include "../Math/Math.h"
#include "../Topology/Cluster.h"
#include "NaniteMesh.h"

namespace nanite
{
	class NaniteBuilder
	{
	public:
		static int SplitMeshIntoClusters(
			int nparts, Mesh* inoutMesh, std::vector<Cluster>* outClusters);
		static int SplitMeshIntoClusters(
			int nparts, 
			const std::vector<FVector3> vertices, 
			std::vector<Triangle>* inoutTriangles,
			std::vector<Cluster>* outClusters);
		static int SplitMeshIntoClusters(
			int nparts, int start, int count,
			const std::vector<FVector3> vertices,
			std::vector<Triangle>* inoutTriangles,
			std::vector<Cluster>* outClusters);

		static int MergeClusters(const std::vector<Cluster>& clusters, Mesh* inoutMesh, std::vector<Cluster>* outClusters);

	private:
		static inline float castToWeight(float w) { return static_cast<idx_t>(w * 10000.f); };

		static AABB computeBoundingBox(
			const std::vector<FVector3>& vertices,
			const std::vector<Triangle>& triangles,
			const Cluster& cluster);
	};
}