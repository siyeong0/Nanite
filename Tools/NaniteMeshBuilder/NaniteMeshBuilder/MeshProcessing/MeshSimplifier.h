#pragma once
#include "../Topology/Mesh.h"

namespace nanite
{
	const FVector3 INVALID_VERTEX = FVector3{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
	const std::tuple<uint32_t, uint32_t, uint32_t> INVALID_TRIANGLE = std::make_tuple(std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max());

	 Mesh SimplifyMesh(const Mesh& mesh, int targetTriangleCount, int* outNumValidTriangles = nullptr, bool bOrganize = true);
}