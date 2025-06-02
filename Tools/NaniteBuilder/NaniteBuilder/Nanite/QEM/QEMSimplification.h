#pragma once
#include <vector>

#include "../../Math/Math.h"
#include "../../Topology/Triangle.h"

namespace nanite
{
	namespace qem
	{
		std::pair<std::vector<FVector3>, std::vector<Triangle>> SimplifyMesh(
			int targetTriangleCount,
			const std::vector<Triangle>& triangles,
			const std::vector<FVector3>& vertices);

	}
}