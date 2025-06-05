#pragma once
#include "../Math/Math.h"

namespace nanite
{
	struct Cluster
	{
		AABB Bounds;
		int StartIndex = 0; // start index in the triangle list
		int NumTriangles = 0; // number of triangles in this cluster
	};
}