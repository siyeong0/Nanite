#pragma once
#include "../Math/AABB.hpp"
namespace nanite
{
	struct Cluster
	{
		int StartIndex = 0; // start index in the triangle list
		int NumTriangles = 0; // number of triangles in this cluster
		AABB Bounds;
	};
}