#pragma once
#include "../Math/AABB.hpp"
#include "Mesh.h"

namespace nanite
{
	struct Cluster
	{
		const Mesh* Mesh = nullptr;
		std::vector<int> Triangles;
		AABB Bounds;
	};
}