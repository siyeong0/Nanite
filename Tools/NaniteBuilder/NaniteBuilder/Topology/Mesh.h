#pragma once
#include <vector>

#include "../Math/Math.h"
#include "Triangle.h"

namespace nanite
{
	struct Mesh
	{
		std::vector<FVector3> Vertices;
		std::vector<Triangle> Triangles;

		Mesh() = default;
		~Mesh() = default;
		Mesh(const Mesh& other) : Vertices(other.Vertices), Triangles(other.Triangles) {}
		Mesh(Mesh&& other) noexcept 
			: Vertices(std::move(other.Vertices)), Triangles(std::move(other.Triangles)) {}
		Mesh& operator=(const Mesh& other)
		{
			Vertices = other.Vertices;
			Triangles = other.Triangles;
			return *this;
		}
	};
}