#pragma once
#include "../Topology/Mesh.h"

namespace nanite
{
	class MeshSimplifier
	{
	public:
		MeshSimplifier() = default;
		~MeshSimplifier() = default;

		Mesh SimplifyMesh(const Mesh& mesh, int targetTriangleCount) const;

	private:

	};
}