#pragma once
#include "../../Topology/Mesh.h"

namespace nanite
{
	 Mesh SimplifyMesh(const Mesh& mesh, int targetTriangleCount);
}