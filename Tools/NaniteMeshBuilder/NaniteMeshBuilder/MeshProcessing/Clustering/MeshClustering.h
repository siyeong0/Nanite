#pragma once

#include "../../Topology/Mesh.h"

namespace nanite
{
	std::vector<int> PartMesh(const Mesh& mesh, int numParts);

}