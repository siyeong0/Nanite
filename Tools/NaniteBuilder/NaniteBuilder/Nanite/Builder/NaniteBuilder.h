#pragma once
#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>

#include <metis.h>

#include "../../Math/Math.h"
#include "../NaniteMesh/NaniteMesh.h"
#include "Edge.h"

namespace nanite
{
	class NaniteBuilder
	{
	public:
		static int BuildGraph(const NaniteMesh& mesh, int nparts, std::vector<int32_t>& partOut);
	};
}