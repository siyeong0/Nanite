#pragma once
#include <cstdint>
#include "../Math/Math.h"

namespace nanite
{
	struct Triangle
	{
		uint32_t i0, i1, i2;
		FVector3 Normal;
		FVector3 Color;
	};
}