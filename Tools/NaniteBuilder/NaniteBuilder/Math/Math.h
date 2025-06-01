#pragma once
#include "FVector3.hpp"
#include "FVector4.hpp"
#include "FMatrix3x3.h"
#include "FMatrix4x4.h"
#include "AABB.hpp"

namespace nanite
{
	inline FVector3 operator*(const FMatrix3x3& mat, const FVector3& vec)
	{
		return FVector3(
			mat[0][0] * vec.x + mat[0][1] * vec.y + mat[0][2] * vec.z,
			mat[1][0] * vec.x + mat[1][1] * vec.y + mat[1][2] * vec.z,
			mat[2][0] * vec.x + mat[2][1] * vec.y + mat[2][2] * vec.z
		);
	}
}