#pragma once

#include "../../Math/Math.h"

namespace nanite
{
	namespace qem
	{
		struct Quadric
		{
			FMatrix4x4 Q;

			Quadric() : Q(FMatrix4x4::Zero()) {}

			inline void AddPlane(const FVector3& normal, float d)
			{
				FVector4 p(normal.x, normal.y, normal.z, d);
				for (int i = 0; i < 4; ++i)
				{
					for (int j = 0; j < 4; ++j)
					{
						Q[i][j] += p[i] * p[j];
					}
				}
			}

			inline float Evaluate(const FVector4& v) const
			{
				float result = 0.0f;
				for (int i = 0; i < 4; ++i)
				{
					for (int j = 0; j < 4; ++j)
					{
						result += v[i] * Q[i][j] * v[j];
					}
				}
				return result;
			}
		};
	}
}