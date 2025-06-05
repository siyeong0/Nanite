#pragma once
#include "../Math/Math.h"

namespace nanite
{
	struct Quadric
	{
		FMatrix4x4 Q;
		Quadric() : Q(FMatrix4x4::Zero()) {}

		inline void AddPlane(const FVector3& normal, float d);
		inline void RemovePlane(const FVector3& normal, float d);
		inline float Evaluate(const FVector4& v) const;
		inline float Evaluate(const FVector3& vec3) const;
	};

	inline void Quadric::AddPlane(const FVector3& normal, float d)
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

	inline void Quadric::RemovePlane(const FVector3& normal, float d)
	{
		FVector4 p(normal.x, normal.y, normal.z, d);
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				Q[i][j] -= p[i] * p[j];
			}
		}
	}

	inline float Quadric::Evaluate(const FVector4& v) const
	{
		float result = 0.0f;
		FVector4 qv;
		for (int i = 0; i < 4; ++i)
		{
			qv[i] = Q[i][0] * v[0] + Q[i][1] * v[1] + Q[i][2] * v[2] + Q[i][3] * v[3];
		}
		return v.Dot(qv);
	}

	inline float Quadric::Evaluate(const FVector3& vec3) const
	{
		const FVector4 vec = FVector4(vec3.x, vec3.y, vec3.z, 1.f);
		return Evaluate(vec);
	}
}