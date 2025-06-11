#pragma once
#include <vector>

#include "../Math/Math.h"

namespace nanite
{
	namespace utils
	{
		struct FVector3Hasher
		{
			size_t operator()(const FVector3& v) const
			{
				const int scale = 1000;
				size_t hx = std::hash<int>()(static_cast<int>(v.x * scale));
				size_t hy = std::hash<int>()(static_cast<int>(v.y * scale));
				size_t hz = std::hash<int>()(static_cast<int>(v.z * scale));
				return hx ^ (hy << 1) ^ (hz << 2);
			}
		};

		inline FVector3 ComputeNormal(const FVector3& v0, const FVector3& v1, const FVector3& v2)
		{
			return (v1 - v0).Cross(v2 - v0).Normalized();
		}

		inline FVector3 ComputeNormal(const std::tuple<const FVector3&, const FVector3&, const FVector3&>& verts)
		{
			return ComputeNormal(std::get<0>(verts), std::get<1>(verts), std::get<2>(verts));
		}

		inline float ComputeArea(const FVector3& v0, const FVector3& v1, const FVector3& v2)
		{
			return 0.5f * (v1 - v0).Cross(v2 - v0).Length();
		}

		inline AABB ComputeBoundingBox(const std::vector<FVector3>& vertices)
		{
			AABB aabb;
			for (const FVector3& v : vertices)
			{
				aabb.Encapsulate(v);
			}
			return aabb;
		}
	}
}