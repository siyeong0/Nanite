#pragma once
#include <cmath>

#include "FVEctor3.hpp"

namespace nanite
{
	struct AABB
	{
		FVector3 Min;
		FVector3 Max;

		AABB() : Min(FVector3::FMaxValue()), Max(FVector3::FMinValue()) {};
		AABB(const FVector3& min, const FVector3& max) : Min(min), Max(max) {}

		inline FVector3 Center() const { return (Min + Max) * 0.5f; }
		inline FVector3 Size() const { return Max - Min; }
		inline FVector3 Extents() const { return Size() * 0.5f; }
		inline float Volume() const { FVector3 size = Size(); return size.x * size.y * size.z; }

		inline void Encapsulate(const FVector3& point)
		{
			Min = FVector3::Min(Min, point);
			Max = FVector3::Max(Max, point);
		}

		inline void Encapsulate(const AABB& other)
		{
			Encapsulate(other.Min);
			Encapsulate(other.Max);
		}
	};
}