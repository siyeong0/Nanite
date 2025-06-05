#pragma once
#include <vector>

#include "../Math/Math.h"

namespace nanite
{
	namespace utils
	{
		struct FVector3Hasher
		{
			size_t operator()(const FVector3& v) const;
		};

		void MergeDuplicatedVertices(
			const std::vector<FVector3>& vertices, const std::vector<uint32_t>& indices,
			std::vector<FVector3>* outVertices, std::vector<uint32_t>* outIndices);

		AABB ComputeBoundingBox(const std::vector<FVector3>& vertices);
	}
}