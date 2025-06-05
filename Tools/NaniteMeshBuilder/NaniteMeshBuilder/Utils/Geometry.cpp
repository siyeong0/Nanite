#include "Geometry.h"

#include <vector>
#include <unordered_map>

#include "../Math/Math.h"

namespace nanite
{
	namespace utils
	{
		size_t FVector3Hasher::operator()(const FVector3& v) const
		{
			const int scale = 1000;
			size_t hx = std::hash<int>()(static_cast<int>(v.x * scale));
			size_t hy = std::hash<int>()(static_cast<int>(v.y * scale));
			size_t hz = std::hash<int>()(static_cast<int>(v.z * scale));
			return hx ^ (hy << 1) ^ (hz << 2);
		}

		void MergeDuplicatedVertices(
			const std::vector<FVector3>& inVertices, const std::vector<uint32_t>& inIndices,
			std::vector<FVector3>* outVertices, std::vector<uint32_t>* outIndices)
		{
			std::unordered_map<FVector3, uint32_t, FVector3Hasher> uniqueVertexMap;
			outVertices->clear();
			outIndices->resize(inIndices.size());

			for (size_t i = 0; i < inIndices.size(); ++i)
			{
				const FVector3& pos = inVertices[inIndices[i]];
				auto it = uniqueVertexMap.find(pos);
				if (it != uniqueVertexMap.end())
				{
					(*outIndices)[i] = it->second;
				}
				else
				{
					uint32_t newIndex = static_cast<uint32_t>(outVertices->size());
					outVertices->emplace_back(pos);
					uniqueVertexMap[pos] = newIndex;
					(*outIndices)[i] = newIndex;
				}
			}
		}

		AABB ComputeBoundingBox(const std::vector<FVector3>& vertices)
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