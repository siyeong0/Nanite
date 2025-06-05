#pragma once
#include <cstdint>
#include <algorithm>

namespace nanite
{
	struct Edge
	{
	public:
		Edge() : mA(0), mB(0) {}

		Edge(uint32_t i, uint32_t j)
			: mA(std::min(i, j))
			, mB(std::max(i, j))
		{

		}

		inline uint32_t GetA() const { return mA; }
		inline uint32_t GetB() const { return mB; }

		inline void Set(uint32_t a, uint32_t b)
		{
			mA = std::min(a, b);
			mB = std::max(a, b);
		}
		inline void SetA(uint32_t v) { Set(v, mB); }
		inline void SetB(uint32_t v) { Set(mA, v); }

		inline bool operator==(const Edge& other) const
		{
			return mA == other.mA && mB == other.mB;
		}

		inline bool operator<(const Edge& other) const
		{
			return std::tie(mA, mB) < std::tie(other.mA, other.mB);
		}

	private:
		uint32_t mA, mB;
	};
}

namespace std
{
	template<>
	struct hash<nanite::Edge>
	{
		size_t operator()(const nanite::Edge& e) const
		{
			return hash<uint32_t>()(e.GetA()) ^ hash<uint32_t>()(e.GetB() << 1);
		}
	};
}