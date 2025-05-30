#pragma once
#include <cstdint>
#include <algorithm>

struct Edge
{
	uint32_t a, b;
	Edge(uint32_t i, uint32_t j)
	{
		a = std::min(i, j);
		b = std::max(i, j);
	}

	bool operator==(const Edge& other) const
	{
		return a == other.a && b == other.b;
	}
};

namespace std
{
	template<>
	struct hash<Edge>
	{
		size_t operator()(const Edge& e) const
		{
			return hash<uint32_t>()(e.a) ^ hash<uint32_t>()(e.b << 1);
		}
	};
}