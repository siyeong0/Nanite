#pragma once
#include <cstdint>
#include <algorithm>

struct Edge
{
	uint32_t a, b;

	Edge() : a(0), b(0) {}

	Edge(uint32_t i, uint32_t j)
		: a(std::min(i, j))
		, b(std::max(i, j))
	{

	}

	inline bool operator==(const Edge& other) const
	{
		return a == other.a && b == other.b;
	}

	inline bool operator<(const Edge& other) const
	{
		return std::tie(a, b) < std::tie(other.a, other.b);
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