#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "Geometry.h"
#include "Path.h"
#include "PRIME_ARRAY.h"
#include "UUID.h"
#include "UUIDGenerator.h"

#include "../Topology/Cluster.h"

namespace nanite
{
	namespace utils
	{
		inline FVector3 HSVtoRGB(float h, float s, float v)
		{
			float c = v * s;
			float x = c * (1.f - abs(fmod(h * 6.f, 2.f) - 1.f));
			float m = v - c;

			FVector3 rgb;

			if (h < 1.0 / 6.0)
				rgb = FVector3(c, x, 0);
			else if (h < 2.0 / 6.0)
				rgb = FVector3(x, c, 0);
			else if (h < 3.0 / 6.0)
				rgb = FVector3(0, c, x);
			else if (h < 4.0 / 6.0)
				rgb = FVector3(0, x, c);
			else if (h < 5.0 / 6.0)
				rgb = FVector3(x, 0, c);
			else
				rgb = FVector3(c, 0, x);

			return rgb + FVector3{ m, m, m };
		}

		inline bool IsPrime(size_t num)
		{
			if (num < 2) return false;
			size_t limit = static_cast<size_t>(std::sqrt(num));
			for (size_t p : PRIME_ARRAY) 
			{
				if (p > limit) break;
				if (num % p == 0) return false;
			}
			return true;
		}

		inline size_t NextPrime(size_t n)
		{
			auto begin = std::begin(PRIME_ARRAY);
			auto end = std::end(PRIME_ARRAY);
			auto it = std::upper_bound(begin, end, n);
			if (it != end) 
			{
				return *it;
			}
			else 
			{
				return 0;
			}
		}

		inline void PaintMeshByCluster(
			Mesh* mesh, 
			const std::vector<Cluster>& clusters, 
			const std::vector<FVector3>& colorCandidates = {})
		{
			std::vector<FVector3> colors = colorCandidates;
			const int DEFAULT_COLOR_CANDIDATES = 6;
			if (colors.empty())
			{
				for (int i = 0; i < DEFAULT_COLOR_CANDIDATES; ++i)
				{
					colors.emplace_back(utils::HSVtoRGB(std::fmod((float)i / DEFAULT_COLOR_CANDIDATES, 1.f), 1.f, 1.f));
				}
			}

			// Create a map to store verts and the clusters they belong to
			std::unordered_map<uint32_t, std::vector<int>> vertToClustersMap;
			vertToClustersMap.reserve(utils::NextPrime(2 * (mesh->NumTriangles() * 3) + 1));
			for (int clusterIdx = 0; clusterIdx < clusters.size(); ++clusterIdx)
			{
				const Cluster& cluster = clusters[clusterIdx];
				for (int triIdx : cluster.Triangles)
				{
					auto [i0, i1, i2] = mesh->GetTriangleIndices(triIdx);
					vertToClustersMap[i0].emplace_back(clusterIdx);
					vertToClustersMap[i1].emplace_back(clusterIdx);
					vertToClustersMap[i2].emplace_back(clusterIdx);
				}
			}

			// Find neighboring clusters based on shared edges
			std::unordered_map<int, std::unordered_set<int>> neighborClusters;
			neighborClusters.reserve(utils::NextPrime(clusters.size() * 2 + 1));
			for (const auto& [vert, clusterIdxs] : vertToClustersMap)
			{
				for (size_t i = 0; i < clusterIdxs.size(); ++i)
				{
					for (size_t j = i + 1; j < clusterIdxs.size(); ++j)
					{
						int c0 = clusterIdxs[i];
						int c1 = clusterIdxs[j];
						neighborClusters[c0].emplace(c1);
						neighborClusters[c1].emplace(c0);
					}
				}
			}
			
			// Assign colors to clusters
			// Ensure that neighboring clusters have different colors
			// Use the least used color first
			std::unordered_map<int, FVector3> clusterColors;
			clusterColors.reserve(utils::NextPrime(clusters.size() * 2 + 1));
			std::vector<int> colorUsageCount(colors.size());
			for (int clusterIdx = 0; clusterIdx < clusters.size(); ++clusterIdx)
			{
				const Cluster& cluster = clusters[clusterIdx];
				// Find all colors of neighboring clusters
				std::vector<FVector3> neighborColors(neighborClusters[clusterIdx].size());
				for (int neighborIdx : neighborClusters[clusterIdx])
				{
					if (clusterColors.find(neighborIdx) != clusterColors.end())
					{
						neighborColors.emplace_back(clusterColors[neighborIdx]);
					}
				}
				// Assign a unique color to the current cluster
				nanite::FVector3 availColor = { 0,0,0 };
				std::vector<int> colorIndices(colorUsageCount.size());
				for (int i = 0; i < colorIndices.size(); ++i) colorIndices[i] = i;
				std::sort(colorIndices.begin(), colorIndices.end(), [&](size_t a, size_t b) { return colorUsageCount[a] < colorUsageCount[b]; });

				for (int cidx : colorIndices)
				{
					if (std::find(neighborColors.begin(), neighborColors.end(), colors[cidx]) == neighborColors.end())
					{
						availColor = colors[cidx];
						colorUsageCount[cidx]++;
						break;
					}
				}
				clusterColors[clusterIdx] = availColor;
			}

			// Apply colors to the triangles
			for (int clusterIdx = 0; clusterIdx < clusters.size(); ++clusterIdx)
			{
				const Cluster& cluster = clusters[clusterIdx];
				FVector3 color = clusterColors[clusterIdx];
				for (int triIdx : cluster.Triangles)
				{
					mesh->Colors[triIdx] = color;
				}
			}
		}
	}
}