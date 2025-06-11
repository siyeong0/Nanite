#include "MeshClustering.h"

#include <iostream>
#include <cassert>
#include <vector>
#include <set>
#include <unordered_map>

#include <metis.h>

#include "../../Utils/Utils.h"

namespace nanite
{
	static inline float castToWeight(float w) { return static_cast<idx_t>(w * 10000.f); };

	std::vector<int> PartMesh(const Mesh& mesh, int numParts)
	{
		const int numTriangles = mesh.NumTriangles();

		std::unordered_map<Edge, std::vector<idx_t>> edgeToTriangelesMap;
		edgeToTriangelesMap.reserve(utils::NextPrime(2 * (numTriangles * 3) + 1));
		for (int triIdx = 0; triIdx < mesh.NumTriangles(); ++triIdx)
		{
			auto [e0, e1, e2] = mesh.GetTriangleEdges(triIdx);
			edgeToTriangelesMap[e0].emplace_back(static_cast<idx_t>(triIdx));
			edgeToTriangelesMap[e1].emplace_back(static_cast<idx_t>(triIdx));
			edgeToTriangelesMap[e2].emplace_back(static_cast<idx_t>(triIdx));
		}

		// build the adjacency list between triangles sharing edges
		struct Link
		{
			idx_t Index;
			idx_t Weight;
			bool operator<(const Link& other) const { return Index < other.Index; }
		};
		std::vector<std::set<Link>> triangsAdjacencyList(numTriangles);
		for (const auto& [edge, triIdxs] : edgeToTriangelesMap)
		{
			for (size_t i = 0; i < triIdxs.size(); ++i)
			{
				for (size_t j = i + 1; j < triIdxs.size(); ++j)
				{
					idx_t t0 = triIdxs[i];
					idx_t t1 = triIdxs[j];
					float length = (mesh.Vertices[edge.GetA()] - mesh.Vertices[edge.GetB()]).Length();
					idx_t weight = castToWeight(length);
					triangsAdjacencyList[t0].insert(Link{ t1, weight });
					triangsAdjacencyList[t1].insert(Link{ t0, weight });
				}
			}
		}


		// prepare METIS_PartGraphKway arguments
		idx_t nvtxs = static_cast<idx_t>(numTriangles); // number of nodes
		idx_t ncon = 1; // number of weights in each node
		std::vector<idx_t> xadj; // start index of each node
		std::vector<idx_t> adjncy; // adjacent list
		std::vector<idx_t> vwgt; // weight of each node; the area of triangle -> makes clusters have approximately equal area
		std::vector<idx_t> adjwgt; // weight of each edges; the length of edge -> makes cluster round-shaped
		std::vector<real_t> tpwgts(numParts, 1.0f / numParts); // target weight of each partition
		real_t ubvec = { 1.2f }; // allowed imbalance ratio
		idx_t options[METIS_NOPTIONS]; // options
		METIS_SetDefaultOptions(options);
		options[METIS_OPTION_NUMBERING] = 0; // index starts with 0

		// fill buffers
		xadj.reserve(numTriangles + 1);
		adjncy.reserve(triangsAdjacencyList.size() * 3);
		vwgt.reserve(numTriangles);
		adjwgt.reserve(triangsAdjacencyList.size() * 3);

		xadj.emplace_back(0); // starts with 0
		for (int triIdx = 0; triIdx < triangsAdjacencyList.size(); ++triIdx)
		{
			const auto& neighbors = triangsAdjacencyList[triIdx];
			for (auto adjLink : neighbors)
			{
				adjncy.emplace_back(adjLink.Index); // index
				adjwgt.emplace_back(adjLink.Weight); // length of edge
			}
			auto [v0, v1, v2] = mesh.GetTriangleVertices(triIdx);
			vwgt.emplace_back(castToWeight(utils::ComputeArea(v0, v1, v2)));
			xadj.emplace_back(static_cast<idx_t>(adjncy.size()));
		}

		// METIS part graph
		idx_t objvalOut; // cost of cutting
		std::vector<idx_t> partOut(numTriangles); // partition data

		int result = METIS_PartGraphKway(
			&nvtxs,
			&ncon,
			xadj.data(),
			adjncy.data(),
			vwgt.data(),
			nullptr, // memory size of each node
			adjwgt.data(),
			&numParts,
			tpwgts.data(),
			&ubvec,
			options,
			&objvalOut, partOut.data());

		std::vector<int> resultParts;
		if (!(result == METIS_OK))
		{
			std::cerr << "METIS partitioning failed with error code " << result << "\n";
			return resultParts;
		}
		resultParts.reserve(partOut.size());

		for (int i = 0; i < partOut.size(); ++i)
		{
			resultParts.emplace_back(static_cast<int>(partOut[i]));
		}

		return resultParts;
	}
}