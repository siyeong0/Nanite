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
	static inline idx_t castToWeight(float w) { return static_cast<idx_t>(w * 10000.f); };

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

	std::vector<Cluster> ClusterAndReorderMesh(Mesh* mesh, int numClusters)
	{
		if (numClusters < 2)
		{
			return {};
		}
		std::vector<int> parts = nanite::PartMesh(*mesh, numClusters);
		return ClusterAndReorderMesh(mesh, parts, numClusters);
	}

	std::vector<Cluster> ClusterAndReorderMesh(Mesh* mesh, const std::vector<int>& parts, int numClusters)
	{
		int numParts = numClusters;
		if (numParts <= 0)
		{
			numParts = *std::max_element(parts.begin(), parts.end()) + 1;
		}

		if (numParts < 2)
		{
			return {};
		}

		// Reorder mesh
		std::vector<std::vector<int>> reorederBuffer(numParts);
		for (int triIdx = 0; triIdx < mesh->NumTriangles(); ++triIdx)
		{
			reorederBuffer[parts[triIdx]].emplace_back(triIdx);
		}

		std::vector<uint32_t> reorderedIndices;
		std::vector<FVector3> reorderedNormals;
		std::vector<FVector3> reorderedColors;
		reorderedIndices.reserve(mesh->Indices.size());
		reorderedNormals.reserve(mesh->Normals.size());
		for (const std::vector<int> tris : reorederBuffer)
		{
			for (int triIdx : tris)
			{
				auto [i0, i1, i2] = mesh->GetTriangleIndices(triIdx);
				reorderedIndices.emplace_back(i0);
				reorderedIndices.emplace_back(i1);
				reorderedIndices.emplace_back(i2);
				reorderedNormals.emplace_back(mesh->Normals[triIdx]);
				reorderedColors.emplace_back(mesh->Colors[triIdx]);
			}
		}
		mesh->Indices = std::move(reorderedIndices);
		mesh->Normals = std::move(reorderedNormals);
		mesh->Colors = std::move(reorderedColors);

		// Cluster triangles
		std::vector<Cluster> clusters;
		clusters.resize(numParts);

		int indexOffset = 0;
		for (int i = 0; i < numParts; ++i)
		{
			Cluster& cluster = clusters[i];
			cluster.StartIndex = indexOffset;
			cluster.NumTriangles = static_cast<int>(reorederBuffer[i].size());
			for (int j = cluster.StartIndex; j < cluster.StartIndex + cluster.NumTriangles; ++j)
			{
				auto [v0, v1, v2] = mesh->GetTriangleVertices(j);
				cluster.Bounds.Encapsulate(v0);
				cluster.Bounds.Encapsulate(v1);
				cluster.Bounds.Encapsulate(v2);
			}
			indexOffset += cluster.NumTriangles;
		}

		return clusters;
	}
}