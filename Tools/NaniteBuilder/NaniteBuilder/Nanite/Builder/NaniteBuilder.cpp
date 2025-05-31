#include "NaniteBuilder.h"

#include <cassert>

namespace nanite
{
	NaniteBuilder::NaniteBuilder(const std::vector<FVector3>& vertices, std::vector<Triangle>& triangles)
		: mVertices(vertices)
	{

	}

	int NaniteBuilder::BuildGraph(int nparts, std::vector<Triangle>& triangles, std::vector<Cluster>& outClusters)
	{
		int numTriangles = static_cast<int>(triangles.size());

		// build triangle graph
		std::unordered_map<Edge, std::vector<idx_t>> edgeToTriangles;
		edgeToTriangles.reserve(numTriangles * 3);
		for (int triIdx = 0; triIdx < numTriangles; ++triIdx)
		{
			const Triangle& triangle = triangles[triIdx];
			uint32_t i0 = triangle.i0;
			uint32_t i1 = triangle.i1;
			uint32_t i2 = triangle.i2;
			edgeToTriangles[Edge(i0, i1)].push_back(static_cast<idx_t>(triIdx));
			edgeToTriangles[Edge(i1, i2)].push_back(static_cast<idx_t>(triIdx));
			edgeToTriangles[Edge(i2, i0)].push_back(static_cast<idx_t>(triIdx));
		}

		// partition the triangle graph using METIS
		std::vector<std::set<idx_t>> triangleAdj(numTriangles);
		for (const auto& [edge, tris] : edgeToTriangles)
		{
			if (tris.size() == 2)
			{
				idx_t t0 = tris[0];
				idx_t t1 = tris[1];
				triangleAdj[t0].insert(t1);
				triangleAdj[t1].insert(t0);
			}
		}

		std::vector<idx_t> xadj;
		std::vector<idx_t> adjncy;

		xadj.push_back(0);
		for (const auto& neighbors : triangleAdj)
		{
			for (auto adjTri : neighbors)
			{
				adjncy.push_back(adjTri);
			}
			xadj.push_back(static_cast<idx_t>(adjncy.size()));
		}

		idx_t ncon = 1;

		idx_t* vwgt = nullptr;
		idx_t* vsize = nullptr;
		idx_t* adjwgt = nullptr;

		std::vector<real_t> tpwgts(nparts, 1.0f / nparts);
		real_t ubvec = { 1.05f };

		idx_t options[METIS_NOPTIONS];
		METIS_SetDefaultOptions(options);
		options[METIS_OPTION_NUMBERING] = 0;

		idx_t objval;
		std::vector<idx_t> partOut;
		partOut.resize(numTriangles);

		int result = METIS_PartGraphKway(
			static_cast<idx_t*>(&numTriangles), &ncon, xadj.data(), adjncy.data(),
			vwgt, vsize, adjwgt, &nparts,
			tpwgts.data(), &ubvec, options, &objval, partOut.data());

		if (!(result == METIS_OK))
		{
			std::cerr << "METIS partitioning failed with error code " << result << "\n";
			return -1;
		}

		// reorder and cluster triangles based on partitions
		std::vector<std::vector<Triangle>> reorederBuffer(nparts);
		for (int i = 0; i < numTriangles; ++i)
		{
			reorederBuffer[partOut[i]].push_back(triangles[i]);
		}

		// reorder and cluster triangles
		triangles.clear();
		outClusters.clear();
		outClusters.resize(nparts);
		for (int i = 0; i < nparts; ++i)
		{
			triangles.insert(triangles.end(), reorederBuffer[i].begin(), reorederBuffer[i].end());
			outClusters[i].StartIndex = static_cast<int>(triangles.size() - reorederBuffer[i].size());
			outClusters[i].NumTriangles = static_cast<int>(reorederBuffer[i].size());
			outClusters[i].Bounds = computeBoundingBox(mVertices, triangles, outClusters[i]);
		}

		return static_cast<int>(objval);
	}

	int NaniteBuilder::MergeClusters(
		const std::vector<Cluster>& clusters,
		const std::vector<Triangle>& triangles,
		std::vector<idx_t>& outMergedClusters)
	{
		int numTriangles = triangles.size();
		int numClusters = static_cast<int>(clusters.size());
		int mergeGroupSize = 4;
		int numGroups = static_cast<int>(std::ceilf(static_cast<float>(numClusters) / mergeGroupSize));

		if (numGroups < 2) return -1;

		std::unordered_map<Edge, std::vector<idx_t>> edgeToClusters;
		edgeToClusters.reserve(numTriangles * 3);
		for (int clusterIdx = 0; clusterIdx < clusters.size(); ++clusterIdx)
		{
			const Cluster& cluster = clusters[clusterIdx];
			for (int triIdx = cluster.StartIndex; triIdx < cluster.StartIndex + cluster.NumTriangles; ++triIdx)
			{
				const Triangle& triangle = triangles[triIdx];
				uint32_t i0 = triangle.i0;
				uint32_t i1 = triangle.i1;
				uint32_t i2 = triangle.i2;
				edgeToClusters[Edge(i0, i1)].push_back(static_cast<idx_t>(clusterIdx));
				edgeToClusters[Edge(i1, i2)].push_back(static_cast<idx_t>(clusterIdx));
				edgeToClusters[Edge(i2, i0)].push_back(static_cast<idx_t>(clusterIdx));
			}
		}

		// partition the triangle graph using METIS
		std::vector<std::set<idx_t>> clusterAdj(numTriangles);
		for (const auto& [edge, clusters] : edgeToClusters)
		{
			if (clusters.size() == 2)
			{
				idx_t t0 = clusters[0];
				idx_t t1 = clusters[1];
				clusterAdj[t0].insert(t1);
				clusterAdj[t1].insert(t0);
			}
		}

		std::vector<idx_t> xadj;
		std::vector<idx_t> adjncy;

		xadj.push_back(0);
		for (const auto& neighbors : clusterAdj)
		{
			for (auto adjTri : neighbors)
			{
				adjncy.push_back(adjTri);
			}
			xadj.push_back(static_cast<idx_t>(adjncy.size()));
		}

		idx_t ncon = 1;

		idx_t* vwgt = nullptr;
		idx_t* vsize = nullptr;
		idx_t* adjwgt = nullptr;

		std::vector<real_t> tpwgts(numGroups, 1.0f / numGroups);
		real_t ubvec = { 1.05f };

		idx_t options[METIS_NOPTIONS];
		METIS_SetDefaultOptions(options);
		options[METIS_OPTION_NUMBERING] = 0;

		idx_t objval;
		std::vector<idx_t> partOut;
		partOut.resize(numClusters);

		int result = METIS_PartGraphKway(
			static_cast<idx_t*>(&numClusters), &ncon, xadj.data(), adjncy.data(),
			vwgt, vsize, adjwgt, &numGroups,
			tpwgts.data(), &ubvec, options, &objval, partOut.data());

		if (!(result == METIS_OK))
		{
			std::cerr << "METIS partitioning failed with error code " << result << "\n";
			return -1;
		}

		outMergedClusters = std::move(partOut);

		return static_cast<int>(objval);
	}

	AABB NaniteBuilder::computeBoundingBox(
		const std::vector<FVector3>& vertices,
		const std::vector<Triangle>& triangles,
		const Cluster& cluster)
	{
		AABB bbox;
		for (int i = cluster.StartIndex; i < cluster.StartIndex + cluster.NumTriangles; ++i)
		{
			const Triangle& triangle = triangles[i];
			bbox.Encapsulate(vertices[triangle.i0]);
			bbox.Encapsulate(vertices[triangle.i1]);
			bbox.Encapsulate(vertices[triangle.i2]);
		}
		return bbox;
	}
}