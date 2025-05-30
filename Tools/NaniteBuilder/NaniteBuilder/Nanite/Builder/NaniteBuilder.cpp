#include "NaniteBuilder.h"

namespace nanite
{
	int NaniteBuilder::BuildGraph(const NaniteMesh& mesh, int nparts, std::vector<int32_t>& partOut)
	{
		// build triangle graph
		std::unordered_map<Edge, std::vector<idx_t>> edgeToTriangles;
		edgeToTriangles.reserve(mesh.NumTriangles() * 3);

		for (const Triangle& triangle : mesh.GetTriangles())
		{
			uint32_t i0 = triangle.i0;
			uint32_t i1 = triangle.i1;
			uint32_t i2 = triangle.i2;
			edgeToTriangles[Edge(i0, i1)].push_back(static_cast<idx_t>(&triangle - &mesh.GetTriangles()[0]));
			edgeToTriangles[Edge(i1, i2)].push_back(static_cast<idx_t>(&triangle - &mesh.GetTriangles()[0]));
			edgeToTriangles[Edge(i2, i0)].push_back(static_cast<idx_t>(&triangle - &mesh.GetTriangles()[0]));
		}

		std::vector<std::set<idx_t>> triangleAdj(mesh.NumTriangles());
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

		// partition the triangle graph using METIS
		idx_t ncon = 1;

		idx_t* vwgt = nullptr;
		idx_t* vsize = nullptr;
		idx_t* adjwgt = nullptr;

		std::vector<real_t> tpwgts(nparts, 1.0f / nparts);
		real_t ubvec = { 1.05f };

		idx_t options[METIS_NOPTIONS];
		METIS_SetDefaultOptions(options);
		options[METIS_OPTION_NUMBERING] = 0;

		idx_t numTriangles = mesh.NumTriangles();
		idx_t objval;
		partOut.resize(numTriangles);

		int result = METIS_PartGraphKway(
			&numTriangles, &ncon, xadj.data(), adjncy.data(),
			vwgt, vsize, adjwgt, &nparts,
			tpwgts.data(), &ubvec, options, &objval, partOut.data());

		if (!(result == METIS_OK))
		{
			std::cerr << "METIS partitioning failed with error code " << result << "\n";
			return -1;
		}
		return static_cast<int>(objval);
	}
}