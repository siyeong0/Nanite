#include "MeshSimplifier.h"

#include <iostream>
#include <set>
#include <map>
#include <queue>

#include "Quadric.h"
#include "../Utils/Utils.h"

namespace nanite
{
	Mesh MeshSimplifier::SimplifyMesh(const Mesh& mesh, int targetTriangleCount) const
	{
		Mesh resultMesh;

		// compute edge quadrics
		std::vector<Quadric> quadrics;
		quadrics.resize(mesh.Vertices.size());
		for (int triIdx = 0; triIdx < mesh.NumTriangles(); ++triIdx)
		{
			auto [i0, i1, i2] = mesh.GetTriangleIndices(triIdx);
			auto [v0, v1, v2] = mesh.GetTriangleVertices(triIdx);
			const FVector3& normal = mesh.Normals[triIdx];
			float d = -normal.Dot(v0);
			quadrics[i0].AddPlane(normal, d);
			quadrics[i1].AddPlane(normal, d);
			quadrics[i2].AddPlane(normal, d);
		}

		// collect edges
		std::set<Edge> edges;
		std::map<Edge, int> edgeUsage;
		std::map<uint32_t, std::set<uint32_t>> edgeMap;
		for (int triIdx = 0; triIdx < mesh.NumTriangles(); ++triIdx)
		{
			auto [e0, e1, e2] = mesh.GetTriangleEdges(triIdx);
			edges.insert(e0);
			edges.insert(e1);
			edges.insert(e2);
			edgeUsage[e0]++;
			edgeUsage[e1]++;
			edgeUsage[e2]++;

			auto [i0, i1, i2] = mesh.GetTriangleIndices(triIdx);
			edgeMap[i0].insert(i1);
			edgeMap[i0].insert(i2);
			edgeMap[i1].insert(i0);
			edgeMap[i1].insert(i2);
			edgeMap[i2].insert(i0);
			edgeMap[i2].insert(i1);
		}

		// collect boundary vertices
		std::set<uint32_t> boundaryVertIndices;
		for (const auto& [edge, count] : edgeUsage)
		{
			assert(count <= 2);
			if (count == 1)
			{
				boundaryVertIndices.insert(edge.GetA());
				boundaryVertIndices.insert(edge.GetB());
			}
		}

		// collase candidate buffer
		struct CollapseCandidate
		{
			Edge Edge;
			Quadric Quadric;
			float Error = std::numeric_limits<float>::max();
			FVector3 Position;

			bool operator<(const CollapseCandidate& other) const
			{
				return std::tie(Error, Edge) < std::tie(other.Error, other.Edge);
			}
		};
		// this set works like priority queue
		std::set<CollapseCandidate> collapseSet;

		// lamda to build collapse candidate
		auto buildEdgeCollapse = [&quadrics](const Edge& e, const Mesh& mesh)
			{
				CollapseCandidate candidate;
				candidate.Edge = e;
				candidate.Quadric.Q = quadrics[e.GetA()].Q + quadrics[e.GetB()].Q;
				// find optimal position
				FMatrix3x3 edgeQuadric3x3 = {
					candidate.Quadric.Q[0][0], candidate.Quadric.Q[0][1], candidate.Quadric.Q[0][2],
					candidate.Quadric.Q[1][0], candidate.Quadric.Q[1][1], candidate.Quadric.Q[1][2],
					candidate.Quadric.Q[2][0], candidate.Quadric.Q[2][1], candidate.Quadric.Q[2][2] };
				FVector3 edgeVector3 = { -candidate.Quadric.Q[0][3], -candidate.Quadric.Q[1][3], -candidate.Quadric.Q[2][3] };
				if (fabs(edgeQuadric3x3.Determinant()) > 1e-6f) // valid
				{
					candidate.Position = edgeQuadric3x3.Inverse() * edgeVector3;
				}
				else
				{
					candidate.Position = (mesh.Vertices[e.GetA()] + mesh.Vertices[e.GetB()]) * 0.5f; // center of edge
				}
				// error
				candidate.Error = candidate.Quadric.Evaluate(candidate.Position);

				return candidate;
			};

		// build priority queue
		Mesh srcMesh(mesh);
		for (const Edge& edge : edges)
		{
			bool aIsBoundary = boundaryVertIndices.find(edge.GetA()) != boundaryVertIndices.end();
			bool bIsBoundary = boundaryVertIndices.find(edge.GetB()) != boundaryVertIndices.end();
			if (aIsBoundary || bIsBoundary) continue;

			// push to priority queue
			collapseSet.insert(buildEdgeCollapse(edge, srcMesh));
		}

		// simplify loop
		while(targetTriangleCount < srcMesh.NumTriangles())
		{
			Mesh dstMesh;
			dstMesh.Vertices = srcMesh.Vertices;

			// there's no edge to collapse
			if (collapseSet.size() == 0)
				return resultMesh;

			const CollapseCandidate& bestCandidate = *collapseSet.begin();

			uint32_t keepIdx = bestCandidate.Edge.GetA();
			uint32_t removeIdx = bestCandidate.Edge.GetB();
			dstMesh.Vertices[keepIdx] = bestCandidate.Position;

			std::vector<std::pair<int, int>> updatedTrianglePairs;
			for (int triIdx = 0; triIdx < srcMesh.NumTriangles(); ++triIdx)
			{
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(triIdx);
				bool bI0 = (i0 == removeIdx) || (i0 == keepIdx);
				bool bI1 = (i1 == removeIdx) || (i1 == keepIdx);
				bool bI2 = (i2 == removeIdx) || (i2 == keepIdx);

				uint32_t newI0 = bI0 ? keepIdx : i0;
				uint32_t newI1 = bI1 ? keepIdx : i1;
				uint32_t newI2 = bI2 ? keepIdx : i2;
				bool bValid = (newI0 != newI1 && newI1 != newI2 && newI2 != newI0);

				if (bValid)
				{
					dstMesh.Indices.emplace_back(newI0);
					dstMesh.Indices.emplace_back(newI1);
					dstMesh.Indices.emplace_back(newI2);
				}

				if (bI0 || bI1 || bI2)
				{
					updatedTrianglePairs.emplace_back(
						std::make_pair(triIdx, bValid ? dstMesh.NumTriangles() - 1 : -1));
				}
			}

			dstMesh.ComputeNormals();

			for (auto [oldTriIdx, newTriIdx] : updatedTrianglePairs)
			{
				if (newTriIdx == -1) continue;
				// check if any triangles are flipped
				const FVector3& oldNormal = srcMesh.Normals[oldTriIdx];
				const FVector3 newNormal = dstMesh.Normals[newTriIdx];
				if (oldNormal.Dot(newNormal) < 0.0f)
				{
					return resultMesh;
				}
			}

			for (uint32_t other : edgeMap[removeIdx])
			{
				auto it = std::find_if(collapseSet.begin(), collapseSet.end(),
					[&](const CollapseCandidate& c) {return c.Edge == Edge(removeIdx, other); });
				collapseSet.erase(it);
			}

			edgeMap[keepIdx].insert(edgeMap[removeIdx].begin(), edgeMap[removeIdx].end());
			for (uint32_t rmNeighbor : edgeMap[removeIdx])
			{
				edgeMap[rmNeighbor].erase(removeIdx);
				edgeMap[rmNeighbor].insert(keepIdx);
			}
			edgeMap[keepIdx].erase(keepIdx);
			edgeMap.erase(removeIdx);

			std::unordered_set<Edge> affectedEdges;
			for (auto [oldTriIdx, newTriIdx] : updatedTrianglePairs)
			{
				// update quadrics
				auto [oldI0, oldI1, oldI2] = srcMesh.GetTriangleIndices(oldTriIdx);
				auto [oldV0, oldV1, oldV2] = srcMesh.GetTriangleVertices(oldTriIdx);
				const FVector3& oldNormal = srcMesh.Normals[oldTriIdx];
				float oldD = -oldNormal.Dot(oldV0);
				quadrics[oldI0].RemovePlane(oldNormal, oldD);
				quadrics[oldI1].RemovePlane(oldNormal, oldD);
				quadrics[oldI2].RemovePlane(oldNormal, oldD);

				for (uint32_t other : edgeMap[oldI0]) affectedEdges.insert(Edge(oldI0, other));
				for (uint32_t other : edgeMap[oldI1]) affectedEdges.insert(Edge(oldI1, other));
				for (uint32_t other : edgeMap[oldI2]) affectedEdges.insert(Edge(oldI2, other));

				if (newTriIdx == -1) continue;
				auto [newI0, newI1, newI2] = dstMesh.GetTriangleIndices(newTriIdx);
				auto [newV0, newV1, newV2] = dstMesh.GetTriangleVertices(newTriIdx);
				const FVector3& newNormal = dstMesh.Normals[newTriIdx];
				float newD = -newNormal.Dot(newV0);
				quadrics[newI0].AddPlane(newNormal, newD);
				quadrics[newI1].AddPlane(newNormal, newD);
				quadrics[newI2].AddPlane(newNormal, newD);

				for (uint32_t other : edgeMap[newI0]) affectedEdges.insert(Edge(newI0, other));
				for (uint32_t other : edgeMap[newI1]) affectedEdges.insert(Edge(newI1, other));
				for (uint32_t other : edgeMap[newI2]) affectedEdges.insert(Edge(newI2, other));
			}

			for (const Edge& affEdge : affectedEdges)
			{
				bool aIsBoundary = boundaryVertIndices.find(affEdge.GetA()) != boundaryVertIndices.end();
				bool bIsBoundary = boundaryVertIndices.find(affEdge.GetB()) != boundaryVertIndices.end();
				if (aIsBoundary || bIsBoundary) continue;

				auto it = std::find_if(collapseSet.begin(), collapseSet.end(),
					[&](const CollapseCandidate& c) {return c.Edge == affEdge; });
				if (it != collapseSet.end()) collapseSet.erase(it);
				collapseSet.insert(buildEdgeCollapse(affEdge, dstMesh));
			}

			srcMesh = std::move(dstMesh);
		}
		resultMesh = std::move(srcMesh);
		return resultMesh;
	}
}