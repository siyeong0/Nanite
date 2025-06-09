#include "MeshSimplifier.h"

#include <iostream>
#include <set>
#include <map>
#include <unordered_map>
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
		for (int triIdx = 0; triIdx < mesh.NumTriangles(); ++triIdx)
		{
			auto [e0, e1, e2] = mesh.GetTriangleEdges(triIdx);
			edges.insert(e0);
			edges.insert(e1);
			edges.insert(e2);
			edgeUsage[e0]++;
			edgeUsage[e1]++;
			edgeUsage[e2]++;
		}

		// collect triangles
		std::unordered_map<uint32_t, std::set<uint32_t>> vertToTriMap;
		for (int triIdx = 0; triIdx < mesh.NumTriangles(); ++triIdx)
		{
			auto [i0, i1, i2] = mesh.GetTriangleIndices(triIdx);
			vertToTriMap[i0].insert(triIdx);
			vertToTriMap[i1].insert(triIdx);
			vertToTriMap[i2].insert(triIdx);
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
			FVector3 Position = FVector3::Zero();
			float Length = 0;
			bool bFixB = false;
			int Phase = 0;

			bool operator<(const CollapseCandidate& other) const
			{
				return std::tie(Phase, Error, Length, Edge) < std::tie(other.Phase, other.Error, other.Length, other.Edge);
			}
		};
		// this set works like priority queue
		std::set<CollapseCandidate> collapseSet;
		
		// lamda to build collapse candidate
		auto buildEdgeCollapse = [&quadrics](const Edge& e, const Mesh& mesh, int phase, bool bFixA, bool bFixB)
			{
				const FVector3& vertexA = mesh.Vertices[e.GetA()];
				const FVector3& vertexB = mesh.Vertices[e.GetB()];
				CollapseCandidate candidate;
				candidate.Edge = e;
				candidate.Quadric.Q = quadrics[e.GetA()].Q + quadrics[e.GetB()].Q;
				// find optimal position
				FMatrix3x3 edgeQuadric3x3 = {
					candidate.Quadric.Q[0][0], candidate.Quadric.Q[0][1], candidate.Quadric.Q[0][2],
					candidate.Quadric.Q[1][0], candidate.Quadric.Q[1][1], candidate.Quadric.Q[1][2],
					candidate.Quadric.Q[2][0], candidate.Quadric.Q[2][1], candidate.Quadric.Q[2][2] };
				FVector3 edgeVector3 = { -candidate.Quadric.Q[0][3], -candidate.Quadric.Q[1][3], -candidate.Quadric.Q[2][3] };
				if (bFixA)
				{
					candidate.Position = vertexA;
				}
				else if (bFixB)
				{
					candidate.Position = vertexB;
				}
				else if (fabs(edgeQuadric3x3.Determinant()) > 1e-6f) // valid
				{
					candidate.Position = edgeQuadric3x3.Inverse() * edgeVector3;
				}
				else
				{
					candidate.Position = (mesh.Vertices[e.GetA()] + mesh.Vertices[e.GetB()]) * 0.5f; // center of edge
				}
				// error
				candidate.Error = candidate.Quadric.Evaluate(candidate.Position);
				candidate.Length = (vertexA - vertexB).Length();;
				candidate.bFixB = bFixB;
				candidate.Phase = phase;

				return candidate;
			};

		std::unordered_map<Edge, std::set<CollapseCandidate>::iterator> edgeToCollpaseMap;

		// build priority queue
		Mesh srcMesh(mesh);
		for (const Edge& edge : edges)
		{
			bool bFixA = boundaryVertIndices.find(edge.GetA()) != boundaryVertIndices.end();
			bool bFixB = boundaryVertIndices.find(edge.GetB()) != boundaryVertIndices.end();
			if (bFixA || bFixB) continue;

			// push to priority queue
			auto [iter, b] = collapseSet.insert(buildEdgeCollapse(edge, srcMesh, 0, bFixA, bFixB));
			edgeToCollpaseMap[edge] = iter;
		}

		// simplify loop
		int numValidVertices = srcMesh.NumVertices();
		int numValidTriangles = srcMesh.NumTriangles();
		while (targetTriangleCount < numValidTriangles)
		{
		CONTINUE:
			if (collapseSet.size() == 0)
			{
				// there's no edge to collapse
				break;
			}

			// pick best one from priority queue
			const CollapseCandidate& bestCandidate = *collapseSet.begin();
			const FVector3 optimalPosition = bestCandidate.Position;
			uint32_t keepIdx = bestCandidate.Edge.GetA();
			uint32_t removeIdx = bestCandidate.Edge.GetB();
			if (bestCandidate.bFixB)
			{
				keepIdx = bestCandidate.Edge.GetB();
				removeIdx = bestCandidate.Edge.GetA();
			}

			const std::set<uint32_t> trisWithKeep = vertToTriMap[keepIdx];
			const std::set<uint32_t> trisWithRemove = vertToTriMap[removeIdx];

			// triangles that have both keepIdx and removeIdx (the intersection) need to be removed.
			std::set<uint32_t> removedTriangles;
			std::set_intersection(
				trisWithKeep.begin(), trisWithKeep.end(),
				trisWithRemove.begin(), trisWithRemove.end(),
				std::inserter(removedTriangles, removedTriangles.begin()));

			if (removedTriangles.size() < 2)
			{
				edgeToCollpaseMap.erase(collapseSet.begin()->Edge);
				collapseSet.erase(collapseSet.begin());
				goto CONTINUE;
			}

			// triangles that have in either keepIdx or removeIdx (the union) need to be updated,
			// except for those that are being deleted.
			std::set<uint32_t> updatedTrianglesTmp;
			std::set_union(
				trisWithKeep.begin(), trisWithKeep.end(),
				trisWithRemove.begin(), trisWithRemove.end(),
				std::inserter(updatedTrianglesTmp, updatedTrianglesTmp.begin()));
			std::set<uint32_t> updatedTriangles;
			std::set_difference(
				updatedTrianglesTmp.begin(), updatedTrianglesTmp.end(),
				removedTriangles.begin(), removedTriangles.end(),
				std::inserter(updatedTriangles, updatedTriangles.begin()));

			// check for flipped triangles using normal vectors.
			for (const uint32_t updatedTriIdx : updatedTriangles)
			{
				const FVector3& oldNormal = srcMesh.Normals[updatedTriIdx];
				// compute the updated normal
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(updatedTriIdx);
				const FVector3& v0 = (i0 == removeIdx) || (i0 == keepIdx) ? optimalPosition : srcMesh.Vertices[i0];
				const FVector3& v1 = (i1 == removeIdx) || (i1 == keepIdx) ? optimalPosition : srcMesh.Vertices[i1];
				const FVector3& v2 = (i2 == removeIdx) || (i2 == keepIdx) ? optimalPosition : srcMesh.Vertices[i2];
				const FVector3 newNormal = utils::ComputeNormal(v0, v1, v2);
				if (oldNormal.Dot(newNormal) < 1e-4f)
				{
					// if flipped triangle exists,
					// exclude current best candidate
					edgeToCollpaseMap.erase(collapseSet.begin()->Edge);
					collapseSet.erase(collapseSet.begin());
					goto CONTINUE;
				}
			}

			// update counts
			numValidVertices -= 1;
			numValidTriangles -= static_cast<int>(removedTriangles.size());
			assert(removedTriangles.size() == 2);

			// update collapse queue
			// erase edges with removeIdx
			for (const uint32_t triWithRemoveIdx : trisWithRemove)
			{
				auto [e0, e1, e2] = srcMesh.GetTriangleEdges(triWithRemoveIdx);
				for (const Edge& e : { e0, e1, e2 })
				{
					if (e.GetA() == removeIdx || e.GetB() == removeIdx)
					{
						if (edgeToCollpaseMap.find(e) != edgeToCollpaseMap.end())
						{
							auto it = edgeToCollpaseMap[e];
							edgeToCollpaseMap.erase(e);
							collapseSet.erase(it);
						}
					}
				}
			}

			// update vertex to triangles map
			vertToTriMap[keepIdx].insert(vertToTriMap[removeIdx].begin(), vertToTriMap[removeIdx].end());
			for (const uint32_t removedTriIdx : removedTriangles)
			{
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(removedTriIdx);
				vertToTriMap[i0].erase(removedTriIdx);
				vertToTriMap[i1].erase(removedTriIdx);
				vertToTriMap[i2].erase(removedTriIdx);
			}
			vertToTriMap.erase(removeIdx);

			// update quadrics; remove old planes
			// before updating vertices and triangles
			for (const uint32_t updatedTriIdx : updatedTrianglesTmp)
			{
				// update quadrics
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(updatedTriIdx);
				auto [v0, v1, v2] = srcMesh.GetTriangleVertices(updatedTriIdx);
				const FVector3& n = srcMesh.Normals[updatedTriIdx];
				float d = -n.Dot(v0);
				quadrics[i0].RemovePlane(n, d);
				quadrics[i1].RemovePlane(n, d);
				quadrics[i2].RemovePlane(n, d);
			}

			// update the vertices
			srcMesh.Vertices[keepIdx] = optimalPosition;
			srcMesh.Vertices[removeIdx] = FVector3{ std::numeric_limits<float>::quiet_NaN(), 0, 0 };
			// update indices
			// replace removeIdx to keepIdx
			for (const uint32_t triWithRemoveIdx : trisWithRemove)
			{
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(triWithRemoveIdx);
				i0 = (i0 == removeIdx) ? keepIdx : i0;
				i1 = (i1 == removeIdx) ? keepIdx : i1;
				i2 = (i2 == removeIdx) ? keepIdx : i2;
			}
			// mark removed triangles
			for (const uint32_t removedTriIdx : removedTriangles)
			{
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(removedTriIdx);
				i0 = 0; i1 = 0; i2 = 0;
			}
			// update normals
			for (const uint32_t updatedTriIdx : updatedTriangles)
			{
				auto [v0, v1, v2] = srcMesh.GetTriangleVertices(updatedTriIdx);
				srcMesh.Normals[updatedTriIdx] = utils::ComputeNormal(v0, v1, v2);
			}

			// update quadrics; add new planes
			// after updating vertices and triangles
			for (const uint32_t updatedTriIdx : updatedTriangles)
			{
				// update quadrics
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(updatedTriIdx);
				auto [v0, v1, v2] = srcMesh.GetTriangleVertices(updatedTriIdx);
				const FVector3& n = srcMesh.Normals[updatedTriIdx];
				float d = -n.Dot(v0);
				quadrics[i0].AddPlane(n, d);
				quadrics[i1].AddPlane(n, d);
				quadrics[i2].AddPlane(n, d);
			}

			// collect affected edges
			std::unordered_set<Edge> affectedEdges;
			for (const uint32_t updatedTriIdx : updatedTriangles)
			{
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(updatedTriIdx);
				for (const uint32_t i : { i0, i1, i2 })
				{
					for (const uint32_t t : vertToTriMap[i])
					{
						auto [e0, e1, e2] = srcMesh.GetTriangleEdges(t);
						for (const Edge& e : { e0, e1, e2 })
						{
							if (e.GetA() == i || e.GetB() == i)
							{
								affectedEdges.insert(e);
							}
						}
					}
				}
			}

			// update collapse candidates queue
			for (const Edge& affEdge : affectedEdges)
			{
				bool bFixA = boundaryVertIndices.find(affEdge.GetA()) != boundaryVertIndices.end();
				bool bFixB = boundaryVertIndices.find(affEdge.GetB()) != boundaryVertIndices.end();
				if (bFixA && bFixB) continue;

				int phase = 0;
				if (edgeToCollpaseMap.find(affEdge) != edgeToCollpaseMap.end())
				{
					auto it = edgeToCollpaseMap[affEdge];
					phase = it->Phase;
					edgeToCollpaseMap.erase(affEdge);
					collapseSet.erase(it);
				}

				auto [iter, b] = collapseSet.insert(buildEdgeCollapse(affEdge, srcMesh, phase, bFixA, bFixB));
				edgeToCollpaseMap[affEdge] = iter;
			}
		}

		resultMesh.Vertices.reserve(numValidVertices);
		resultMesh.Indices.reserve(numValidTriangles * 3);
		resultMesh.Normals.reserve(numValidTriangles);
		resultMesh.Colors.reserve(numValidTriangles);

		std::unordered_map<uint32_t, uint32_t> vertIndexMap;
		for (int i = 0; i < srcMesh.Vertices.size(); ++i)
		{
			const FVector3& v = srcMesh.Vertices[i];
			if (std::isnan(v.x)) continue;
			resultMesh.Vertices.emplace_back(v);
			vertIndexMap[i] = static_cast<uint32_t>(resultMesh.Vertices.size() - 1);
		}

		for (int triIdx = 0; triIdx < srcMesh.NumTriangles(); ++triIdx)
		{
			auto [i0, i1, i2] = srcMesh.GetTriangleIndices(triIdx);
			if (i0 == 0 && i1 == 0 && i2 == 0) continue;
			resultMesh.Indices.emplace_back(vertIndexMap[i0]);
			resultMesh.Indices.emplace_back(vertIndexMap[i1]);
			resultMesh.Indices.emplace_back(vertIndexMap[i2]);
			resultMesh.Normals.emplace_back(srcMesh.Normals[triIdx]);
			resultMesh.Colors.emplace_back(srcMesh.Colors[triIdx]);
		}

		return resultMesh;
	}
}