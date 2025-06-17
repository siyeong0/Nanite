#include "MeshSimplifier.h"

#include <iostream>
#include <array>
#include <map>

#include "../Utils/Utils.h"
#include "CollapseQueue.h"

namespace nanite
{

	Mesh SimplifyMesh(const Mesh& mesh, int targetTriangleCount, int* outNumValidTriangles, bool bOrganize)
	{
		// Prepare buffers
		Mesh srcMesh(mesh);

		std::vector<Quadric> quadrics;
		std::set<Edge> edges;
		std::unordered_map<Edge, int> edgeUsage;
		std::unordered_map<uint32_t, std::set<uint32_t>> vertToTriMap;

		quadrics.resize(srcMesh.Vertices.size());
		vertToTriMap.reserve(utils::NextPrime(2 * srcMesh.Vertices.size() + 1));
		for (int triIdx = 0; triIdx < srcMesh.NumTriangles(); ++triIdx)
		{
			auto [i0, i1, i2] = srcMesh.GetTriangleIndices(triIdx);
			auto [v0, v1, v2] = srcMesh.GetTriangleVertices(triIdx);
			auto [e0, e1, e2] = srcMesh.GetTriangleEdges(triIdx);
			// Compute quadrics
			const FVector3& normal = srcMesh.Normals[triIdx];
			float d = -normal.Dot(v0);
			quadrics[i0].AddPlane(normal, d);
			quadrics[i1].AddPlane(normal, d);
			quadrics[i2].AddPlane(normal, d);
			// Collect edges
			edges.emplace(e0);
			edges.emplace(e1);
			edges.emplace(e2);
			edgeUsage[e0]++;
			edgeUsage[e1]++;
			edgeUsage[e2]++;
			// Collect triangles
			vertToTriMap[i0].emplace(triIdx);
			vertToTriMap[i1].emplace(triIdx);
			vertToTriMap[i2].emplace(triIdx);
		}

		// Collect boundary vertices
		std::set<uint32_t> boundaryVertIndices;
		for (const auto& [edge, count] : edgeUsage)
		{
			assert(count <= 2);
			if (count == 1)
			{
				boundaryVertIndices.emplace(edge.GetA());
				boundaryVertIndices.emplace(edge.GetB());
			}
		}

		// Priority queue of collapses
		// Smaller error has higher priority
		CollapseQueue collapseQueue(srcMesh, quadrics, boundaryVertIndices);
		collapseQueue.Reserve(edges.size());
		for (const Edge& edge : edges)
		{
			collapseQueue.Insert(edge);
		}

		// Simplificataion loop
		int numValidVertices = srcMesh.NumVertices();
		int numValidTriangles = srcMesh.NumTriangles();
		while (targetTriangleCount < numValidTriangles)
		{
		CONTINUE:
			if (collapseQueue.Size() == 0)
			{
				// There's no edge to collapse
				break;
			}

			// Pick best one from priority queue
			const Collapse& bestCandidate = collapseQueue.PickBest();
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
			// Triangles that have both keepIdx and removeIdx (the intersection) need to be removed.
			std::set<uint32_t> removedTriangles;
			std::set_intersection(
				trisWithKeep.begin(), trisWithKeep.end(),
				trisWithRemove.begin(), trisWithRemove.end(),
				std::inserter(removedTriangles, removedTriangles.begin()));

			// The edge must be belongs to two triangles
			if (removedTriangles.size() != 2)
			{
				collapseQueue.Erase(bestCandidate);
				goto CONTINUE;
			}

			// Triangles that have in either keepIdx or removeIdx (the union) need to be updated,
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

			// Check for flipped and zero area triangles
			for (const uint32_t updatedTriIdx : updatedTriangles)
			{
				const FVector3& oldNormal = srcMesh.Normals[updatedTriIdx];
				// Compute the updated normal
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(updatedTriIdx);
				const FVector3& v0 = (i0 == removeIdx) || (i0 == keepIdx) ? optimalPosition : srcMesh.Vertices[i0];
				const FVector3& v1 = (i1 == removeIdx) || (i1 == keepIdx) ? optimalPosition : srcMesh.Vertices[i1];
				const FVector3& v2 = (i2 == removeIdx) || (i2 == keepIdx) ? optimalPosition : srcMesh.Vertices[i2];
				const FVector3 newNormal = utils::ComputeNormal(v0, v1, v2);
				// compute area
				float area = utils::ComputeArea(v0, v1, v2);
				if (oldNormal.Dot(newNormal) < 0.1f || area < 1e-6f)
				{
					// If flipped triangle exists,
					// Exclude current best candidate
					collapseQueue.Erase(bestCandidate);
					goto CONTINUE;
				}
			}

			// Check that does merging cause non-manifolds
			edgeUsage.clear();
			for (const uint32_t updatedTriIdx : updatedTriangles)
			{
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(updatedTriIdx);
				uint32_t ui0 = (i0 == removeIdx) ? keepIdx : i0;
				uint32_t ui1 = (i1 == removeIdx) ? keepIdx : i1;
				uint32_t ui2 = (i2 == removeIdx) ? keepIdx : i2;
				edgeUsage[Edge(ui0, ui1)]++;
				edgeUsage[Edge(ui1, ui2)]++;
				edgeUsage[Edge(ui2, ui0)]++;
			}
			for (const auto& [edge, count] : edgeUsage)
			{
				if (count > 2)
				{
					// Edge belongs to over two triangles. It means mesh is non-manifold
					// Exclude current best candidate
					collapseQueue.Erase(bestCandidate);
					goto CONTINUE;
				}
			}

			// Update count values
			numValidVertices -= 1;
			numValidTriangles -= static_cast<int>(removedTriangles.size());
			assert(removedTriangles.size() == 2);

			// Update collapse queue and erase edges with removeIdx
			for (const uint32_t triWithRemoveIdx : trisWithRemove)
			{
				auto [e0, e1, e2] = srcMesh.GetTriangleEdges(triWithRemoveIdx);
				for (const Edge& e : { e0, e1, e2 })
				{
					if (e.GetA() == removeIdx || e.GetB() == removeIdx)
					{
						collapseQueue.Erase(e);
					}
				}
			}

			// Update vertex to triangles map
			vertToTriMap[keepIdx].insert(vertToTriMap[removeIdx].begin(), vertToTriMap[removeIdx].end());
			for (const uint32_t removedTriIdx : removedTriangles)
			{
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(removedTriIdx);
				vertToTriMap[i0].erase(removedTriIdx);
				vertToTriMap[i1].erase(removedTriIdx);
				vertToTriMap[i2].erase(removedTriIdx);
			}
			vertToTriMap.erase(removeIdx);

			// Before updating vertices and triangles, update quadrics; Remove old planes
			for (const uint32_t updatedTriIdx : updatedTrianglesTmp)
			{
				// Update quadrics
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(updatedTriIdx);
				auto [v0, v1, v2] = srcMesh.GetTriangleVertices(updatedTriIdx);
				const FVector3& n = srcMesh.Normals[updatedTriIdx];
				float d = -n.Dot(v0);
				quadrics[i0].RemovePlane(n, d);
				quadrics[i1].RemovePlane(n, d);
				quadrics[i2].RemovePlane(n, d);
			}

			// Update vertices
			srcMesh.Vertices[keepIdx] = optimalPosition;
			srcMesh.Vertices[removeIdx] = INVALID_VERTEX;
			// Update indices; replace removeIdx to keepIdx
			for (const uint32_t triWithRemoveIdx : trisWithRemove)
			{
				auto [i0, i1, i2] = srcMesh.GetTriangleIndices(triWithRemoveIdx);
				i0 = (i0 == removeIdx) ? keepIdx : i0;
				i1 = (i1 == removeIdx) ? keepIdx : i1;
				i2 = (i2 == removeIdx) ? keepIdx : i2;
			}
			// Mark removed triangles
			for (const uint32_t removedTriIdx : removedTriangles)
			{
				srcMesh.GetTriangleIndices(removedTriIdx) = INVALID_TRIANGLE;
			}
			// Update normals
			for (const uint32_t updatedTriIdx : updatedTriangles)
			{
				auto [v0, v1, v2] = srcMesh.GetTriangleVertices(updatedTriIdx);
				srcMesh.Normals[updatedTriIdx] = utils::ComputeNormal(v0, v1, v2);
			}

			// Update quadrics; add new planes
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

			// Collect affected edges
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
								affectedEdges.emplace(e);
							}
						}
					}
				}
			}

			// Update collapse candidates queue
			for (const Edge& affEdge : affectedEdges)
			{
				int phase = collapseQueue.Erase(affEdge);
				if (phase < 0)
				{
					// Edge is not in the queue
					continue;
				}
				collapseQueue.Insert(affEdge, phase);
			}
		}

		Mesh resultMesh;

		if (!bOrganize)
		{
			resultMesh = srcMesh;
		}
		else
		{
			resultMesh.Vertices.reserve(numValidVertices);
			resultMesh.Indices.reserve(numValidTriangles * 3);
			resultMesh.Normals.reserve(numValidTriangles);
			resultMesh.Colors.reserve(numValidTriangles);

			// Remove invalid vertices
			std::unordered_map<uint32_t, uint32_t> vertIndexMap;
			for (int i = 0; i < srcMesh.Vertices.size(); ++i)
			{
				const FVector3& v = srcMesh.Vertices[i];
				if (v == INVALID_VERTEX) continue;
				resultMesh.Vertices.emplace_back(v);
				vertIndexMap[i] = static_cast<uint32_t>(resultMesh.Vertices.size() - 1);
			}

			// Remove dubplicate and invalid triangles
			struct UniqueTriangle
			{
				std::array<uint32_t, 3> Indices;
				std::array<uint32_t, 3> SortedIndices;
				int TriangleIndex = -1;

				UniqueTriangle(uint32_t i0, uint32_t i1, uint32_t i2, int triIdx)
					: Indices({ i0, i1, i2 })
					, SortedIndices({ i0, i1, i2 })
					, TriangleIndex(triIdx)
				{
					std::sort(SortedIndices.begin(), SortedIndices.end());
				}

				bool operator<(const UniqueTriangle& other) const
				{
					return SortedIndices < other.SortedIndices;
				}
			};

			std::set<UniqueTriangle> uniqueTriangles;
			for (int triIdx = 0; triIdx < srcMesh.NumTriangles(); ++triIdx)
			{
				auto indices = srcMesh.GetTriangleIndices(triIdx);
				if (indices == INVALID_TRIANGLE) continue;
				auto [i0, i1, i2] = indices;
				uniqueTriangles.insert(UniqueTriangle(
					vertIndexMap[i0],
					vertIndexMap[i1],
					vertIndexMap[i2],
					triIdx));
			}

			for (const UniqueTriangle& ut : uniqueTriangles)
			{
				resultMesh.Indices.emplace_back(ut.Indices[0]);
				resultMesh.Indices.emplace_back(ut.Indices[1]);
				resultMesh.Indices.emplace_back(ut.Indices[2]);
				resultMesh.Normals.emplace_back(srcMesh.Normals[ut.TriangleIndex]);
				resultMesh.Colors.emplace_back(srcMesh.Colors[ut.TriangleIndex]);
			}
		}
		
		if (outNumValidTriangles)
		{
			*outNumValidTriangles = numValidTriangles;
		}

		return resultMesh;
	}
}