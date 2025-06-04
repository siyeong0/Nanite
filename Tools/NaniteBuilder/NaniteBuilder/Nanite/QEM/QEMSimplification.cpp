#include "QEMSimplification.h"

#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include <queue>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>

#include "../../Math/Math.h"
#include "../../Utils/Utils.h"
#include "../../Topology/Edge.h"
#include "../../Topology/Triangle.h"
#include "Quadric.h"

namespace nanite
{
	namespace qem
	{
		Mesh simplifyMesh(const Mesh& mesh)
		{
			const std::vector<FVector3>& vertices = mesh.Vertices;
			const std::vector<Triangle>& triangles = mesh.Triangles;

			// compute edge quadrics
			std::vector<Quadric> quadrics;
			quadrics.resize(vertices.size());
			for (const Triangle& tri : triangles)
			{
				const FVector3& v0 = vertices[tri.i0];
				const FVector3& v1 = vertices[tri.i1];
				const FVector3& v2 = vertices[tri.i2];
				// compute plane
				FVector3 normal = (v1 - v0).Cross(v2 - v0).Norm();
				float d = -normal.Dot(v0);

				quadrics[tri.i0].AddPlane(normal, d);
				quadrics[tri.i1].AddPlane(normal, d);
				quadrics[tri.i2].AddPlane(normal, d);
			}

			// collect edges
			std::set<Edge> edges;
			std::map<Edge, int> edgeUsage;
			for (const Triangle& tri : triangles)
			{
				auto addEdge = [&](int a, int b)
					{
						Edge e(a, b);
						edges.insert(e);
						edgeUsage[e]++;
					};
				addEdge(tri.i0, tri.i1);
				addEdge(tri.i1, tri.i2);
				addEdge(tri.i2, tri.i0);
			}

			// collect boundary vertices
			std::set<uint32_t> boundaryVertIndices;
			for (const auto& [edge, count] : edgeUsage)
			{
				assert(count <= 2);
				if (count == 1)
				{
					boundaryVertIndices.insert(edge.a);
					boundaryVertIndices.insert(edge.b);
				}
			}

			// simpify mesh
			std::vector<FVector3> simplifiedVertices(vertices);
			std::vector<Triangle> simplifiedTriangles(triangles);

			float bestError = std::numeric_limits<float>::max();
			Edge bestEdge(-1, -1);
			FVector3 bestPos;
			Quadric bestQuadric;

			for (const Edge& edge : edges)
			{
				const FVector3 vertexA = simplifiedVertices[edge.a];
				const FVector3 vertexB = simplifiedVertices[edge.b];

				bool aIsBoundary = boundaryVertIndices.count(edge.a);
				bool bIsBoundary = boundaryVertIndices.count(edge.b);
				if (aIsBoundary || bIsBoundary) continue;

				// compute edge error
				Quadric edgeQuadric;
				edgeQuadric.Q = quadrics[edge.a].Q + quadrics[edge.b].Q;

				// find position to merged on the edge 
				FVector3 mergedPos;
				if (aIsBoundary)
				{
					mergedPos = vertexA;
				}
				else if (bIsBoundary)
				{
					mergedPos = vertexB;
				}
				else
				{
					// find optimal position
					FMatrix3x3 edgeQuadric3x3 = {
						edgeQuadric.Q[0][0], edgeQuadric.Q[0][1], edgeQuadric.Q[0][2],
						edgeQuadric.Q[1][0], edgeQuadric.Q[1][1], edgeQuadric.Q[1][2],
						edgeQuadric.Q[2][0], edgeQuadric.Q[2][1], edgeQuadric.Q[2][2] };

					if (fabs(edgeQuadric3x3.Determinant()) > 1e-6f) // valid
					{
						mergedPos = edgeQuadric3x3.Inverse() * FVector3(-edgeQuadric.Q[0][3], -edgeQuadric.Q[1][3], -edgeQuadric.Q[2][3]);
					}
					else
					{
						mergedPos = (vertexA + vertexB) * 0.5f; // center of edge
					}
				}

				// update best vertex info
				float error = edgeQuadric.Evaluate(mergedPos);
				if (error < bestError)
				{
					bestError = error;
					bestEdge = edge;
					bestPos = mergedPos;
					bestQuadric = edgeQuadric;
				}
			}

			// there's no edge to merge
			if (bestEdge.a == -1 || bestEdge.b == -1)
				return Mesh();

			// update vertex and quadric
			uint32_t keepIdx = bestEdge.a;
			uint32_t removeIdx = bestEdge.b;
			simplifiedVertices[keepIdx] = bestPos;
			quadrics[bestEdge.a] = bestQuadric;

			// update triangles
			std::vector<Triangle> updatedSimpTriangles;
			updatedSimpTriangles.reserve(simplifiedTriangles.size());
			for (Triangle& tri : simplifiedTriangles)
			{
				tri.i0 = tri.i0 == removeIdx ? keepIdx : tri.i0;
				tri.i1 = tri.i1 == removeIdx ? keepIdx : tri.i1;
				tri.i2 = tri.i2 == removeIdx ? keepIdx : tri.i2;
				if (tri.i0 != tri.i1 && tri.i1 != tri.i2 && tri.i2 != tri.i0) updatedSimpTriangles.emplace_back(tri);
			}
			simplifiedTriangles = std::move(updatedSimpTriangles);

			// remap vertices
			std::unordered_set<int> usedVertices;
			usedVertices.reserve(simplifiedVertices.size());
			for (const Triangle& tri : simplifiedTriangles)
			{
				usedVertices.insert(tri.i0);
				usedVertices.insert(tri.i1);
				usedVertices.insert(tri.i2);
			}

			std::unordered_map<int, int> oldToNewIndex;
			int newIndex = 0;
			for (int idx : usedVertices)
			{
				oldToNewIndex[idx] = newIndex++;
			}

			std::vector<FVector3> remapedVertices(usedVertices.size());
			for (const auto& [oldIdx, newIdx] : oldToNewIndex)
			{
				remapedVertices[newIdx] = simplifiedVertices[oldIdx];
			}

			std::vector<Triangle> remapedTriangles;
			remapedTriangles.reserve(simplifiedTriangles.size());
			for (const Triangle& tri : simplifiedTriangles)
			{
				Triangle newTri;
				newTri.i0 = oldToNewIndex[tri.i0];
				newTri.i1 = oldToNewIndex[tri.i1];
				newTri.i2 = oldToNewIndex[tri.i2];
				newTri.Normal = tri.Normal;
				newTri.Color = tri.Color;
				remapedTriangles.emplace_back(newTri);
			}

			Mesh simplifiedMesh;
			simplifiedMesh.Vertices = std::move(remapedVertices);
			simplifiedMesh.Triangles = std::move(remapedTriangles);
			return simplifiedMesh;
		}

		std::pair<std::vector<FVector3>, std::vector<Triangle>> SimplifyMesh(
			int targetTriangleCount,
			const std::vector<Triangle>& triangles,
			const std::vector<FVector3>& vertices)
		{
			//Mesh m;
			//m.Vertices = vertices;
			//m.Triangles = triangles;
			//while (m.Triangles.size() > targetTriangleCount)
			//{
			//	m = simplifyMesh(m);
			//}
			//return std::make_pair(m.Vertices, m.Triangles);

			CheckDuplicateTriangles(triangles);
			// compute edge quadrics
			std::vector<Quadric> quadrics;
			quadrics.resize(vertices.size());
			for (const Triangle& tri : triangles)
			{
				const FVector3& v0 = vertices[tri.i0];
				const FVector3& v1 = vertices[tri.i1];
				const FVector3& v2 = vertices[tri.i2];
				// compute plane
				FVector3 normal = (v1 - v0).Cross(v2 - v0).Norm();
				float d = -normal.Dot(v0);

				quadrics[tri.i0].AddPlane(normal, d);
				quadrics[tri.i1].AddPlane(normal, d);
				quadrics[tri.i2].AddPlane(normal, d);
			}

			// collect edges
			std::set<Edge> edges;
			std::map<Edge, int> edgeUsage;
			for (const Triangle& tri : triangles)
			{
				auto addEdge = [&](int a, int b)
					{
						Edge e(a, b);
						edges.insert(e);
						edgeUsage[e]++;
					};
				addEdge(tri.i0, tri.i1);
				addEdge(tri.i1, tri.i2);
				addEdge(tri.i2, tri.i0);
			}

			// collect boundary vertices
			std::set<uint32_t> boundaryVertIndices;
			for (const auto& [edge, count] : edgeUsage)
			{
				//assert(count <= 2);
				if (count == 1)
				{
					boundaryVertIndices.insert(edge.a);
					boundaryVertIndices.insert(edge.b);
				}
			}

			// simpify mesh
			std::vector<FVector3> simplifiedVertices(vertices);
			std::vector<Triangle> simplifiedTriangles(triangles);

			// helper functions
			auto isValidTriangle = [](const Triangle& tri)
				{
					return tri.i0 != tri.i1 && tri.i1 != tri.i2 && tri.i2 != tri.i0;
				};

			while (simplifiedTriangles.size() > targetTriangleCount)
			{
				float bestError = std::numeric_limits<float>::max();
				Edge bestEdge(-1, -1);
				FVector3 bestPos;
				Quadric bestQuadric;
				bool bestAIsBoundary = false;
				bool bestBIsBoundary = false;

				for (const Edge& edge : edges)
				{
					const FVector3 vertexA = simplifiedVertices[edge.a];
					const FVector3 vertexB = simplifiedVertices[edge.b];

					bool aIsBoundary = boundaryVertIndices.count(edge.a);
					bool bIsBoundary = boundaryVertIndices.count(edge.b);
					if (aIsBoundary || bIsBoundary) continue;

					// compute edge error
					Quadric edgeQuadric;
					edgeQuadric.Q = quadrics[edge.a].Q + quadrics[edge.b].Q;

					// find position to merged on the edge 
					FVector3 mergedPos;
					if (aIsBoundary)
					{
						mergedPos = vertexA;
					}
					else if (bIsBoundary)
					{
						mergedPos = vertexB;
					}
					else
					{
						// find optimal position
						FMatrix3x3 edgeQuadric3x3 = {
							edgeQuadric.Q[0][0], edgeQuadric.Q[0][1], edgeQuadric.Q[0][2],
							edgeQuadric.Q[1][0], edgeQuadric.Q[1][1], edgeQuadric.Q[1][2],
							edgeQuadric.Q[2][0], edgeQuadric.Q[2][1], edgeQuadric.Q[2][2] };

						if (fabs(edgeQuadric3x3.Determinant()) > 1e-6f) // valid
						{
							mergedPos = edgeQuadric3x3.Inverse() * FVector3(-edgeQuadric.Q[0][3], -edgeQuadric.Q[1][3], -edgeQuadric.Q[2][3]);
						}
						else
						{
							mergedPos = (vertexA + vertexB) * 0.5f; // center of edge
						}
					}

					// update best vertex info
					float error = edgeQuadric.Evaluate(mergedPos);
					if (error < bestError)
					{
						bestError = error;
						bestEdge = edge;
						bestPos = mergedPos;
						bestQuadric = edgeQuadric;
						bestAIsBoundary = aIsBoundary;
						bestBIsBoundary = bIsBoundary;
					}
				}

				// there's no edge to merge
				if (bestEdge.a == -1 || bestEdge.b == -1)
					break;

				// update vertex and quadric
				uint32_t keepIdx = bestEdge.a;
				uint32_t removeIdx = bestEdge.b;
				simplifiedVertices[keepIdx] = bestPos;
				quadrics[bestEdge.a] = bestQuadric;

				// update triangles
				std::vector<Triangle> updatedSimpTriangles;
				updatedSimpTriangles.reserve(simplifiedTriangles.size());
				for (Triangle& tri : simplifiedTriangles)
				{
					tri.i0 = tri.i0 == removeIdx ? keepIdx : tri.i0;
					tri.i1 = tri.i1 == removeIdx ? keepIdx : tri.i1;
					tri.i2 = tri.i2 == removeIdx ? keepIdx : tri.i2;
					if (isValidTriangle(tri)) updatedSimpTriangles.emplace_back(tri);
				}
				simplifiedTriangles = std::move(updatedSimpTriangles);

				CheckDuplicateTriangles(simplifiedTriangles);
				// update boundaries
				/*if (bestBIsBoundary)
				{
					boundaryVertIndices.erase(removeIdx);
					boundaryVertIndices.insert(keepIdx);
				}*/
				boundaryVertIndices.insert(removeIdx);
				boundaryVertIndices.insert(keepIdx);

				// update edges
				edges.clear();
				edgeUsage.clear();
				for (const Triangle& tri : simplifiedTriangles)
				{
					edges.insert(Edge(tri.i0, tri.i1));
					edges.insert(Edge(tri.i1, tri.i2));
					edges.insert(Edge(tri.i2, tri.i0));

					edgeUsage[Edge(tri.i0, tri.i1)]++;
					edgeUsage[Edge(tri.i1, tri.i2)]++;
					edgeUsage[Edge(tri.i2, tri.i0)]++;
				}

				//boundaryVertIndices.clear();
				//for (const auto& [edge, count] : edgeUsage)
				//{
				//	//assert(count <= 2);
				//	if (count == 1)
				//	{
				//		boundaryVertIndices.insert(edge.a);
				//		boundaryVertIndices.insert(edge.b);
				//	}
				//}
				for (auto x : edgeUsage)
				{
					//assert(x.second <= 2);
				}
			}

			// remap vertices
			std::unordered_set<int> usedVertices;
			usedVertices.reserve(simplifiedVertices.size());
			for (const Triangle& tri : simplifiedTriangles)
			{
				usedVertices.insert(tri.i0);
				usedVertices.insert(tri.i1);
				usedVertices.insert(tri.i2);
			}

			std::unordered_map<int, int> oldToNewIndex;
			int newIndex = 0;
			for (int idx : usedVertices)
			{
				oldToNewIndex[idx] = newIndex++;
			}

			std::vector<FVector3> remapedVertices(usedVertices.size());
			for (const auto& [oldIdx, newIdx] : oldToNewIndex)
			{
				remapedVertices[newIdx] = simplifiedVertices[oldIdx];
			}

			std::vector<Triangle> remapedTriangles;
			remapedTriangles.reserve(simplifiedTriangles.size());
			for (const Triangle& tri : simplifiedTriangles)
			{
				Triangle newTri;
				newTri.i0 = oldToNewIndex[tri.i0];
				newTri.i1 = oldToNewIndex[tri.i1];
				newTri.i2 = oldToNewIndex[tri.i2];
				newTri.Normal = tri.Normal;
				newTri.Color = tri.Color;
				remapedTriangles.emplace_back(newTri);
			}

			return std::make_pair(remapedVertices, remapedTriangles);
		}
	}
}