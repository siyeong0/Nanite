#pragma once
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
#include "../../Topology/Edge.h"
#include "../../Topology/Triangle.h"
#include "Quadraic.h"

namespace nanite
{
	namespace qem
	{
		inline void ComputeVertexQuadrics(
			const std::vector<Triangle>& triangles,
			const std::vector<FVector3>& vertices,
			std::vector<Quadric>& outQuadrics)
		{
			outQuadrics.resize(vertices.size());

			for (const Triangle& tri : triangles)
			{
				const FVector3& v0 = vertices[tri.i0];
				const FVector3& v1 = vertices[tri.i1];
				const FVector3& v2 = vertices[tri.i2];
				// compute plane
				FVector3 edge1 = v1 - v0;
				FVector3 edge2 = v2 - v0;
				FVector3 normal = edge1.Cross(edge2).Norm();
				float d = -normal.Dot(v0);

				outQuadrics[tri.i0].AddPlane(normal, d);
				outQuadrics[tri.i1].AddPlane(normal, d);
				outQuadrics[tri.i2].AddPlane(normal, d);
			}
		}

		inline bool OptimalPosition(const Quadric& Q, FVector3& outPos)
		{
			FMatrix3x3 A;
			FVector3 b;

			for (int i = 0; i < 3; ++i)
			{
				b[i] = -Q.Q[i][3];
				for (int j = 0; j < 3; ++j)
				{
					A[i][j] = Q.Q[i][j];
				}
			}

			float det = A.Determinant();
			if (fabs(det) < 1e-6f)
				return false;

			FVector3 result = A.Inverse() * b;
			outPos = result;
			return true;
		}

		inline float ComputeEdgeError(
			const Quadric& q1, const Quadric& q2,
			const FVector3& v1, const FVector3& v2,
			bool bFix1, bool bFix2, FVector3& outPos)
		{
			Quadric q = q1;
			for (int i = 0; i < 4; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					q.Q[i][j] += q2.Q[i][j];
				}
			}

			if (bFix1)
			{
				outPos = v1;
			}
			else if (bFix2)
			{
				outPos = v2;
			}
			else if (!OptimalPosition(q, outPos))
			{
				outPos = (v1 + v2) * 0.5f;
			}

			return q.Evaluate(FVector4(outPos.x, outPos.y, outPos.z, 1.0f));
		}

		inline std::pair<std::vector<FVector3>, std::vector<Triangle>>RemapMeshVertices(
			const std::vector<Triangle>& triangles,
			const std::vector<FVector3>& vertices)
		{
			// 1. 사용된 정점 집합 수집
			std::unordered_set<int> usedVertices;
			for (const Triangle& tri : triangles)
			{
				usedVertices.insert(tri.i0);
				usedVertices.insert(tri.i1);
				usedVertices.insert(tri.i2);
			}

			// 2. old 인덱스 -> new 인덱스 매핑 생성
			std::unordered_map<int, int> oldToNewIndex;
			int newIndex = 0;
			for (int idx : usedVertices)
			{
				oldToNewIndex[idx] = newIndex++;
			}

			// 3. 새 vertices 생성
			std::vector<FVector3> newVertices(usedVertices.size());
			for (const auto& [oldIdx, newIdx] : oldToNewIndex)
			{
				newVertices[newIdx] = vertices[oldIdx];
			}

			// 4. 삼각형 인덱스 재매핑
			std::vector<Triangle> newTriangles;
			newTriangles.reserve(triangles.size());
			for (const Triangle& tri : triangles) {
				Triangle newTri;
				newTri.i0 = oldToNewIndex[tri.i0];
				newTri.i1 = oldToNewIndex[tri.i1];
				newTri.i2 = oldToNewIndex[tri.i2];
				newTri.Normal = tri.Normal;
				newTri.Color = tri.Color;
				newTriangles.push_back(newTri);
			}

			return std::make_pair(newVertices, newTriangles);
		}

		inline std::pair<std::vector<FVector3>, std::vector<Triangle>> SimplifyMesh(
			int targetTriangleCount,
			const std::vector<Triangle>& triangles,
			const std::vector<FVector3>& vertices)
		{
			auto [newVertices, newTriangles] = RemapMeshVertices(triangles, vertices);

			std::vector<Quadric> quadrics;
			ComputeVertexQuadrics(newTriangles, newVertices, quadrics);

			std::set<Edge> edges;
			std::map<Edge, int> edgeUsage;

			for (const Triangle& tri : newTriangles)
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

			// lock boundary edges
			std::set<int> fixedVertices;
			for (const auto& [edge, count] : edgeUsage)
			{
				if (count == 1)
				{
					fixedVertices.insert(edge.a);
					fixedVertices.insert(edge.b);
				}
			}

			auto IsValidVertex = [](const FVector3& v) { return !std::isnan(v.x); };

			auto CountValidTriangles = [&]()
				{
					int count = 0;
					for (const auto& tri : newTriangles)
					{
						if (!IsValidVertex(newVertices[tri.i0]) ||
							!IsValidVertex(newVertices[tri.i1]) ||
							!IsValidVertex(newVertices[tri.i2]))
							continue;

						if (tri.i0 == tri.i1 || tri.i1 == tri.i2 || tri.i2 == tri.i0)
							continue;

						count++;
					}
					return count;
				};

			while (CountValidTriangles() > targetTriangleCount)
			{
				float bestError = std::numeric_limits<float>::max();
				Edge bestEdge(-1, -1);
				FVector3 bestPos;

				for (const Edge& edge : edges)
				{
					if (!IsValidVertex(newVertices[edge.a]) || !IsValidVertex(newVertices[edge.b]))
						continue;

					if (fixedVertices.count(edge.a) && fixedVertices.count(edge.b))
						continue;

					FVector3 newPos;
					float error = ComputeEdgeError(
						quadrics[edge.a], quadrics[edge.b],
						newVertices[edge.a], newVertices[edge.b],
						fixedVertices.count(edge.a), fixedVertices.count(edge.b),
						newPos);

					if (error < bestError)
					{
						bestError = error;
						bestEdge = edge;
						bestPos = newPos;
					}
				}

				if (bestEdge.a == -1 || bestEdge.b == -1)
					break;

				if (fixedVertices.count(bestEdge.a))
					bestPos = newVertices[bestEdge.a];
				else if (fixedVertices.count(bestEdge.b))
					bestPos = newVertices[bestEdge.b];

				newVertices[bestEdge.a] = bestPos;
				for (int i = 0; i < 4; ++i)
					for (int j = 0; j < 4; ++j)
						quadrics[bestEdge.a].Q[i][j] += quadrics[bestEdge.b].Q[i][j];

				if (fixedVertices.count(bestEdge.b))
				{
					fixedVertices.erase(bestEdge.b);
					fixedVertices.insert(bestEdge.a);
				}

				for (Triangle& tri : newTriangles)
				{
					if (tri.i0 == bestEdge.b) tri.i0 = bestEdge.a;
					if (tri.i1 == bestEdge.b) tri.i1 = bestEdge.a;
					if (tri.i2 == bestEdge.b) tri.i2 = bestEdge.a;
				}

				newVertices[bestEdge.b] = FVector3(std::numeric_limits<float>::quiet_NaN(), 0, 0);
				std::set<Edge> updatedEdges;
				for (const Edge& edge : edges)
				{
					if (edge.a == bestEdge.b || edge.b == bestEdge.b)
						continue; // 제거
					updatedEdges.insert(edge);
				}
				edges = std::move(updatedEdges);
			}

			std::vector<Triangle> filteredTriangles;
			for (const Triangle& tri : newTriangles)
			{
				if (!IsValidVertex(newVertices[tri.i0]) ||
					!IsValidVertex(newVertices[tri.i1]) ||
					!IsValidVertex(newVertices[tri.i2]))
					continue;

				if (tri.i0 == tri.i1 || tri.i1 == tri.i2 || tri.i2 == tri.i0)
					continue;

				filteredTriangles.push_back(tri);
			}

			return RemapMeshVertices(filteredTriangles, newVertices);
		}
	}
}