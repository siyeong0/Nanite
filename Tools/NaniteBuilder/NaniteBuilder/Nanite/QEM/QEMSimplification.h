#pragma once
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

		inline void ComputePlane(const FVector3& v0, const FVector3& v1, const FVector3& v2, FVector3& normal, float& d) {
			FVector3 edge1 = v1 - v0;
			FVector3 edge2 = v2 - v0;
			normal = edge1.Cross(edge2).Norm();
			d = -normal.Dot(v0);
		}

		inline void ComputeVertexQuadrics(
			const std::vector<Triangle>& triangles,
			const std::vector<FVector3>& vertices,
			std::vector<Quadric>& outQuadrics)
		{
			outQuadrics.resize(vertices.size());

			for (const Triangle& tri : triangles)
			{
				FVector3 normal;
				float d;
				const FVector3& v0 = vertices[tri.i0];
				const FVector3& v1 = vertices[tri.i1];
				const FVector3& v2 = vertices[tri.i2];
				ComputePlane(v0, v1, v2, normal, d);

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

			float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
				- A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
				+ A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

			if (fabs(det) > 1e-6f)
			{
				// 간단한 역행렬 계산 (크래머 공식 등은 생략 가능)
				// 실제로는 고급 선형대수 라이브러리를 쓰는 게 안전함
				// 여기서는 생략하고 실패 처리만
				return false; // 간단화
			}
			return false;
		}

		inline float ComputeEdgeError(const Quadric& q1, const Quadric& q2, const FVector3& v1, const FVector3& v2, FVector3& outPos) {
			Quadric q = q1;
			for (int i = 0; i < 4; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					q.Q[i][j] += q2.Q[i][j];
				}
			}

			if (OptimalPosition(q, outPos))
			{
				return q.Evaluate(FVector4(outPos.x, outPos.y, outPos.z, 1.0f));
			}
			else
			{
				outPos = FVector3((v1.x + v2.x) * 0.5f, (v1.y + v2.y) * 0.5f, (v1.z + v2.z) * 0.5f);
				return q.Evaluate(FVector4(outPos.x, outPos.y, outPos.z, 1.0f));
			}
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

		//inline std::pair<std::vector<FVector3>, std::vector<Triangle>> SimplifyMesh(
		//	const std::vector<Triangle>& triangles,
		//	const std::vector<FVector3>& vertices,
		//	int targetVertexCount)
		//{
		//	auto [newVertices, newTriangles] = RemapMeshVertices(triangles, vertices);

		//	std::vector<Quadric> quadrics;
		//	ComputeVertexQuadrics(newTriangles, newVertices, quadrics);

		//	std::set<Edge> edges;
		//	for (const Triangle& tri : newTriangles)
		//	{
		//		edges.insert(Edge(tri.i0, tri.i1));
		//		edges.insert(Edge(tri.i1, tri.i2));
		//		edges.insert(Edge(tri.i2, tri.i0));
		//	}

		//	auto IsValidVertex = [](const FVector3& v) 
		//		{
		//		return !std::isnan(v.x);
		//		};

		//	auto CountValidVertices = [&]() 
		//		{
		//		int count = 0;
		//		for (const auto& v : newVertices)
		//			if (IsValidVertex(v)) ++count;
		//		return count;
		//		};

		//	while (CountValidVertices() > targetVertexCount)
		//	{
		//		float bestError = std::numeric_limits<float>::max();
		//		Edge bestEdge(-1, -1);
		//		FVector3 bestPos;

		//		for (const Edge& edge : edges)
		//		{
		//			if (!IsValidVertex(newVertices[edge.a]) || !IsValidVertex(newVertices[edge.b]))
		//				continue;

		//			FVector3 newPos;
		//			float error = ComputeEdgeError(
		//				quadrics[edge.a], quadrics[edge.b],
		//				newVertices[edge.a], newVertices[edge.b], newPos);

		//			if (error < bestError)
		//			{
		//				bestError = error;
		//				bestEdge = edge;
		//				bestPos = newPos;
		//			}
		//		}

		//		if (bestEdge.a == -1 || bestEdge.b == -1)
		//			break; // 더 이상 병합할 유효한 엣지가 없음

		//		// 병합 수행
		//		newVertices[bestEdge.a] = bestPos;
		//		for (int i = 0; i < 4; ++i)
		//			for (int j = 0; j < 4; ++j)
		//				quadrics[bestEdge.a].Q[i][j] += quadrics[bestEdge.b].Q[i][j];

		//		for (Triangle& tri : newTriangles)
		//		{
		//			if (tri.i0 == bestEdge.b) tri.i0 = bestEdge.a;
		//			if (tri.i1 == bestEdge.b) tri.i1 = bestEdge.a;
		//			if (tri.i2 == bestEdge.b) tri.i2 = bestEdge.a;
		//		}

		//		newVertices[bestEdge.b] = FVector3(std::numeric_limits<float>::quiet_NaN(), 0, 0);
		//		edges.erase(bestEdge);
		//	}

		//	// 유효한 삼각형 필터링
		//	std::vector<Triangle> filteredTriangles;
		//	for (const Triangle& tri : newTriangles) {
		//		if (!IsValidVertex(newVertices[tri.i0]) ||
		//			!IsValidVertex(newVertices[tri.i1]) ||
		//			!IsValidVertex(newVertices[tri.i2]))
		//			continue;

		//		// 정점이 서로 달라야 함 (degenerate triangle 제거)
		//		if (tri.i0 == tri.i1 || tri.i1 == tri.i2 || tri.i2 == tri.i0)
		//			continue;

		//		filteredTriangles.push_back(tri);
		//	}

		//	return RemapMeshVertices(filteredTriangles, newVertices);
		//}

		inline std::pair<std::vector<FVector3>, std::vector<Triangle>> SimplifyMesh(
			const std::vector<Triangle>& triangles,
			const std::vector<FVector3>& vertices,
			int targetVertexCount)
		{
			auto [newVertices, newTriangles] = RemapMeshVertices(triangles, vertices);

			std::vector<Quadric> quadrics;
			ComputeVertexQuadrics(newTriangles, newVertices, quadrics);

			std::set<Edge> edges;
			std::map<Edge, int> edgeUsage;

			for (const Triangle& tri : newTriangles)
			{
				auto addEdge = [&](int a, int b) {
					Edge e(a, b);
					edges.insert(e);
					edgeUsage[e]++;
					};
				addEdge(tri.i0, tri.i1);
				addEdge(tri.i1, tri.i2);
				addEdge(tri.i2, tri.i0);
			}

			// 경계 정점 감지
			std::set<int> fixedVertices;
			for (const auto& [edge, count] : edgeUsage)
			{
				if (count == 1) // 삼각형 한 개에만 포함된 엣지 → 경계
				{
					fixedVertices.insert(edge.a);
					fixedVertices.insert(edge.b);
				}
			}

			auto IsValidVertex = [](const FVector3& v) {
				return !std::isnan(v.x);
				};

			auto CountValidVertices = [&]() {
				int count = 0;
				for (const auto& v : newVertices)
					if (IsValidVertex(v)) ++count;
				return count;
				};

			while (CountValidVertices() > targetVertexCount)
			{
				float bestError = std::numeric_limits<float>::max();
				Edge bestEdge(-1, -1);
				FVector3 bestPos;

				for (const Edge& edge : edges)
				{
					if (!IsValidVertex(newVertices[edge.a]) || !IsValidVertex(newVertices[edge.b]))
						continue;

					// 경계 정점이 포함된 엣지는 병합하지 않음
					if (fixedVertices.count(edge.a) || fixedVertices.count(edge.b))
						continue;

					FVector3 newPos;
					float error = ComputeEdgeError(
						quadrics[edge.a], quadrics[edge.b],
						newVertices[edge.a], newVertices[edge.b], newPos);

					if (error < bestError)
					{
						bestError = error;
						bestEdge = edge;
						bestPos = newPos;
					}
				}

				if (bestEdge.a == -1 || bestEdge.b == -1)
					break; // 병합할 엣지가 없음

				// 병합 수행
				newVertices[bestEdge.a] = bestPos;
				for (int i = 0; i < 4; ++i)
					for (int j = 0; j < 4; ++j)
						quadrics[bestEdge.a].Q[i][j] += quadrics[bestEdge.b].Q[i][j];

				for (Triangle& tri : newTriangles)
				{
					if (tri.i0 == bestEdge.b) tri.i0 = bestEdge.a;
					if (tri.i1 == bestEdge.b) tri.i1 = bestEdge.a;
					if (tri.i2 == bestEdge.b) tri.i2 = bestEdge.a;
				}

				newVertices[bestEdge.b] = FVector3(std::numeric_limits<float>::quiet_NaN(), 0, 0);
				edges.erase(bestEdge);
			}

			// 유효한 삼각형 필터링
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