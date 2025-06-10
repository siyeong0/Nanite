#pragma once
#include "../Topology/Edge.h"
#include "Quadric.h"

namespace nanite
{
	struct Collapse
	{
		Edge Edge;
		Quadric Quadric;
		float Error = std::numeric_limits<float>::max();
		FVector3 Position = FVector3::Zero();
		float Length = 0;
		bool bFixA = false;
		bool bFixB = false;
		int Phase = 0;

		inline bool operator<(const Collapse& other) const
		{
			return std::tie(Phase, Error, Length, Edge) < std::tie(other.Phase, other.Error, other.Length, other.Edge);
		}

		static inline FVector3 FindOptimalPosition(const nanite::Quadric& q, const FVector3& vertexA, const FVector3& vertexB, bool bFixA = false, bool bFixB = false)
		{
			FVector3 optimalPos;
			FMatrix3x3 edgeQuadric3x3 = {
				q.Q[0][0], q.Q[0][1], q.Q[0][2],
				q.Q[1][0], q.Q[1][1], q.Q[1][2],
				q.Q[2][0], q.Q[2][1], q.Q[2][2] };
			FVector3 edgeVector3 = { -q.Q[0][3], -q.Q[1][3], -q.Q[2][3] };
			if (bFixA)
			{
				optimalPos = vertexA;
			}
			else if (bFixB)
			{
				optimalPos = vertexB;
			}
			else if (fabs(edgeQuadric3x3.Determinant()) > 1e-6f) // valid
			{
				optimalPos = edgeQuadric3x3.Inverse() * edgeVector3;
			}
			else
			{
				optimalPos = (vertexA + vertexB) * 0.5f; // center of edge
			}
			return optimalPos;
		}
	};
}