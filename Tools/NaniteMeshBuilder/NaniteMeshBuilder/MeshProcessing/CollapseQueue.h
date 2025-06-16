#pragma once
#include <vector>
#include <set>
#include <unordered_map>

#include "../Topology/Mesh.h"
#include "Collapse.h"

namespace nanite
{
	class CollapseQueue
	{
	public:
		CollapseQueue(const Mesh& mesh, const std::vector<Quadric>& quadrics, const std::set<uint32_t>& fixedVertices)
			: mMesh(mesh)
			, mQuadrics(quadrics)
			, mFixedVertices(fixedVertices)
		{

		}
		~CollapseQueue() = default;

		const Collapse& PickBest() const
		{
			return *mCollapseSet.begin();
		}

		void Insert(const Edge& e, int phase = 0)
		{
			bool bFixA = mFixedVertices.find(e.GetA()) != mFixedVertices.end();
			bool bFixB = mFixedVertices.find(e.GetB()) != mFixedVertices.end();
			if (bFixA && bFixB) return;

			Collapse collapse;
			collapse.Edge = e;
			collapse.Quadric.Q = mQuadrics[e.GetA()].Q + mQuadrics[e.GetB()].Q;
			collapse.Position = Collapse::FindOptimalPosition(collapse.Quadric, mMesh.Vertices[e.GetA()], mMesh.Vertices[e.GetB()], bFixA, bFixB);
			collapse.Error = collapse.Quadric.Evaluate(collapse.Position);
			collapse.Length = (mMesh.Vertices[e.GetA()] - mMesh.Vertices[e.GetB()]).Length();;
			collapse.bFixA = bFixA;
			collapse.bFixB = bFixB;
			collapse.Phase = phase;

			Insert(collapse);
		}

		void Insert(const Collapse& c)
		{
			auto [iter, b] = mCollapseSet.emplace(c);
			mEdgeToCollpaseMap[c.Edge] = iter;
		}

		int Erase(const Edge& e)
		{
			if (mEdgeToCollpaseMap.find(e) != mEdgeToCollpaseMap.end())
			{
				auto it = mEdgeToCollpaseMap[e];
				int phase = it->Phase;
				mEdgeToCollpaseMap.erase(e);
				mCollapseSet.erase(it);
				return phase;
			}
			return -1;
		}

		int Erase(const Collapse& c)
		{
			return Erase(c.Edge);
		}

		int Size() const { return static_cast<int>(mCollapseSet.size()); }
		void Reserve(size_t numExpectedElements) { mEdgeToCollpaseMap.reserve(utils::NextPrime(numExpectedElements * 2 + 1)); }

	private:
		std::set<Collapse> mCollapseSet;
		std::unordered_map<Edge, std::set<Collapse>::iterator> mEdgeToCollpaseMap;

		const Mesh& mMesh;
		const std::vector<Quadric>& mQuadrics;
		const std::set<uint32_t>& mFixedVertices;
	};
}