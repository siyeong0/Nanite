#pragma once
#include <set>

#include "../Utils/Utils.h"
#include "../Topology/Cluster.h"

namespace nanite
{
	class NaniteNode
	{
	public:
		NaniteNode()
			: mID(msIDGenerator())
			, mChilds()
			, mClusterData()
		{
			
		}

		NaniteNode(Cluster clusterData)
			: mID(msIDGenerator())
			, mChilds()
			, mClusterData(clusterData)
		{

		}

		void SetParent(NaniteNode* parent)
		{
			assert(parent != nullptr);
			parent->AddChild(this);
		}

		void AddChild(NaniteNode* child)
		{
			assert(child != nullptr);
			mChilds.emplace(child);
			mClusterData.Bounds.Encapsulate(child->mClusterData.Bounds);
		}

		inline bool IsLeaf() const { return mChilds.empty(); }

		inline const utils::UUID& GetID() const { return mID; }
		inline const std::set<NaniteNode*>& GetChilds() const { return mChilds; }
		inline const Cluster& GetClusterData() const { return mClusterData; }
		inline void SetClusterData(const Cluster& clusterData) { mClusterData = clusterData; }

	private:
		const utils::UUID mID;
		std::set<NaniteNode*> mChilds;
		Cluster mClusterData;

		inline static utils::UUIDGnerator msIDGenerator;
	};
}