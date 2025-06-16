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
			: ID(IDGenerator())
			, Parent(nullptr)
			, Childs()
			, ClusterData()
		{
			
		}

		NaniteNode(Cluster clusterData)
			: ID(IDGenerator())
			, Parent(nullptr)
			, Childs()
			, ClusterData(clusterData)
		{

		}

		void SetParent(NaniteNode* parent)
		{
			assert(parent != nullptr);
			Parent = parent;
			parent->Childs.emplace(this);
		}

		void AddChild(NaniteNode* child)
		{
			assert(child != nullptr);
			child->SetParent(this);
			Childs.emplace(child);
		}

		inline bool IsLeaf() const { return Childs.empty(); }
		inline bool IsRoot() const { return Parent == nullptr; }

		inline const utils::UUID& GetID() const { return ID; }
		inline const NaniteNode* GetParent() const { return Parent; }
		inline const std::set<NaniteNode*>& GetChilds() const { return Childs; }
		inline const Cluster& GetClusterData() const { return ClusterData; }
		inline void SetClusterData(const Cluster& clusterData) { ClusterData = clusterData; }

	private:
		const utils::UUID ID;
		const NaniteNode* Parent = nullptr;
		std::set<NaniteNode*> Childs;
		Cluster ClusterData;

		inline static utils::UUIDGnerator IDGenerator;
	};
}