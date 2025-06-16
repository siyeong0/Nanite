#pragma once
#include <string>

#include "../Topology/Mesh.h"
#include "NaniteNode.h"

namespace nanite
{
	class NaniteMesh
	{
	public:
		NaniteMesh();
		NaniteMesh(const std::string name);
		~NaniteMesh() = default;
		NaniteMesh(const std::string& name) : mName(name) {}
		NaniteMesh(const NaniteMesh& other) = delete;
		NaniteMesh& operator=(const NaniteMesh& other) = delete;
		NaniteMesh(NaniteMesh&& other) noexcept = default;
		NaniteMesh& operator=(NaniteMesh&& other) noexcept = default;

		bool Build(const Mesh& originMesh, int leafTriThreshold);

		inline int GetLODDepth() const { return static_cast<int>(mLODMeshes.size()); };
		inline const Mesh& GetLODMesh(int lod) const { return mLODMeshes[lod]; }

		void PaintByCluster();

	private:
		std::string mName;

		std::vector<std::vector<NaniteNode>> mNodes;
		std::vector<Mesh> mLODMeshes;
	};
}