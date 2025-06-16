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

		inline const Mesh& GetLODMesh(int lod) const { return mLODMeshes[lod]; }

	private:
		std::string mName;

		std::vector<NaniteNode> mNodes;
		std::vector<Mesh> mLODMeshes;
	};
}