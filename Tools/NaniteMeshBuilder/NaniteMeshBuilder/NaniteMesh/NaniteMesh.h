#pragma once
#include <string>

#include "../Topology/Mesh.h"

namespace nanite
{
	class NaniteMesh
	{
	public:
		NaniteMesh() = default;
		~NaniteMesh() = default;
		NaniteMesh(const std::string& name) : mName(name) {}
		NaniteMesh(const NaniteMesh& other) = delete;
		NaniteMesh& operator=(const NaniteMesh& other) = delete;
		NaniteMesh(NaniteMesh&& other) noexcept = default;
		NaniteMesh& operator=(NaniteMesh&& other) noexcept = default;

		bool Build(const Mesh& originMesh, int leafTriThreshold);

	private:
		std::string mName;

		struct Node
		{
			int LOD = -1;

			Node* Parent = nullptr;
			std::vector<Node*> Childs;
		};
	};
}