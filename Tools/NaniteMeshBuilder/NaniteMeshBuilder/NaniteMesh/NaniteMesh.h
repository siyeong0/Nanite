#pragma once
#include <string>
#include <vector>

#include <assimp/scene.h>

#include "../Math/Math.h"
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
		void PaintColorByCluster();


	private:
		static void mergeDuplicatedVertices(Mesh* mesh);
		static void mergeDuplicatedTriangles(Mesh* mesh);

	private:
		std::string mName;
	};
}