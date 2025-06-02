#pragma once
#include <string>
#include <vector>

#include <assimp/scene.h>

#include "../Math/Math.h"
#include "../Topology/Triangle.h"
#include "../Topology/Cluster.h"
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

		int GetDepth() const { return static_cast<int>(mLevels.size()); }
		const Mesh& GetMesh(int level = 0) const { return mLevels[level].Mesh; }
		const std::vector<Cluster>& GetClusters(int level = 0) const { return mLevels[level].Clusters; }

		const std::string& GetName() const { return mName; }
		void SetName(const std::string& name) { mName = name; }

	private:
		static void mergeDuplicatedVertices(Mesh* mesh);
		static void mergeDuplicatedTriangles(Mesh* mesh);
	private:
		struct Level
		{
			Mesh Mesh;
			std::vector<Cluster> Clusters;
		};
		std::vector<Level> mLevels;

		std::string mName;
	};
}