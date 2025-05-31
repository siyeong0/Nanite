#pragma once
#include <string>
#include <vector>

#include <assimp/scene.h>

#include "../../Math/Math.h"
#include "Geometry/Triangle.h"
#include "Geometry/Cluster.h"

namespace nanite
{
	class NaniteMesh
	{
	public:
		NaniteMesh() = default;
		~NaniteMesh() = default;
		NaniteMesh(std::vector<FVector3>&& vertices, std::vector<Triangle>&& triangles) : mVertices(vertices), mTriangles(triangles) {}
		NaniteMesh(const std::string& name) : mName(name) {}
		NaniteMesh(const NaniteMesh& other) = delete;
		NaniteMesh& operator=(const NaniteMesh& other) = delete;
		NaniteMesh(NaniteMesh&& other) noexcept = default;
		NaniteMesh& operator=(NaniteMesh&& other) noexcept = default;

		bool Build(int leafTriThreshold, int simpLevel = 0);

		bool LoadFromFile(const std::string& path);
		bool SaveToFile(const std::string& path) const;
		bool SaveToFbx(const std::string& path) const;

		std::vector<FVector3>& GetVerticesRef() { return mVertices; }
		const std::vector<FVector3>& GetVertices() const { return mVertices; }
		std::vector<Triangle>& GetTrianglesRef() { return mTriangles; }
		const std::vector<Triangle>& GetTriangles() const { return mTriangles; }

		inline int NumTriangles() const { return static_cast<int>(mTriangles.size()); }
		inline int NumVertices() const { return static_cast<int>(mVertices.size()); }

		void SetName(const std::string& name) { mName = name; }
	private:

		static void mergeVertices(
			const std::vector<FVector3>& inVertices, const std::vector<uint32_t>& inIndices,
			std::vector<FVector3>& outVertices, std::vector<uint32_t>& outIndices);

	private:
		std::vector<FVector3> mVertices;
		std::vector<Triangle> mTriangles;

		std::vector<Cluster> mClusters;

		std::string mName;
		std::string mExtension;
	};
}