#pragma once
#include <vector>
#include <string>

#include <assimp/scene.h>

#include "Edge.h"
#include "../Math/Math.h"

namespace nanite
{
	struct Mesh
	{
		std::string Name;

		std::vector<FVector3> Vertices;
		std::vector<uint32_t> Indices;
		// per triangles
		std::vector<FVector3> Normals;
		std::vector<FVector3> Colors;

		std::vector<aiMaterial*> Materials;

		const std::string DEFAULT_FORMAT = ".fbx";

		Mesh() = default;
		Mesh(const std::string& name) : Name(name) {}
		~Mesh();
		Mesh(const Mesh& other);
		Mesh(Mesh&& other) noexcept;
		Mesh& operator=(const Mesh& other);

		void ComputeNormals();
		void MergeDuplicatedVertices();
		void RemoveUnusedVertices();
		static std::vector<Mesh> ExractUnconnectedMeshes(const Mesh& mesh);

		std::tuple<uint32_t&, uint32_t&, uint32_t&> GetTriangleIndices(int index);
		const std::tuple<const uint32_t&, const uint32_t&, const uint32_t&> GetTriangleIndices(int index) const;
		std::tuple<FVector3&, FVector3&, FVector3&> GetTriangleVertices(int index);
		const std::tuple<const FVector3&, const FVector3&, const FVector3&> GetTriangleVertices(int index) const;
		std::tuple<Edge, Edge, Edge> GetTriangleEdges(int index);
		const std::tuple<const Edge, const Edge, const Edge> GetTriangleEdges(int index) const;

		inline int NumVertices() const { return static_cast<int>(Vertices.size()); }
		inline int NumTriangles() const { return static_cast<int>(Indices.size() / 3); }

		bool LoadFromFile(const std::string& path);
		bool SaveToFile(const std::string& path) const;
		bool SaveToFile(const std::string& path, const std::string& format) const;
		bool SaveToFile(const std::string& directory, const std::string& name, const std::string& format) const;
		bool SaveToFileDbg(const std::string& directory, const std::string& name, const std::string& format) const;

	private:
		static aiMaterial* deepCopyMaterial(const aiMaterial* src);
		static std::ostream* msOutStream;
	};
}