#include "Mesh.h"

#include <iostream>
#include <algorithm>
#include <set>
#include <queue>
#include <fstream>
#include <filesystem>

#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "../Utils/Utils.h"

namespace nanite
{
	Mesh::~Mesh()
	{
		for (aiMaterial* mat : Materials) delete mat;
		Materials.clear();
	}

	Mesh::Mesh(const Mesh& other)
		: Vertices(other.Vertices)
		, Indices(other.Indices)
		, Normals(other.Normals)
		, Colors(other.Colors)
	{

	}

	Mesh::Mesh(Mesh&& other) noexcept
		: Vertices(std::move(other.Vertices))
		, Indices(std::move(other.Indices))
		, Normals(std::move(other.Normals))
		, Colors(std::move(other.Colors))
	{

	}

	Mesh& Mesh::operator=(const Mesh& other)
	{
		Vertices = other.Vertices;
		Indices = other.Indices;
		Normals = other.Normals;
		Colors = other.Colors;
		return *this;
	}

	void Mesh::ComputeNormals()
	{
		Normals.clear();
		Normals.reserve(NumTriangles());
		for (int triIdx = 0; triIdx < NumTriangles(); ++triIdx)
		{
			Normals.emplace_back(utils::ComputeNormal(GetTriangleVertices(triIdx)));
		}
	}

	void Mesh::MergeDuplicateVertices()
	{
		std::unordered_map<FVector3, std::set<uint32_t>, utils::FVector3Hasher> spatialGrid;
		spatialGrid.reserve(utils::NextPrime(2 * NumVertices() + 1));
		for (int i = 0; i < NumVertices(); ++i)
		{
			spatialGrid[Vertices[i]].emplace(i);
		}

		std::unordered_map<uint32_t, std::set<uint32_t>> neighborVerts;
		neighborVerts.reserve(utils::NextPrime(2 * NumVertices() + 1));
		for (int triIdx = 0; triIdx < NumTriangles(); ++triIdx)
		{
			auto [e0, e1, e2] = GetTriangleEdges(triIdx);
			for (const Edge& e : { e0, e1, e2 })
			{
				neighborVerts[e.GetA()].emplace(e.GetB());
				neighborVerts[e.GetB()].emplace(e.GetA());
			}
		}

		std::unordered_map<Edge, std::set<uint32_t>> edgeToTriangleMap;
		edgeToTriangleMap.reserve(utils::NextPrime(2 * (NumTriangles() * 3) + 1));
		for (int triIdx = 0; triIdx < NumTriangles(); ++triIdx)
		{
			auto [e0, e1, e2] = GetTriangleEdges(triIdx);
			edgeToTriangleMap[e0].emplace(triIdx);
			edgeToTriangleMap[e1].emplace(triIdx);
			edgeToTriangleMap[e2].emplace(triIdx);
		}

		std::unordered_map<uint32_t, uint32_t> mergeMap;
		mergeMap.rehash(utils::NextPrime(2 * Indices.size() + 1));

		for (auto& [vertex, indexSet] : spatialGrid)
		{
			std::vector<uint32_t> indices(indexSet.begin(), indexSet.end());
			if (indices.size() < 2) continue; // No duplicates

			std::priority_queue<
				std::pair<float, Edge>,
				std::vector<std::pair<float, Edge>>,
				std::greater<std::pair<float, Edge>>> mergeQueue;

			for (int i = 0; i < indices.size(); ++i)
			{
				for (int j = i + 1; j < indices.size(); ++j)
				{
					Edge edge(indices[i], indices[j]);
					float dist = FVector3::Distance(Vertices[indices[i]], Vertices[indices[j]]);
					mergeQueue.emplace(dist, edge);
				}
			}

			std::vector<uint32_t> mergedIndices;
			while (mergeQueue.size() > 0)
			{
				auto [dist, edge] = mergeQueue.top();
				mergeQueue.pop();

				uint32_t a = edge.GetA();
				uint32_t b = edge.GetB();

				if (std::find(mergedIndices.begin(), mergedIndices.end(), a) != mergedIndices.end() ||
					std::find(mergedIndices.begin(), mergedIndices.end(), b) != mergedIndices.end())
				{
					continue; // Skip already merged vertices
				}

				const std::set<uint32_t>& neighborsA = neighborVerts[a];
				const std::set<uint32_t>& neighborsB = neighborVerts[b];
				std::set<uint32_t> mergedNeighbors;
				std::set_union(
					neighborsA.begin(), neighborsA.end(),
					neighborsB.begin(), neighborsB.end(),
					std::inserter(mergedNeighbors, mergedNeighbors.end()));

				std::unordered_map<uint32_t, std::set<uint32_t>> mergedNeighborToTriangleMap;
				mergedNeighborToTriangleMap.reserve(utils::NextPrime(2 * mergedNeighbors.size() + 1));
				for (uint32_t neighbor : neighborsA)
				{
					if (neighbor == b) continue;
					mergedNeighborToTriangleMap[neighbor].insert(
						edgeToTriangleMap[Edge(a, neighbor)].begin(), edgeToTriangleMap[Edge(a, neighbor)].end());
				}
				for (uint32_t neighbor : neighborsB)
				{
					if (neighbor == a) continue;
					mergedNeighborToTriangleMap[neighbor].insert(
						edgeToTriangleMap[Edge(b, neighbor)].begin(), edgeToTriangleMap[Edge(b, neighbor)].end());
				}

				for (const auto& [neighbor, triangles] : mergedNeighborToTriangleMap)
				{
					if (triangles.size() > 1) // Non-manifold edge exists
					{
						goto SKIP_MERGING;
					}
				}

				for (const auto& [neighbor, triangles] : mergedNeighborToTriangleMap)
				{
					// Merge the edge
					spatialGrid[vertex].erase(b);
					mergeMap[b] = a;

					neighborVerts[a].insert(neighborVerts[b].begin(), neighborVerts[b].end());
					neighborVerts[a].erase(b);
					neighborVerts[a].erase(a);
					neighborVerts.erase(b);

					for (uint32_t neighbor : neighborVerts[a])
					{
						edgeToTriangleMap[Edge(a, neighbor)].insert(
							edgeToTriangleMap[Edge(b, neighbor)].begin(), edgeToTriangleMap[Edge(b, neighbor)].end());
						edgeToTriangleMap.erase(Edge(b, neighbor));
					}

					mergedIndices.emplace_back(b);
				}
			SKIP_MERGING:;
			}
		}

		std::vector<FVector3> mergedVertices;
		std::vector<uint32_t> mergedIndices;
		mergedVertices.reserve(Vertices.size());
		mergedVertices.reserve(Indices.size());

		std::unordered_map<uint32_t, uint32_t> indexMap; // Maps old index to new index
		indexMap.reserve(utils::NextPrime(2 * Vertices.size() + 1));
		for (auto& [vertex, indexSet] : spatialGrid)
		{
			for (uint32_t index : indexSet)
			{
				const FVector3& v = Vertices[index];
				mergedVertices.emplace_back(v);
				indexMap[index] = static_cast<uint32_t>(mergedVertices.size() - 1);
			}
		}

		for (uint32_t oldIndex : Indices)
		{
			uint32_t newIndex = -1;
			if (indexMap.find(oldIndex) != indexMap.end())
			{
				newIndex = indexMap[oldIndex];
			}
			else
			{
				assert(mergeMap.find(oldIndex) != mergeMap.end());
				newIndex = indexMap[mergeMap[oldIndex]];
			}
			mergedIndices.emplace_back(newIndex);
		}


		Vertices = std::move(mergedVertices);
		Indices = std::move(mergedIndices);
	}

	std::tuple<uint32_t&, uint32_t&, uint32_t&> Mesh::GetTriangleIndices(int index)
	{
		return std::tie(Indices[3 * index + 0], Indices[3 * index + 1], Indices[3 * index + 2]);
	}

	const std::tuple<const uint32_t&, const uint32_t&, const uint32_t&> Mesh::GetTriangleIndices(int index) const
	{
		return std::tie(Indices[3 * index + 0], Indices[3 * index + 1], Indices[3 * index + 2]);
	}

	std::tuple<FVector3&, FVector3&, FVector3&> Mesh::GetTriangleVertices(int index)
	{
		auto [i0, i1, i2] = GetTriangleIndices(index);
		return std::tie(Vertices[i0], Vertices[i1], Vertices[i2]);
	}

	const std::tuple<const FVector3&, const FVector3&, const FVector3&> Mesh::GetTriangleVertices(int index) const
	{
		auto [i0, i1, i2] = GetTriangleIndices(index);
		return std::tie(Vertices[i0], Vertices[i1], Vertices[i2]);
	}

	std::tuple<Edge, Edge, Edge> Mesh::GetTriangleEdges(int index)
	{
		auto [i0, i1, i2] = GetTriangleIndices(index);
		return std::make_tuple(Edge(i0, i1), Edge(i1, i2), Edge(i2, i0));
	}

	const std::tuple<const Edge, const Edge, const Edge> Mesh::GetTriangleEdges(int index) const
	{
		auto [i0, i1, i2] = GetTriangleIndices(index);
		return std::make_tuple(Edge(i0, i1), Edge(i1, i2), Edge(i2, i0));
	}

	bool Mesh::LoadFromFile(const std::string& path)
	{
		Assimp::Importer importer;
		const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
		if (!scene || !scene->HasMeshes())
		{
			std::cerr << "Failed to load model\n";
			return false;
		}

		const aiMesh* mesh = scene->mMeshes[0];
		const int numVertices = mesh->mNumVertices;
		const int numFaces = mesh->mNumFaces;
		const int numIndices = numFaces * 3;
		const bool hasNormals = mesh->HasNormals();

		Vertices.clear();
		Vertices.reserve(numVertices);
		for (int i = 0; i < numVertices; ++i)
		{
			aiVector3D vertex = mesh->mVertices[i];
			Vertices.emplace_back(vertex.x, vertex.y, vertex.z);
		}

		Indices.clear();
		Indices.reserve(numIndices);
		for (int i = 0; i < numFaces; ++i)
		{
			aiFace face = mesh->mFaces[i];
			Indices.emplace_back(face.mIndices[0]);
			Indices.emplace_back(face.mIndices[1]);
			Indices.emplace_back(face.mIndices[2]);
		}

		MergeDuplicateVertices();

		ComputeNormals();
		Colors.resize(NumTriangles(), { 1.f, 1.f, 1.f });

		Materials.clear();
		for (unsigned int i = 0; i < scene->mNumMaterials; ++i)
		{
			aiMaterial* mat = deepCopyMaterial(scene->mMaterials[i]);
			Materials.push_back(mat);
		}

		return true;
	}

	bool Mesh::SaveToFile(const std::string& path)
	{
		std::string directory = utils::ExtractDirectory(path);
		std::string file = utils::ExtractFileName(path);
		return SaveToFile(directory, file);
	}

	bool Mesh::SaveToFile(const std::string& directory, const std::string& file)
	{
		std::string fileName = utils::ExtractFileName(file);
		std::string extension = utils::ExtractExtension(file);
		return SaveToFile(directory, fileName, extension);
	}

	bool Mesh::SaveToFile(const std::string& directory, const std::string& name, const std::string& format)
	{
		std::string extension = format;
		if (extension == "")
		{
			extension = DEFAULT_FORMAT;
		}
		else if (extension[0] != '.')
		{
			extension = "." + format;
		}

		Assimp::Importer importer;
		if (!importer.IsExtensionSupported(format))
		{
			return false;
		}

		const std::vector<FVector3>& outVertices = Vertices;
		const std::vector<uint32_t>& outIndices = Indices;
		std::vector<FVector3> outNormals;
		std::vector<FVector3> outColors;
		outNormals.resize(NumVertices());
		outColors.resize(NumVertices());
		if (Colors.size() < NumTriangles()) Colors.resize(NumTriangles(), { 1.f, 1.f, 1.f });
		for (int i = 0; i < NumTriangles(); ++i)
		{
			auto [i0, i1, i2] = GetTriangleIndices(i);
			outNormals[i0] = Normals[i];
			outNormals[i1] = Normals[i];
			outNormals[i2] = Normals[i];
			outColors[i0] = Colors[i];
			outColors[i1] = Colors[i];
			outColors[i2] = Colors[i];
		}

		aiScene* outScene = new aiScene();
		outScene->mRootNode = new aiNode();

		aiMesh* aimesh = new aiMesh();
		aimesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
		aimesh->mMaterialIndex = 0;

		aimesh->mNumVertices = static_cast<unsigned int>(outVertices.size());
		aimesh->mVertices = new aiVector3D[aimesh->mNumVertices];
		aimesh->mNormals = new aiVector3D[aimesh->mNumVertices];
		aimesh->mColors[0] = new aiColor4D[aimesh->mNumVertices];
		for (int i = 0; i < static_cast<int>(aimesh->mNumVertices); ++i)
		{
			aimesh->mVertices[i] = aiVector3D(outVertices[i].x, outVertices[i].y, outVertices[i].z);
			aimesh->mNormals[i] = aiVector3D(outNormals[i].x, outNormals[i].y, outNormals[i].z);
			aimesh->mColors[0][i] = aiColor4D(outColors[i].x, outColors[i].y, outColors[i].z, 1.0f);
		}
		aimesh->mNumFaces = static_cast<unsigned int>(NumTriangles());
		aimesh->mFaces = new aiFace[aimesh->mNumFaces];
		for (int i = 0; i < static_cast<int>(aimesh->mNumFaces); ++i)
		{
			aimesh->mFaces[i].mNumIndices = 3;
			aimesh->mFaces[i].mIndices = new unsigned int[3];
			aimesh->mFaces[i].mIndices[0] = outIndices[3 * i + 0];
			aimesh->mFaces[i].mIndices[1] = outIndices[3 * i + 1];
			aimesh->mFaces[i].mIndices[2] = outIndices[3 * i + 2];
		}

		outScene->mNumMeshes = 1;
		outScene->mMeshes = new aiMesh * [1];
		outScene->mMeshes[0] = aimesh;

		outScene->mRootNode->mNumMeshes = 1;
		outScene->mRootNode->mMeshes = new unsigned int[1];
		outScene->mRootNode->mMeshes[0] = 0;

		outScene->mMaterials = new aiMaterial * [std::max(1u, static_cast<unsigned int>(Materials.size()))];
		outScene->mNumMaterials = static_cast<unsigned int>(Materials.size());
		if (!Materials.empty())
		{
			for (size_t i = 0; i < Materials.size(); ++i)
			{
				outScene->mMaterials[i] = deepCopyMaterial(Materials[i]);
			}
		}
		else
		{
			outScene->mMaterials[0] = new aiMaterial();
			aiColor4D color(1.0f, 1.0f, 1.0f, 1.0f);
			outScene->mMaterials[0]->AddProperty(&color, 1, AI_MATKEY_COLOR_DIFFUSE);
			outScene->mNumMaterials = 1;
		}

		if (!std::filesystem::exists(directory))
		{
			if (std::filesystem::create_directory(directory))
			{
				std::cout << "Folder" << "\'" << directory << "\' " << "created successfully.\n";
			}
			else
			{
				std::cout << "Failed to create folder" << "\'" << directory << "\' " << ".\n";
				return false;
			}
		}

		Assimp::Exporter exporter;
		aiReturn ret = exporter.Export(outScene, "fbx", directory + "/" + name + ".fbx");

		if (ret == AI_SUCCESS)
		{
			std::cout << "Export succeeded!\n";
		}
		else
		{
			std::cerr << "Export failed: " << exporter.GetErrorString() << "\n";
			return false;
		}

		return true;
	}

	aiMaterial* Mesh::deepCopyMaterial(const aiMaterial* src)
	{
		if (!src) return new aiMaterial();

		aiMaterial* dest = new aiMaterial();

		for (unsigned int i = 0; i < src->mNumProperties; ++i)
		{
			const aiMaterialProperty* sprop = src->mProperties[i];
			if (!sprop || !sprop->mData || sprop->mDataLength == 0)
				continue;

			dest->AddBinaryProperty(
				sprop->mData,
				sprop->mDataLength,
				sprop->mKey.C_Str(),
				sprop->mSemantic,
				sprop->mIndex,
				static_cast<aiPropertyTypeInfo>(sprop->mType)
			);
		}

		return dest;
	}
}