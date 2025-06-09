#include "Mesh.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <regex>

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

		std::vector<FVector3> vertices;
		vertices.reserve(numVertices);
		for (int i = 0; i < numVertices; ++i)
		{
			aiVector3D vertex = mesh->mVertices[i];
			vertices.emplace_back(vertex.x, vertex.y, vertex.z);
		}

		std::vector<uint32_t> indices;
		indices.reserve(numIndices);
		for (int i = 0; i < numFaces; ++i)
		{
			aiFace face = mesh->mFaces[i];
			indices.emplace_back(face.mIndices[0]);
			indices.emplace_back(face.mIndices[1]);
			indices.emplace_back(face.mIndices[2]);
		}

		std::vector<FVector3> mergedVertices;
		std::vector<uint32_t> mergedIndices;
		utils::MergeDuplicatedVertices(vertices, indices, &mergedVertices, &mergedIndices);

		Vertices = std::move(mergedVertices);
		Indices = std::move(mergedIndices);
		ComputeNormals();
		Colors.resize(NumTriangles(), {1.f, 1.f, 1.f});

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