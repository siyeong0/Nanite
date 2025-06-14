#include "Mesh.h"

#include <iostream>
#include <algorithm>
#include <array>
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
	std::ostream* Mesh::msOutStream = nullptr; // Global output stream for debug messages

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

	void Mesh::MergeDuplicatedVertices()
	{
		int prevNumIndicesWithUsedOnce = -1;
		while (true)
		{
			std::unordered_set<Edge> edges;
			std::unordered_map<Edge, int> edgeUsage;
			edges.reserve(utils::NextPrime(NumTriangles() * 3));
			edgeUsage.reserve(utils::NextPrime(NumTriangles() * 3));
			for (int triIdx = 0; triIdx < NumTriangles(); ++triIdx)
			{
				auto [e0, e1, e2] = GetTriangleEdges(triIdx);
				edges.emplace(e0);
				edges.emplace(e1);
				edges.emplace(e2);
				edgeUsage[e0]++;
				edgeUsage[e1]++;
				edgeUsage[e2]++;
			}

			std::set<uint32_t> indicesWithEdgeUsedOnceTmp;
			std::set<uint32_t> indicesWithEdgeUsedTwiceTmp;
			for (const auto& [edge, count] : edgeUsage)
			{
				assert(count == 1 || count == 2);
				if (count == 1)
				{
					indicesWithEdgeUsedOnceTmp.emplace(edge.GetA());
					indicesWithEdgeUsedOnceTmp.emplace(edge.GetB());
				}
				else if (count == 2)
				{
					indicesWithEdgeUsedTwiceTmp.emplace(edge.GetA());
					indicesWithEdgeUsedTwiceTmp.emplace(edge.GetB());
				}
			}

			std::set<uint32_t> indicesWithEdgeUsedOnce;
			std::set<uint32_t> indicesWithEdgeUsedTwice;
			std::set_difference(
				indicesWithEdgeUsedTwiceTmp.begin(), indicesWithEdgeUsedTwiceTmp.end(),
				indicesWithEdgeUsedOnceTmp.begin(), indicesWithEdgeUsedOnceTmp.end(),
				std::inserter(indicesWithEdgeUsedTwice, indicesWithEdgeUsedTwice.end()));
			indicesWithEdgeUsedOnce = std::move(indicesWithEdgeUsedOnceTmp);
			assert(indicesWithEdgeUsedOnce.size() + indicesWithEdgeUsedTwice.size() == NumVertices());

			int numIndicesWithEdgeUsedOnce = static_cast<int>(indicesWithEdgeUsedOnce.size());
			if (numIndicesWithEdgeUsedOnce == 0 ||
				numIndicesWithEdgeUsedOnce == prevNumIndicesWithUsedOnce)
			{
				break; // No more merging possible
			}
			prevNumIndicesWithUsedOnce = numIndicesWithEdgeUsedOnce;

			if (msOutStream != nullptr)
			{
				std::unordered_map<int, int> countMap;
				for (const auto& [edge, count] : edgeUsage)
				{
					countMap[count]++;
				}
				assert(countMap[1] + countMap[2] == edgeUsage.size());

				std::ostream& out = *msOutStream;
				out << "Number of Edges used once : " << countMap[1] << std::endl;
				out << "Number of Edges used once : " << countMap[2] << std::endl;
				out << std::endl;
			}

			const float MERGE_THRESHOLD = 0.001f; // Distance threshold for merging vertices
			std::vector<uint32_t> indicesWithEdgeUsedOnceBuf(indicesWithEdgeUsedOnce.begin(), indicesWithEdgeUsedOnce.end());
			std::unordered_map<uint32_t, uint32_t> onceBufIndexMap;
			for (int i = 0; i < indicesWithEdgeUsedOnceBuf.size(); ++i)
			{
				uint32_t index = indicesWithEdgeUsedOnceBuf[i];
				if (onceBufIndexMap.find(index) != onceBufIndexMap.end()) continue; // Already has pair

				const FVector3& vertex = Vertices[index];
				float minDistance = std::numeric_limits<float>::max();
				int closestIndexOffset = -1;
				for (int j = i + 1; j < indicesWithEdgeUsedOnceBuf.size(); ++j)
				{
					uint32_t otherIndex = indicesWithEdgeUsedOnceBuf[j];
					if (onceBufIndexMap.find(otherIndex) != onceBufIndexMap.end()) continue; // Already has pair

					const FVector3& otherVertex = Vertices[otherIndex];
					float distance = FVector3::Distance(vertex, otherVertex);
					if (distance < minDistance && edges.find(Edge(index, otherIndex)) == edges.end())
					{
						minDistance = distance;
						closestIndexOffset = j;
					}
				}
				if (minDistance < MERGE_THRESHOLD && closestIndexOffset != -1)
				{
					onceBufIndexMap[indicesWithEdgeUsedOnceBuf[closestIndexOffset]] = index;
				}
			}

			std::vector<FVector3> mergedVertices;
			mergedVertices.reserve(Vertices.size());

			std::unordered_map<uint32_t, uint32_t> indexMap;
			for (uint32_t index : indicesWithEdgeUsedOnce)
			{
				const FVector3& pos = Vertices[index];
				if (onceBufIndexMap.find(index) == onceBufIndexMap.end())
				{
					mergedVertices.push_back(pos);
					indexMap[index] = static_cast<uint32_t>(mergedVertices.size() - 1);
				}
				else
				{
					indexMap[index] = indexMap[onceBufIndexMap[index]];
				}
			}
			for (uint32_t index : indicesWithEdgeUsedTwice)
			{
				const FVector3& pos = Vertices[index];
				mergedVertices.push_back(pos);
				indexMap[index] = static_cast<uint32_t>(mergedVertices.size() - 1);
			}

			std::vector<uint32_t> mergedIndices;
			mergedIndices.resize(Indices.size());
			for (uint32_t i = 0; i < Indices.size(); ++i)
			{
				assert(indexMap.find(Indices[i]) != indexMap.end() && "Index not found in index map during merging vertices.");
				mergedIndices[i] = indexMap[Indices[i]];
			}

			Vertices = std::move(mergedVertices);
			Indices = std::move(mergedIndices);
		}
		
		struct UniqueTriangle
		{
			std::array<uint32_t, 3> Indices;
			std::array<uint32_t, 3> SortedIndices;
			int TriangleIndex = -1;

			UniqueTriangle(uint32_t i0, uint32_t i1, uint32_t i2, int triIdx)
				: Indices({ i0, i1, i2 })
				, SortedIndices({ i0, i1, i2 })
				, TriangleIndex(triIdx)
			{
				std::sort(SortedIndices.begin(), SortedIndices.end());
			}

			bool operator<(const UniqueTriangle& other) const
			{
				return SortedIndices < other.SortedIndices;
			}
		};

		{
			std::unordered_map<Edge, int> edgeUsage;
			for (int triIdx = 0; triIdx < NumTriangles(); ++triIdx)
			{
				auto [e0, e1, e2] = GetTriangleEdges(triIdx);
				edgeUsage[e0]++;
				edgeUsage[e1]++;
				edgeUsage[e2]++;
			}
			for (const auto& [edge, count] : edgeUsage)
			{
				if (count == 1)
				{
					std::cout << "Edge used once:" << "(" << edge.GetA() << "," << edge.GetB() << ")\n";
				}
			}
			std::cout << std::endl;
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
		const aiScene* scene = importer.ReadFile(path,
			aiProcess_Triangulate |
			aiProcess_DropNormals |
			aiProcess_JoinIdenticalVertices);

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

		MergeDuplicatedVertices();

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
		if (extension[0] != '.')
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
		aiReturn ret = exporter.Export(outScene, extension.substr(1), directory + "/" + name + extension);

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

	bool Mesh::SaveToFileDbg(const std::string& directory, const std::string& name, const std::string& format)
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

		std::vector<FVector3> outVertices;
		std::vector<uint32_t> outIndices;
		std::vector<FVector3> outNormals;
		std::vector<FVector3> outColors;

		for (int i = 0; i < NumTriangles(); ++i)
		{
			auto [v0, v1, v2] = GetTriangleVertices(i);
			FVector3 n = Normals[i];
			FVector3 c = Colors[i];

			outVertices.push_back(v0);
			outVertices.push_back(v1);
			outVertices.push_back(v2);
			outIndices.push_back(3 * i + 0);
			outIndices.push_back(3 * i + 1);
			outIndices.push_back(3 * i + 2);
			outNormals.push_back(n);
			outNormals.push_back(n);
			outNormals.push_back(n);
			outColors.push_back(c);
			outColors.push_back(c);
			outColors.push_back(c);
		}

		aiScene* outScene = new aiScene();
		outScene->mRootNode = new aiNode();

		outScene->mMaterials = new aiMaterial * [1];
		outScene->mMaterials[0] = new aiMaterial();
		outScene->mNumMaterials = 1;

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
		aiReturn ret = exporter.Export(outScene, extension.substr(1), directory + "/" + name + extension);

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