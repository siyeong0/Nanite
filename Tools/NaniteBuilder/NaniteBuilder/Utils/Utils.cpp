#include "Utils.h"

#include <iostream>
#include <fstream>
#include <set>
#include <vector>
#include <array>

#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace nanite
{
	std::string ExtractFileName(const std::string& path)
	{
		size_t lastSlash = path.find_last_of("/\\");
		size_t lastDot = path.find_last_of('.');
		if (lastSlash != std::string::npos && lastDot != std::string::npos)
		{
			return path.substr(lastSlash + 1, lastDot - lastSlash - 1);
		}
		return "";
	}

	std::string ExtractExtension(const std::string& path)
	{
		size_t lastDot = path.find_last_of('.');
		if (lastDot != std::string::npos)
		{
			return path.substr(lastDot + 1);
		}
		return "";
	}

	void MergeVertices(
		const std::vector<FVector3>& inVertices,
		const std::vector<uint32_t>& inIndices,
		std::vector<FVector3>* outVertices,
		std::vector<uint32_t>* outIndices)
	{
		std::unordered_map<FVector3, uint32_t, FVector3Hasher> uniqueVertexMap;
		outVertices->clear();
		outIndices->resize(inIndices.size());

		for (size_t i = 0; i < inIndices.size(); ++i)
		{
			const FVector3& pos = inVertices[inIndices[i]];
			auto it = uniqueVertexMap.find(pos);
			if (it != uniqueVertexMap.end())
			{
				(*outIndices)[i] = it->second;
			}
			else
			{
				uint32_t newIndex = static_cast<uint32_t>(outVertices->size());
				outVertices->emplace_back(pos);
				uniqueVertexMap[pos] = newIndex;
				(*outIndices)[i] = newIndex;
			}
		}
	}

	AABB ComputeBoundingBox(const std::vector<FVector3>& vertices)
	{
		AABB aabb;
		for (const FVector3& v : vertices)
		{
			aabb.Encapsulate(v);
		}
		return aabb;
	}

	bool LoadMeshFromFile(const std::string& path, Mesh* outMesh)
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

		std::vector<FVector3> normals;
		normals.reserve(numFaces);
		if (hasNormals)
		{
			for (int i = 0; i < numFaces; ++i)
			{
				aiVector3D n0 = mesh->mNormals[indices[3 * i + 0]];
				aiVector3D n1 = mesh->mNormals[indices[3 * i + 1]];
				aiVector3D n2 = mesh->mNormals[indices[3 * i + 2]];
				FVector3 faceNormal(
					(n0.x + n1.x + n2.x) / 3.0f,
					(n0.y + n1.y + n2.y) / 3.0f,
					(n0.z + n1.z + n2.z) / 3.0f
				);
				normals.emplace_back(faceNormal);
			}
		}
		else
		{
			for (int i = 0; i < numFaces; ++i)
			{
				aiVector3D av0 = mesh->mVertices[indices[3 * i + 0]];
				aiVector3D av1 = mesh->mVertices[indices[3 * i + 1]];
				aiVector3D av2 = mesh->mVertices[indices[3 * i + 2]];
				FVector3 v0 = FVector3(av0.x, av0.y, av0.z);
				FVector3 v1 = FVector3(av1.x, av1.y, av1.z);
				FVector3 v2 = FVector3(av2.x, av2.y, av2.z);
				FVector3 faceNormal = (v1 - v0).Cross(v2 - v0).Norm();
				normals.emplace_back(faceNormal);
			}
		}

		std::vector<FVector3> mergedVertices;
		std::vector<uint32_t> mergedIndices;
		MergeVertices(vertices, indices, &mergedVertices, &mergedIndices);

		outMesh->Vertices = std::move(mergedVertices);

		outMesh->Triangles.reserve(numFaces);
		for (int i = 0; i < numFaces; ++i)
		{
			Triangle triangle;
			triangle.i0 = mergedIndices[3 * i + 0];
			triangle.i1 = mergedIndices[3 * i + 1];
			triangle.i2 = mergedIndices[3 * i + 2];
			triangle.Normal = normals[i];
			triangle.Color = FVector3(1.0f, 1.0f, 1.0f); // Default color TODO: implement color support
			outMesh->Triangles.emplace_back(triangle);
		}

		return true;
	}

	bool SaveMeshToFbx(const Mesh& mesh, const std::string& path, const std::string& name)
	{
		std::vector<FVector3> outVertices;
		std::vector<uint32_t> outIndices;
		std::vector<FVector3> outNormals;
		std::vector<FVector3> outColors;

		for (int i = 0; i < mesh.Triangles.size(); ++i)
		{
			const Triangle& triangle = mesh.Triangles[i];
			outVertices.push_back(mesh.Vertices[triangle.i0]);
			outVertices.push_back(mesh.Vertices[triangle.i1]);
			outVertices.push_back(mesh.Vertices[triangle.i2]);
			outIndices.push_back(3 * i + 0);
			outIndices.push_back(3 * i + 1);
			outIndices.push_back(3 * i + 2);
			outNormals.push_back(triangle.Normal);
			outNormals.push_back(triangle.Normal);
			outNormals.push_back(triangle.Normal);
			outColors.push_back(triangle.Color);
			outColors.push_back(triangle.Color);
			outColors.push_back(triangle.Color);
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
		aimesh->mNumFaces = static_cast<unsigned int>(mesh.Triangles.size());
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

		Assimp::Exporter exporter;
		aiReturn ret = exporter.Export(outScene, "fbx", path + name + ".fbx");

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

	bool SaveClustersMetadata(const std::vector<Cluster> clusters, const std::string& path, const std::string& name)
	{
		// save metadata; bounding box info
		std::ofstream file(path + name + "_metadata.txt");
		if (!file.is_open()) return false;
		for (int i = 0; i < clusters.size(); ++i)
		{
			const Cluster& cluster = clusters[i];
			const AABB& aabb = cluster.Bounds;
			FVector3 color = HSVtoRGB(std::fmod(i / 8.f, 1.f), 1.f, 1.f);
			file << aabb.Min.x << " " << aabb.Min.y << " " << aabb.Min.z << " "
				<< aabb.Max.x << " " << aabb.Max.y << " " << aabb.Max.z << " "
				<< color.x << " " << color.y << " " << color.z << "\n";
		}
		file.close();

		return true;
	}

	FVector3 HSVtoRGB(float h, float s, float v)
	{
		float c = v * s;
		float x = c * (1.f - abs(fmod(h * 6.f, 2.f) - 1.f));
		float m = v - c;

		FVector3 rgb;

		if (h < 1.0 / 6.0)
			rgb = FVector3(c, x, 0);
		else if (h < 2.0 / 6.0)
			rgb = FVector3(x, c, 0);
		else if (h < 3.0 / 6.0)
			rgb = FVector3(0, c, x);
		else if (h < 4.0 / 6.0)
			rgb = FVector3(0, x, c);
		else if (h < 5.0 / 6.0)
			rgb = FVector3(x, 0, c);
		else
			rgb = FVector3(c, 0, x);

		return rgb + FVector3{ m, m, m };
	}

	using namespace nanite;

	void CheckDuplicateTriangles(const std::vector<Triangle>& triangles)
	{
		std::set<std::array<uint32_t, 3>> triangleSet;
		bool hasDuplicate = false;

		for (size_t i = 0; i < triangles.size(); ++i)
		{
			const Triangle& tri = triangles[i];

			// 정점 인덱스를 정렬해서 방향과 무관한 중복도 걸러냄
			std::array<uint32_t, 3> sortedIndices = { tri.i0, tri.i1, tri.i2 };
			std::sort(sortedIndices.begin(), sortedIndices.end());

			if (!triangleSet.insert(sortedIndices).second)
			{
				std::cout << "Duplicate triangle found at index " << i
					<< " with vertices (" << tri.i0 << ", " << tri.i1 << ", " << tri.i2 << ")\n";
				//assert(false);
				hasDuplicate = true;
			}
		}

		if (!hasDuplicate)
		{
			std::cout << "No duplicate triangles found.\n";
		}
	}
}