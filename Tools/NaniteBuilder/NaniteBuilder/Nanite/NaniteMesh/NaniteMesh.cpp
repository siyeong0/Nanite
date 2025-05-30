#include "NaniteMesh.h"

#include <iostream>

#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "../../Utils/Utils.h"	
#include "../Builder/NaniteBuilder.h"

namespace nanite
{
	bool NaniteMesh::Build(int leafTriThreshold)
	{
		int nparts = static_cast<int>(std::ceilf(NumTriangles() / (float)leafTriThreshold));
		if (nparts < 1) return true;

		std::vector<int32_t> triParts;
		int edgeCut = NaniteBuilder::BuildGraph(*this, nparts, triParts);

		std::cout << "METIS partitioning succeeded. Edge cut = " << edgeCut << "\n";

		std::vector<int> numElementsPerPart(nparts, 0);
		for (int i = 0; i < NumTriangles(); ++i)
		{
			int part = triParts[i];
			numElementsPerPart[part]++;
		}

		std::cout << "Number of triangles in each partition:\n";
		for (int i = 0; i < nparts; ++i)
		{
			std::cout << "Partition " << i << ": " << numElementsPerPart[i] << " triangles\n";
		}

		// set vertex color from partition
		for (int i = 0; i < NumTriangles(); ++i)
		{
			int part = triParts[i];
			mTriangles[i].Color = HSVtoRGB(std::fmod(part / 6.f, 1.f), 1.f, 1.f);
		}

		return true;
	}

	bool NaniteMesh::LoadFromFile(const std::string& path)
	{
		Assimp::Importer importer;
		mScene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
		if (!mScene || !mScene->HasMeshes())
		{
			std::cerr << "Failed to load model\n";
			return false;
		}
		if (mName.empty())
		{
			// Extract the name from the file path
			size_t lastSlash = path.find_last_of("/\\");
			if (lastSlash != std::string::npos)
				mName = path.substr(lastSlash + 1);
		}

		const aiMesh* mesh = mScene->mMeshes[0];
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
		mergeVertices(vertices, indices, mergedVertices, mergedIndices);

		mVertices = std::move(mergedVertices);

		mTriangles.reserve(numFaces);
		for (int i = 0; i < numFaces; ++i)
		{
			Triangle triangle;
			triangle.i0 = mergedIndices[3 * i + 0];
			triangle.i1 = mergedIndices[3 * i + 1];
			triangle.i2 = mergedIndices[3 * i + 2];
			triangle.Normal = normals[i];
			triangle.Color = FVector3(1.0f, 1.0f, 1.0f); // Default color TODO: implement color support
			mTriangles.emplace_back(triangle);
		}

		return true;
	}

	bool NaniteMesh::SaveToFile(const std::string& path) const
	{
		aiScene* outScene = new aiScene();
		outScene->mRootNode = new aiNode();

		outScene->mMaterials = new aiMaterial * [1];
		outScene->mMaterials[0] = mScene->mMaterials ? mScene->mMaterials[0] : new aiMaterial();
		outScene->mNumMaterials = 1;

		aiMesh* mesh = new aiMesh();
		mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
		mesh->mNumVertices = static_cast<unsigned int>(mVertices.size());
		mesh->mVertices = new aiVector3D[mesh->mNumVertices];
		mesh->mNormals = new aiVector3D[mesh->mNumVertices];
		mesh->mColors[0] = new aiColor4D[mesh->mNumVertices];
		mesh->mMaterialIndex = 0;

		for (int i = 0; i < static_cast<int>(mesh->mNumVertices); ++i) 
		{
			mesh->mVertices[i] = aiVector3D(mVertices[i].x, mVertices[i].y, mVertices[i].z);
		}

		mesh->mNumFaces = static_cast<unsigned int>(mTriangles.size());
		mesh->mFaces = new aiFace[mesh->mNumFaces];
		for (int i = 0; i < static_cast<int>(mesh->mNumFaces); ++i)
		{
			const Triangle& triangle = mTriangles[i];

			mesh->mFaces[i].mNumIndices = 3;
			mesh->mFaces[i].mIndices = new unsigned int[3];
			mesh->mFaces[i].mIndices[0] = triangle.i0;
			mesh->mFaces[i].mIndices[1] = triangle.i1;
			mesh->mFaces[i].mIndices[2] = triangle.i2;

			aiVector3D normal = aiVector3D(triangle.Normal.x, triangle.Normal.y, triangle.Normal.z);
			mesh->mNormals[triangle.i0] = normal;
			mesh->mNormals[triangle.i1] = normal;
			mesh->mNormals[triangle.i2] = normal;

			// Default color TODO: implement color support
			aiColor4D color = aiColor4D(triangle.Color.x, triangle.Color.y, triangle.Color.z, 1.0f);
			mesh->mColors[0][triangle.i0] = color;
			mesh->mColors[0][triangle.i1] = color;
			mesh->mColors[0][triangle.i2] = color;
		}

		outScene->mNumMeshes = 1;
		outScene->mMeshes = new aiMesh * [1];
		outScene->mMeshes[0] = mesh;

		outScene->mRootNode->mNumMeshes = 1;
		outScene->mRootNode->mMeshes = new unsigned int[1];
		outScene->mRootNode->mMeshes[0] = 0;

		Assimp::Exporter exporter;
		aiReturn ret = exporter.Export(outScene, "fbx", path + mName + ".fbx");

		if (ret == AI_SUCCESS)
		{
			std::cout << "Export succeeded!\n";
			return true;
		}
		else
		{
			std::cerr << "Export failed: " << exporter.GetErrorString() << "\n";
			return false;
		}
	}

	void NaniteMesh::mergeVertices(
		const std::vector<FVector3>& inVertices,
		const std::vector<uint32_t>& inIndices,
		std::vector<FVector3>& outVertices,
		std::vector<uint32_t>& outIndices)
	{
		struct FVector3Hasher
		{
			size_t operator()(const FVector3& v) const
			{
				const int scale = 100000;
				size_t hx = std::hash<int>()(static_cast<int>(v.x * scale));
				size_t hy = std::hash<int>()(static_cast<int>(v.y * scale));
				size_t hz = std::hash<int>()(static_cast<int>(v.z * scale));
				return hx ^ (hy << 1) ^ (hz << 2);
			}
		};

		std::unordered_map<FVector3, uint32_t, FVector3Hasher> uniqueVertexMap;
		outVertices.clear();
		outIndices.resize(inIndices.size());

		for (size_t i = 0; i < inIndices.size(); ++i)
		{
			const FVector3& pos = inVertices[inIndices[i]];
			auto it = uniqueVertexMap.find(pos);
			if (it != uniqueVertexMap.end())
			{
				outIndices[i] = it->second;
			}
			else
			{
				uint32_t newIndex = static_cast<uint32_t>(outVertices.size());
				outVertices.emplace_back(pos);
				uniqueVertexMap[pos] = newIndex;
				outIndices[i] = newIndex;
			}
		}
	}
}