#include <iostream>
#include <fstream>

#include "MeshProcessing/Simplifying/MeshSimplifier.h"
#include "MeshProcessing/Clustering/MeshClustering.h"
#include "Utils/Utils.h"
#include "Math/Math.h"

int main(void)
{
	std::cout << "Nanite Mesh Builder is Running!" << std::endl;

	//std::string modelPath = "../../../Resources/Sphere.obj";
	//std::string modelPath = "../../../Resources/SphereH.obj";
	std::string modelPath = "../../../Resources/Plane.obj";
	//std::string modelPath = "../../../Resources/Dragon_8K.obj";
	//std::string modelPath = "../../../Resources/Dragon_80K.obj";
	//std::string modelPath = "../../../Resources/boguchi.glb";

	std::string modelName = nanite::utils::ExtractFileName(modelPath);
	std::string outputPath = "../../../Nanite/Assets/Resources/QEM/" + modelName;

	nanite::Mesh mesh;
	mesh.LoadFromFile(modelPath);
	mesh.SaveToFile(outputPath, modelName + "_" + std::to_string(0), ".obj");

	//// clustering
	//const int numParts = 4;
	//std::vector<int> parts = nanite::PartMesh(mesh, numParts);

	//// reorder triangles
	//std::vector<std::vector<int>> reorederBuffer(numParts);
	//for (int triIdx = 0; triIdx < mesh.NumTriangles(); ++triIdx)
	//{
	//	reorederBuffer[parts[triIdx]].emplace_back(triIdx);
	//}

	//std::vector<uint32_t> reorderedIndices;
	//reorderedIndices.reserve(mesh.Indices.size());
	//for (const std::vector<int> tris : reorederBuffer)
	//{
	//	for (int triIdx : tris)
	//	{
	//		auto [i0, i1, i2] = mesh.GetTriangleIndices(triIdx);
	//		reorderedIndices.push_back(i0);
	//		reorderedIndices.push_back(i1);
	//		reorderedIndices.push_back(i2);
	//	}
	//}
	//mesh.Indices = std::move(reorderedIndices);

	//struct Cluster
	//{
	//	nanite::AABB Bounds;
	//	int StartIndex = 0; // start index in the triangle list
	//	int NumTriangles = 0; // number of triangles in this cluster
	//};
	//std::vector<Cluster> clusters;
	//clusters.resize(numParts);

	//int indexOffset = 0;
	//for (int i = 0; i < numParts; ++i)
	//{
	//	Cluster& cluster = clusters[i];
	//	cluster.StartIndex = indexOffset;
	//	cluster.NumTriangles = static_cast<int>(reorederBuffer[i].size());
	//	for (int j = cluster.StartIndex; j < cluster.StartIndex + cluster.NumTriangles; ++j)
	//	{
	//		auto [v0, v1, v2] = mesh.GetTriangleVertices(j);
	//		cluster.Bounds.Encapsulate(v0);
	//		cluster.Bounds.Encapsulate(v1);
	//		cluster.Bounds.Encapsulate(v2);
	//	}
	//	indexOffset += cluster.NumTriangles;
	//}

	//for (int i = 0; i < clusters.size(); ++i)
	//{
	//	const Cluster& cluster = clusters[i];
	//	nanite::FVector3 color = nanite::utils::HSVtoRGB(std::fmod(i / 6.f, 1.f), 1.f, 1.f);
	//	for (int j = cluster.StartIndex; j < cluster.StartIndex + cluster.NumTriangles; ++j)
	//	{
	//		mesh.Colors[j] = color;
	//	}
	//}
	//mesh.ComputeNormals();
	//mesh.SaveToFile(outputPath, modelName + "_clu" ".obj");

	//// save metadata; bounding box info
	//std::ofstream file(outputPath + "/" + modelName + "_clu" + "_metadata.txt");
	//if (!file.is_open()) return false;
	//for (int i = 0; i < clusters.size(); ++i)
	//{
	//	const Cluster& cluster = clusters[i];
	//	const nanite::AABB& aabb = cluster.Bounds;
	//	nanite::FVector3 color = nanite::utils::HSVtoRGB(std::fmod(i / 8.f, 1.f), 1.f, 1.f);
	//	file << aabb.Min.x << " " << aabb.Min.y << " " << aabb.Min.z << " "
	//		<< aabb.Max.x << " " << aabb.Max.y << " " << aabb.Max.z << " "
	//		<< color.x << " " << color.y << " " << color.z << "\n";
	//}
	//file.close();


	// simplifying
	for (int i = 0; i < 5; ++i)
	{
		mesh = nanite::SimplifyMesh(mesh, mesh.NumTriangles() / 2);
		mesh.SaveToFile(outputPath, modelName + "_" + std::to_string(i + 1), ".obj");
	}

	return 0;
}