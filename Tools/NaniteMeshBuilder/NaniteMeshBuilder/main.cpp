#include <iostream>
#include <fstream>

#include "MeshProcessing/MeshSimplifier.h"
#include "MeshProcessing/MeshClustering.h"
#include "Utils/Utils.h"
#include "Math/Math.h"
#include "NaniteMesh\NaniteMesh.h"

int main(void)
{
	std::cout << "Nanite Mesh Builder is Running!" << std::endl;

	std::string modelPaths[] = 
	{
		//"../../../Resources/Sphere.obj",
		//"../../../Resources/SphereH.obj",
		//"../../../Resources/Plane.obj",
		"../../../Resources/Dragon_8K.obj",
		//"../../../Resources/Dragon_80K.obj",
		//"../../../Resources/boguchi.glb"
	};

	const int TARGET_LEAF_POLGON_COUNT = 128;
	for (const std::string& modelPath : modelPaths)
	{
		std::string modelName = nanite::utils::ExtractFileName(modelPath);
		std::string outputPath = "../../../Nanite/Assets/Resources/LOD/" + modelName;

		std::cout << "\n\nProcessing model: " << modelName << std::endl;

		nanite::Mesh mesh;
		mesh.LoadFromFile(modelPath);

		nanite::NaniteMesh naniteMesh;
		naniteMesh.Build(mesh, TARGET_LEAF_POLGON_COUNT);

		naniteMesh.GetLODMesh(0).SaveToFileDbg(outputPath, modelName + "_nanite_LOD0", ".fbx");
		naniteMesh.GetLODMesh(1).SaveToFileDbg(outputPath, modelName + "_nanite_LOD1", ".fbx");
	}
	return 0;
	for (const std::string& modelPath : modelPaths)
	{
		std::string modelName = nanite::utils::ExtractFileName(modelPath);
		std::string outputPath = "../../../Nanite/Assets/Resources/QEM/" + modelName;

		std::cout << "\n\nProcessing model: " << modelName << std::endl;

		nanite::Mesh mesh;
		mesh.LoadFromFile(modelPath);

		std::vector<nanite::Cluster> clusters = nanite::ClusterMesh(mesh, TARGET_LEAF_POLGON_COUNT);
		nanite::utils::PaintMeshByCluster(&mesh, clusters);

		mesh.SaveToFileDbg(outputPath, modelName + "_" + std::to_string(0), ".fbx");

		for (int i = 0; i < 5; ++i)
		{
			// Simplifying
			mesh = nanite::SimplifyMesh(mesh, mesh.NumTriangles() / 2);

			// Clustering
			clusters = nanite::ClusterMesh(mesh, TARGET_LEAF_POLGON_COUNT);
			nanite::utils::PaintMeshByCluster(&mesh, clusters);

			// Save mesh
			mesh.SaveToFileDbg(outputPath, modelName + "_" + std::to_string(i + 1), ".fbx");

			// Save metadata; bounding box info
			std::ofstream file(outputPath + "/" + modelName + "_clu" + "_metadata.txt");
			if (!file.is_open()) return false;
			for (int j = 0; j < clusters.size(); ++j)
			{
				const nanite::Cluster& cluster = clusters[j];
				const nanite::AABB& aabb = cluster.Bounds;
				nanite::FVector3 color = nanite::utils::HSVtoRGB(std::fmod(j / 8.f, 1.f), 1.f, 1.f);
				file << aabb.Min.x << " " << aabb.Min.y << " " << aabb.Min.z << " "
					<< aabb.Max.x << " " << aabb.Max.y << " " << aabb.Max.z << " "
					<< color.x << " " << color.y << " " << color.z << "\n";
			}
			file.close();
		}
	}
	return 0;
}