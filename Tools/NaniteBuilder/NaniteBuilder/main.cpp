#include <iostream>

#include "Nanite/NaniteMesh.h"
#include "Utils/Utils.h"

int main(void)
{
	std::cout << "NaniteBuilder is running!" << std::endl;

	//const char* modelFile = "../../../Resources/Sphere.obj";
	//const char* outputPath = "../../../Nanite/Assets/Resources/Sphere/";
	const char* modelFile = "../../../Resources/Dragon_8K.obj";
	const char* outputPath = "../../../Nanite/Assets/Resources/Dragon_8K/";
	//const char* modelFile = "../../../Resources/Dragon_80K.obj";
	//const char* outputPath = "../../../Nanite/Assets/Resources/Dragon_80K/";

	nanite::Mesh mesh;
	nanite::LoadMeshFromFile(modelFile, &mesh);
	
	nanite::NaniteMesh naniteMesh(nanite::ExtractFileName(modelFile));
	naniteMesh.Build(mesh, 128);
	naniteMesh.PaintColorByCluster();

	for (int i = 0; i < naniteMesh.GetDepth(); ++i)
	{
		const nanite::Mesh& nnmesh = naniteMesh.GetMesh(i);
		const std::vector<nanite::Cluster>& clusters = naniteMesh.GetClusters(i);
		nanite::SaveMeshToFbx(nnmesh, outputPath, naniteMesh.GetName() + "_L" + std::to_string(i));
		nanite::SaveClustersMetadata(clusters, outputPath, naniteMesh.GetName() + "_L" + std::to_string(i));
	}

	return 0;
}