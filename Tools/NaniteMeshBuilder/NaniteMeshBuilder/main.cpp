#include <iostream>
#include <fstream>

#include "MeshProcessing/Simplifying/MeshSimplifier.h"
#include "Utils/Utils.h"

int main(void)
{
	std::cout << "Nanite Mesh Builder is Running!" << std::endl;

	//std::string modelPath = "../../../Resources/Sphere.obj";
	//std::string modelPath = "../../../Resources/SphereH.obj";
	//std::string modelPath = "../../../Resources/Plane.obj";
	//std::string modelPath = "../../../Resources/Dragon_8K.obj";
	//std::string modelPath = "../../../Resources/Dragon_80K.obj";
	std::string modelPath = "../../../Resources/boguchi.glb";

	std::string modelName = nanite::utils::ExtractFileName(modelPath);
	std::string outputPath = "../../../Nanite/Assets/Resources/QEM/" + modelName;

	nanite::Mesh mesh;
	mesh.LoadFromFile(modelPath);
	mesh.SaveToFile(outputPath, modelName + "_" + std::to_string(0), ".obj");

	for (int i = 0; i < 5; ++i)
	{
		mesh = nanite::SimplifyMesh(mesh, mesh.NumTriangles() / 2);
		mesh.SaveToFile(outputPath, modelName + "_" + std::to_string(i + 1), ".obj");
	}

	return 0;
}