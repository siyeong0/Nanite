#include <iostream>

#include "Nanite/NaniteMesh/NaniteMesh.h"

int main(void)
{
	std::cout << "NaniteBuilder is running!" << std::endl;

	const char* modelFile = "../../../Resources/Dragon_8K.obj";
	//const char* modelFile = "../../../Resources/Sphere.obj";
	const char* outputPath = "../../../Nanite/Assets/Resources/";

	nanite::NaniteMesh mesh;
	mesh.LoadFromFile(modelFile);
	mesh.Build(128);
	mesh.SaveToFbx(outputPath);

	return 0;
}