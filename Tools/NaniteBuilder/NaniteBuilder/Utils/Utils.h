#pragma once
#include <string>
#include <vector>

#include "../Math/FVector3.hpp"
#include "../Topology/Cluster.h"
#include "../Topology/Mesh.h"

namespace nanite
{
	std::string ExtractFileName(const std::string& path);
	std::string ExtractExtension(const std::string& path);

	struct FVector3Hasher
	{
		size_t operator()(const FVector3& v) const
		{
			const int scale = 1000;
			size_t hx = std::hash<int>()(static_cast<int>(v.x * scale));
			size_t hy = std::hash<int>()(static_cast<int>(v.y * scale));
			size_t hz = std::hash<int>()(static_cast<int>(v.z * scale));
			return hx ^ (hy << 1) ^ (hz << 2);
		}
	};

	void MergeVertices(
		const std::vector<FVector3>& inVertices,
		const std::vector<uint32_t>& inIndices,
		std::vector<FVector3>* outVertices,
		std::vector<uint32_t>* outIndices);

	AABB ComputeBoundingBox(const std::vector<FVector3>& vertices);
	inline float ComputeArea(const FVector3& a, const FVector3& b, const FVector3& c) { return 0.5f * (b - a).Cross(c - a).Length(); }

	bool LoadMeshFromFile(const std::string& path, Mesh* outMesh);
	bool SaveMeshToFbx(const Mesh& mesh, const std::string& path, const std::string& name);

	bool SaveClustersMetadata(const std::vector<Cluster> clusters, const std::string& path, const std::string& name);

	FVector3 HSVtoRGB(float h, float s, float v);

	void CheckDuplicateTriangles(const std::vector<Triangle>& triangles);
}