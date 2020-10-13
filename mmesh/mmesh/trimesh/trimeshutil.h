#ifndef MMESH_TRIMESHUTIL_1602590289222_H
#define MMESH_TRIMESHUTIL_1602590289222_H
#include <vector>

namespace trimesh
{
	class TriMesh;
}

namespace mmesh
{
	void mergeTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes);

	void reverseTriMesh(trimesh::TriMesh* Mesh);

	void mergeTrianglesTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes); // inMeshes is triangles

	trimesh::TriMesh* partMesh(const std::vector<int>& indices, trimesh::TriMesh* inMesh);

	void dumplicateMesh(trimesh::TriMesh* mesh);
}

#endif // MMESH_TRIMESHUTIL_1602590289222_H