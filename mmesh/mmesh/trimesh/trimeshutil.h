#ifndef MMESH_TRIMESHUTIL_1602590289222_H
#define MMESH_TRIMESHUTIL_1602590289222_H
#include <vector>
#include "trimesh2/XForm.h"
#include "trimesh2/Box.h"

namespace trimesh
{
	class TriMesh;
}

namespace mmesh
{
	void mergeTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes, bool fanzhuan = false);
	void mergeTriMesh_omp(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes);

	void reverseTriMesh(trimesh::TriMesh* Mesh);

	void mergeTrianglesTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes); // inMeshes is triangles

	trimesh::TriMesh* partMesh(const std::vector<int>& indices, trimesh::TriMesh* inMesh);

	void dumplicateMesh(trimesh::TriMesh* mesh);

	void mergeTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes, const trimesh::fxform& globalMatrix, bool fanzhuan);

	trimesh::vec3 moveTrimesh2Center(trimesh::TriMesh* mesh);
	void moveMeshes2BoxCenter(std::vector<trimesh::TriMesh*> meshes, const trimesh::box3& box, bool zZero = true);
}

#endif // MMESH_TRIMESHUTIL_1602590289222_H