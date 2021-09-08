#ifndef MMESH_TRIMESHUTIL_1602590289222_H
#define MMESH_TRIMESHUTIL_1602590289222_H
#include <vector>
#include "trimesh2/XForm.h"
#include "trimesh2/Box.h"

namespace trimesh
{
	class TriMesh;
}

namespace ccglobal
{
	class Tracer;
}

namespace mmesh
{
	void mergeTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes, bool fanzhuan = false);
	void mergeTriMesh_omp(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes);

	void reverseTriMesh(trimesh::TriMesh* Mesh);

	void mergeTrianglesTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes); // inMeshes is triangles

	trimesh::TriMesh* partMesh(const std::vector<int>& indices, trimesh::TriMesh* inMesh);

	void dumplicateMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer = nullptr);

	void mergeTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes, const trimesh::fxform& globalMatrix, bool fanzhuan);

	trimesh::vec3 moveTrimesh2Center(trimesh::TriMesh* mesh, bool zZero = true);
	void moveMeshes2BoxCenter(std::vector<trimesh::TriMesh*> meshes, const trimesh::box3& box, bool zZero = true);

	void convertUV2Azi(trimesh::TriMesh* mesh);
	void convertUV2Equ(trimesh::TriMesh* mesh);

	void flipZ(trimesh::TriMesh* mesh);
}

#endif // MMESH_TRIMESHUTIL_1602590289222_H