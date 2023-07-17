#ifndef MMESH_TRIMESHRENDER_1688522129048_H
#define MMESH_TRIMESHRENDER_1688522129048_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	void traitTriangles(trimesh::TriMesh* mesh, const std::vector<int>& indices, std::vector<trimesh::vec3>& tris);
	void traitTriangles(trimesh::TriMesh* mesh, std::vector<trimesh::vec3>& tris);
	void traitTrianglesFromMeshes(const std::vector<trimesh::TriMesh*>& meshes, std::vector<trimesh::vec3>& tris);
}

#endif // MMESH_TRIMESHRENDER_1688522129048_H