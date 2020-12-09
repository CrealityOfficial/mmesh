#ifndef MMESH_BOOLEAN_1607481576423_H
#define MMESH_BOOLEAN_1607481576423_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	trimesh::TriMesh* minus(trimesh::TriMesh* A, trimesh::TriMesh* B);
	trimesh::TriMesh* plus(trimesh::TriMesh* A, trimesh::TriMesh* B);
	trimesh::TriMesh* intersect(trimesh::TriMesh* A, trimesh::TriMesh* B);
}

#endif // MMESH_BOOLEAN_1607481576423_H