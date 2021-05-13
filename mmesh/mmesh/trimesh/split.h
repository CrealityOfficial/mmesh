#ifndef MMESH_SPLIT_1612341980784_H
#define MMESH_SPLIT_1612341980784_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	bool split(trimesh::TriMesh* inputMesh, float z, const trimesh::vec3& normal,
		trimesh::TriMesh** mesh1, trimesh::TriMesh** mesh2);
}

#endif // MMESH_SPLIT_1612341980784_H