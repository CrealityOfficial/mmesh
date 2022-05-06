#ifndef MMESH_GEOMETRY_UTIL_H
#define MMESH_GEOMETRY_UTIL_H

#include "trimesh2/Vec.h"

namespace mmesh
{
	float GetArea(trimesh::vec3& A, trimesh::vec3& B, trimesh::vec3& C);

	bool PointinTriangle(trimesh::vec3& A, trimesh::vec3& B, trimesh::vec3& C, trimesh::vec3& P);
}

#endif // MMESH_GEOMETRY_UTIL_H