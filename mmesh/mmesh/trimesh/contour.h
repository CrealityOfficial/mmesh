#ifndef MMESH_CONTOUR_1603376186170_H
#define MMESH_CONTOUR_1603376186170_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	void zoff(trimesh::vec3* svertex, trimesh::vec3* dvertex, size_t size, float z);
	void fillLoop(int sindex1, int sindex2, int size, trimesh::TriMesh::Face* faces);
}

#endif // MMESH_CONTOUR_1603376186170_H