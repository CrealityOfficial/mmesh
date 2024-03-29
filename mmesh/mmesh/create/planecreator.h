#ifndef MMESH_PLANECREATOR_1620884873798_H
#define MMESH_PLANECREATOR_1620884873798_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	trimesh::TriMesh* defaultPlane(int width, int height, float pixel, float h = 0.35f, bool createUV = false);

	void createGrid(std::vector<trimesh::vec3>& lines , const trimesh::box2& box, float pixel, float z = - 0.001f);
}

#endif // MMESH_PLANECREATOR_1620884873798_H