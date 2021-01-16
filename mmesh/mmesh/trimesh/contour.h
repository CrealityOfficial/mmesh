#ifndef MMESH_CONTOUR_1603376186170_H
#define MMESH_CONTOUR_1603376186170_H
#include "trimesh2/TriMesh.h"
#include "trimesh2/Box.h"

namespace mmesh
{
	void zoff(trimesh::vec3* svertex, trimesh::vec3* dvertex, size_t size, float z);
	void fillLoop(int sindex1, int sindex2, int size, trimesh::TriMesh::Face* faces);

	void sortBoxes(const trimesh::box3& box, const std::vector<trimesh::box3>& boxes, float gap,
		std::vector<trimesh::vec3>& newPosition);
	void sortBoxes(const trimesh::box3& box, const std::vector<trimesh::box3>& boxes,
		std::vector<trimesh::vec3>& newPosition);
}

#endif // MMESH_CONTOUR_1603376186170_H