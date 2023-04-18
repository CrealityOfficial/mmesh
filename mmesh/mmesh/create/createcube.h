#ifndef MMESH_CREATECUBE_1619866821989_H
#define MMESH_CREATECUBE_1619866821989_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	trimesh::TriMesh* createCube(const trimesh::box3& box, float gap = 10.0);

	void boxLineIndices(const trimesh::box3& box, std::vector<trimesh::vec3>& corners, std::vector<int>& indices);
	void boxLines(const trimesh::box3& box, std::vector<trimesh::vec3>& lines);
}

#endif // MMESH_CREATECUBE_1619866821989_H