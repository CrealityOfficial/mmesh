#ifndef MMESH_CREATECUBE_1619866821989_H
#define MMESH_CREATECUBE_1619866821989_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	trimesh::TriMesh* createCube(const trimesh::box3& box, float gap =10.0);
}

#endif // MMESH_CREATECUBE_1619866821989_H