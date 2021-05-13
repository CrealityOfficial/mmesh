#ifndef MMESH_SHAPECREATOR_1619603379371_H
#define MMESH_SHAPECREATOR_1619603379371_H
#include <trimesh2/Box.h>
#include <trimesh2/TriMesh.h>

namespace mmesh
{
	trimesh::TriMesh* createBox(const trimesh::box3& box);
}

#endif // MMESH_SHAPECREATOR_1619603379371_H