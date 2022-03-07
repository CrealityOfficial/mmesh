#ifndef MMESH_CREATECYLINDER_1619866821989_H
#define MMESH_CREATECYLINDER_1619866821989_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	trimesh::TriMesh* createCylinder(float _radius, float _height);
	trimesh::TriMesh* createSoupCylinder(int count, float _radius, float _height);
	trimesh::TriMesh* createSoupCylinder(int count, float _radius, float _height,
		const trimesh::vec3& position, const trimesh::vec3& normal);
}

#endif // MMESH_CREATECYLINDER_1619866821989_H