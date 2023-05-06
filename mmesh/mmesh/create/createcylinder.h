#ifndef MMESH_CREATECYLINDER_1619866821989_H
#define MMESH_CREATECYLINDER_1619866821989_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	trimesh::TriMesh* createCylinder(float _radius, float _height);
	trimesh::TriMesh* createSoupObliqueCylinder(int count, float _radius, float _height);
	trimesh::TriMesh* createSoupObliqueCylinder(int count, const trimesh::vec2& v1, const trimesh::vec2& v2, float _radius);
	trimesh::TriMesh* createSoupCylinder(int count, float _radius, float _height, bool offsetOnZero = false);
	trimesh::TriMesh* createSoupCylinder(int count, float _radius, float _height,
		const trimesh::vec3& centerPoint, const trimesh::vec3& normal);
	trimesh::TriMesh* createCylinderBasepos(int count, float radius, float height, const trimesh::vec3& sp, const trimesh::vec3& normal);
	trimesh::TriMesh* createCylinderWallBasepos(int count, float radius, float height, const trimesh::vec3& sp);
	trimesh::TriMesh* createHollowCylinder(trimesh::TriMesh* wallOutter, trimesh::TriMesh* wallInner, int sectionCount, const trimesh::vec3& normal);

}

#endif // MMESH_CREATECYLINDER_1619866821989_H