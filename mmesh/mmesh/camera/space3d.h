#ifndef _MMESH_SPACE3D_1588841647525_H
#define _MMESH_SPACE3D_1588841647525_H
#include "trimesh2/Vec.h"
#include "mmesh/camera/ray.h"

namespace mmesh
{
	bool lineCollidePlane(const trimesh::vec3& planeCenter, const trimesh::vec3& planeDir, const mmesh::Ray& ray, trimesh::vec3& collide);
}
#endif // _MMESH_SPACE3D_1588841647525_H
