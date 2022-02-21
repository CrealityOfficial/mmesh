#include "space3d.h"

namespace mmesh
{
	bool lineCollidePlane(const trimesh::vec3& planeCenter, const trimesh::vec3& planeDir, const Ray& ray, trimesh::vec3& collide)
	{
		float l = ray.dir DOT planeDir;
		if (l == 0.0f) return false;

		float t = (planeDir DOT (planeCenter - ray.start)) / l;
		collide = ray.start + ray.dir * t;

		return true;
	}
}
