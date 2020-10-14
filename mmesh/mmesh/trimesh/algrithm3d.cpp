#include "mmesh/trimesh/algrithm3d.h"

namespace mmesh
{
	bool rayIntersectTriangle(const trimesh::vec3& orig, const trimesh::vec3& dir,
		trimesh::vec3& v0, trimesh::vec3& v1, trimesh::vec3& v2, float* t, float* u, float* v)
	{
		trimesh::vec3 E1 = v1 - v0;
		trimesh::vec3 E2 = v2 - v0;

		trimesh::vec3 P = trimesh::cross(dir, E2);
		float det = trimesh::dot(E1, P);

		// keep det > 0, modify T accordingly
		trimesh::vec3 T;
		if (det > 0)
		{
			T = orig - v0;
		}
		else
		{
			T = v0 - orig;
			det = -det;
		}

		// If determinant is near zero, ray lies in plane of triangle
		if (det < 0.0001f)
			return false;

		// Calculate u and make sure u <= 1
		*u = trimesh::dot(T, P);
		if (*u < 0.0f || *u > det)
			return false;

		trimesh::vec3 Q = trimesh::cross(T, E1);

		// Calculate v and make sure u + v <= 1
		*v = trimesh::dot(dir, Q);
		if (*v < 0.0f || *u + *v > det)
			return false;

		// Calculate t, scale parameters, ray intersects triangle
		*t = trimesh::dot(E2, Q);

		float fInvDet = 1.0f / det;
		*t *= fInvDet;
		*u *= fInvDet;
		*v *= fInvDet;

		return true;
	}

	bool rayIntersectPlane(const trimesh::vec3& orig, const trimesh::vec3& dir, trimesh::vec3& vertex, trimesh::vec3& normal, float& t)
	{
		float l = trimesh::dot(normal, dir);
		if (l == 0.0f) return false;

		t = trimesh::dot(normal, (vertex - orig)) / l;
		return true;
	}

}