#ifndef CREATIVE_KERNEL_ALGRITHM3D_1593580656409_H
#define CREATIVE_KERNEL_ALGRITHM3D_1593580656409_H
#include "trimesh2/Vec.h"
#include "trimesh2/Box.h"
#include "trimesh2/XForm.h"
#include <vector>

namespace mmesh
{
	bool rayIntersectTriangle(const trimesh::vec3& orig, const trimesh::vec3& dir,
        trimesh::vec3& v0, trimesh::vec3& v1, trimesh::vec3& v2, float* t, float* u, float* v);
    bool rayIntersectTriangle(const trimesh::vec3& orig, const trimesh::vec3& dir,
        trimesh::vec3& v0, trimesh::vec3& v1, trimesh::vec3& v2, trimesh::vec3& baryPosition);

	bool rayIntersectPlane(const trimesh::vec3& orig, const trimesh::vec3& dir, trimesh::vec3& vertex, trimesh::vec3& normal, float& t);
	bool PointinTriangle(const trimesh::vec3& A, const trimesh::vec3& B, const trimesh::vec3& C, const trimesh::vec3& P);

	float GetArea(const trimesh::vec3& A, const trimesh::vec3& B, const trimesh::vec3& C);

	trimesh::box3 extendBox(const trimesh::box3& b, float r);

	void offsetPoints(std::vector<trimesh::vec3>& points, const trimesh::vec3& offset);
	void applyMatrix2Points(std::vector<trimesh::vec3>& points, const trimesh::fxform& xf);
	trimesh::box3 pointsBox(const std::vector<trimesh::vec3>& points);

	float getAngelOfTwoVector(const trimesh::vec& pt1, const trimesh::vec& pt2, const trimesh::vec& c);
	void getDevidePoint(const trimesh::vec& p0, const trimesh::vec& p1,
		std::vector<trimesh::vec>& out, float theta, bool clockwise);
}
#endif // CREATIVE_KERNEL_ALGRITHM3D_1593580656409_H