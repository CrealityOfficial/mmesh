#pragma once
#include "trimesh2/Vec.h"
#include "trimesh2/TriMesh.h"


namespace mmesh
{
	class FPoint3
	{
	public:
		float x, y, z;
		FPoint3() {}
		FPoint3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
		FPoint3(const trimesh::point& p) : x(p.x* .001), y(p.y* .001), z(p.z* .001) {}
		float vSize2() const
		{
			return x * x + y * y + z * z;
		}

		float vSize() const
		{
			return sqrt(vSize2());
		}
		bool operator==(FPoint3& p) const { return x == p.x && y == p.y && z == p.z; }
		FPoint3 operator/(const float f) const { return FPoint3(x / f, y / f, z / f); }
		FPoint3 operator*(const float f) const { return FPoint3(x * f, y * f, z * f); }
		FPoint3& operator *= (const float f) { x *= f; y *= f; z *= f; return *this; }
		FPoint3 cross(const FPoint3 & p) const
		{
			return FPoint3(
				y * p.z - z * p.y,
				z * p.x - x * p.z,
				x * p.y - y * p.x);
		}
		double operator*(FPoint3 lhs)
		{
			return lhs.x* x + lhs.y * y + lhs.z * z;
		}
	};
}
