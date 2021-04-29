#ifndef MMESH_SHAPEFILLER_1595391864496_H
#define MMESH_SHAPEFILLER_1595391864496_H
#include "trimesh2/Vec.h"

namespace mmesh
{
	class ShapeFiller
	{
	public:
		ShapeFiller();
		~ShapeFiller();

		static int fillSphere(trimesh::vec3* data, float radius, trimesh::vec3 center);
		static int fillLink(trimesh::vec3* data, float radius1, trimesh::vec3& center1, float radius2, trimesh::vec3& center2);
		static int fillCylinder(trimesh::vec3* data, float radius1, trimesh::vec3& center1, float radius2, trimesh::vec3& center2
			, int n, float theta);
	};
}
#endif // MMESH_SHAPEFILLER_1595391864496_H