#ifndef MMESH_BALLCREATOR_1620831280553_H
#define MMESH_BALLCREATOR_1620831280553_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	class BallCreator
	{
	public:
		BallCreator();
		~BallCreator();

		// Icosahedron based algorithms
		// ref: https://en.wikipedia.org/wiki/Regular_icosahedron
		// num_iter: a bigger value results in smaller triangles, and more precise triangulation
		// createUV:
		//    0: no uv
		//    1: get uv by mapping rectangle area to sphere
		//	  2:get uv by mapping circle area to sphere(Azimuthal Projection)
		//    3: (Equirectangular Projection)
		static trimesh::TriMesh* create(float radius = 1.0f, unsigned int num_iter = 4, int createUV = 0);

	};
}

#endif // MMESH_BALLCREATOR_1620831280553_H