#include "mmesh/trimesh/contour.h"

#define FACE(x, y , z) trimesh::TriMesh::Face(x, y, z)

namespace mmesh
{
	void zoff(trimesh::vec3* svertex, trimesh::vec3* dvertex, size_t size, float z)
	{
		trimesh::vec3 offz = trimesh::vec3(0.0f, 0.0f, z);
		for (size_t i = 0; i < size; ++i)
		{
			*(dvertex + i) = *(svertex + i) + offz;
		}
	}

	void fillLoop(int sindex1, int sindex2, int size, trimesh::TriMesh::Face* faces)
	{
		if (size < 2) return;

		int findex = 0;
		for (int i = 0; i < size; ++i)
		{
			int i1 = sindex1 + i;
			int i3 = sindex2 + i;
			int i2 = i1 + 1;
			int i4 = i3 + 1;

			if (i == size - 1)
			{
				i2 = sindex1;
				i4 = sindex2;
			}

			*(faces + findex++) = FACE(i1, i2, i4);
			*(faces + findex++) = FACE(i1, i4, i3);
		}
	}
}