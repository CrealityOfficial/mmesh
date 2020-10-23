#include "mmesh/clipper/contour.h"

namespace mmesh
{
	void fill(trimesh::vec3* vertex, ClipperLib::Path& path)
	{
		size_t size = path.size();
		for (size_t i = 0; i < size; ++i)
		{
			ClipperLib::IntPoint& p = path.at(i);
			trimesh::vec3* data = vertex + i;
			*data = trimesh::vec3(INT2MM(p.X), INT2MM(p.Y), 0.0f);
		}
	}
}