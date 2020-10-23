#ifndef MMESH_CONTOUR_1603376914865_H
#define MMESH_CONTOUR_1603376914865_H
#include <clipper/clipper.hpp>
#include "trimesh2/Vec.h"

namespace mmesh
{
	void fill(trimesh::vec3* vertex, ClipperLib::Path& path);
}

#endif // MMESH_CONTOUR_1603376914865_H