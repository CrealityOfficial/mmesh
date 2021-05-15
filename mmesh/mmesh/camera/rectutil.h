#ifndef _qtuser_3d_RECTUTIL_1588842280407_H
#define _qtuser_3d_RECTUTIL_1588842280407_H
#include "trimesh2/Vec.h"

namespace mmesh
{
	int adjustY(int y, const trimesh::ivec2& size);

	float viewportRatio(int x, int w);
	trimesh::vec2 viewportRatio(const trimesh::ivec2& p, const trimesh::ivec2& size);
}
#endif // _qtuser_3d_RECTUTIL_1588842280407_H
