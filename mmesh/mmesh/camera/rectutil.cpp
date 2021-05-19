#include "rectutil.h"

namespace mmesh
{
	int adjustY(int y, const trimesh::ivec2& size)
	{
		int ay = size.y - y;
		if (ay < 0) ay = 0;
		if (ay > size.y) ay = size.y;
		return ay;
	}

	float viewportRatio(int x, int w)
	{
		return (float)x / (float)w - 0.5f;
	}

	trimesh::vec2 viewportRatio(const trimesh::ivec2& p, const trimesh::ivec2& size)
	{
		trimesh::vec2 pf;
		pf.x = ((float)p.x / (float)size.x - 0.5f);
		pf.y = ((float)(size.y - p.y) / (float)size.y - 0.5f);
		return pf;
	}
}
