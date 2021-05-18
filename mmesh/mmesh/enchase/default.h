#ifndef ENCHASE_DEFAULT_1600265605949_H
#define ENCHASE_DEFAULT_1600265605949_H
#include "trimesh2/Vec.h"
#include <vector>

namespace trimesh
{
	class TriMesh;
}

namespace enchase
{
	class TextureGroup;
	trimesh::TriMesh* defaultPlane(int width, int height, float pixel, float h = 0.35f);
	void defaultCoord(int width, int height, std::vector<TextureGroup*>& textureGroups);
}

#endif // ENCHASE_DEFAULT_1600265605949_H