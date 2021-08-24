#ifndef ENCHASE_TRIMESHGENERATOR_1620897704388_H
#define ENCHASE_TRIMESHGENERATOR_1620897704388_H
#include "trimesh2/TriMesh.h"
#include "mmesh/enchase/surface.h"

namespace enchase
{
	trimesh::TriMesh* enchaseGenerate(trimesh::TriMesh* input, SurfacePtr source, bool ball = false);
}

#endif // ENCHASE_TRIMESHGENERATOR_1620897704388_H