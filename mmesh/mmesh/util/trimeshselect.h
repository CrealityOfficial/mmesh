#ifndef TRIMESH_SELECT_H
#define TRIMESH_SELECT_H

#include "trimesh2/Vec.h"
#include <vector>

namespace trimesh
{
	class TriMesh;
}

namespace mmesh
{
	int trimeshSelect(trimesh::TriMesh* mesh, int faceid, std::vector<trimesh::vec3>& vertexData, float offset = 0.1f);


	trimesh::vec3 trimeshFaceNormal(trimesh::TriMesh* mesh, int faceID);
}

#endif //