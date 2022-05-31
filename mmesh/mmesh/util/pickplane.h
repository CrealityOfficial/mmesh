#ifndef PICKPLANE_1622032440408_H
#define PICKPLANE_1622032440408_H
#include "trimesh2/TriMesh.h"

namespace ccglobal
{
	class Tracer;
}

namespace mmesh
{

	void pickPlane(trimesh::TriMesh* mesh, trimesh::vec3 positon , trimesh::vec3 normal, std::vector<int>& faceIndex,ccglobal::Tracer* tracer = nullptr);

	void pickPlane(trimesh::TriMesh* mesh, int& curFaceIndex, std::vector<int>& faceIndex, ccglobal::Tracer* tracer = nullptr);
}

#endif // PICKPLANE_1622032440408_H