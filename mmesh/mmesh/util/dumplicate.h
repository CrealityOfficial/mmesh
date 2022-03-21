#ifndef MMESH_MNODE_1622032440408_H
#define MMESH_MNODE_1622032440408_H
#include "trimesh2/TriMesh.h"

namespace ccglobal
{
	class Tracer;
}

namespace mmesh
{
	void qDumplicateMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer = nullptr);
}

#endif // MMESH_MNODE_1622032440408_H