#ifndef MMESH_MNODE_1622032440408_H
#define MMESH_MNODE_1622032440408_H
#include "trimesh2/TriMesh.h"

namespace ccglobal
{
	class Tracer;
}

namespace mmesh
{

	void weldingMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer = nullptr);

    //未用到的顶点用有效的顶点替换
    void removeNorVector2(trimesh::TriMesh* mesh);

	template<class T>
	void hashMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
	{

	}
}

#endif // MMESH_MNODE_1622032440408_H