#include "simplify.h"

namespace mmesh
{
	trimesh::TriMesh* simplify(trimesh::TriMesh* mesh)
	{
		if (!mesh)
			return nullptr;

		trimesh::TriMesh* newMesh = new trimesh::TriMesh();
		*newMesh = *mesh;
		return newMesh;
	}
}