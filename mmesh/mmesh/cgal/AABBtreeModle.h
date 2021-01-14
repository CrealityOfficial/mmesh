// Author(s): Camille Wormser, Pierre Alliez
// Example of an AABB tree used with indexed triangle set

#ifndef __AABB_TREE_MODLE_H
#define __AABB_TREE_MODLE_H

#include "trimesh2/TriMesh.h"
#include "trimesh2/XForm.h"

namespace AABBTreeModle
{
	int CreateAABBTree(const trimesh::TriMesh* meshPtr, const trimesh::fxform* xfPtr);
	bool TriangleIntersect(const trimesh::TriMesh* meshPtr);
	bool TriangleFaceIntersect(const trimesh::vec3* v0, const trimesh::vec3* v1, const trimesh::vec3* v2);
	static bool testTriangleIntersect(const trimesh::TriMesh* meshPtr)
	{
		return TriangleIntersect(meshPtr);
	}
	static bool testTriangleFaceIntersect(const trimesh::vec3 *v0, const trimesh::vec3 *v1, const trimesh::vec3 *v2)
	{
		return   TriangleFaceIntersect(v0, v1, v2);
	}
}
#endif
