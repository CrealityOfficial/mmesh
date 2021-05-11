#ifndef MMESH_TESTHELPER_1620554099576_H
#define MMESH_TESTHELPER_1620554099576_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	void test_equal(const trimesh::TriMesh::Face& lface,
		const trimesh::TriMesh::Face& rface);
	void test_equal(const std::vector<trimesh::TriMesh::Face>& lfaces,
		const std::vector<trimesh::TriMesh::Face>& rfaces);
}

#endif // MMESH_TESTHELPER_1620554099576_H