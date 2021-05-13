#include "testhelper.h"
#include <gtest/gtest.h>
namespace mmesh
{
	void test_equal(const trimesh::TriMesh::Face& lface,
		const trimesh::TriMesh::Face& rface)
	{
		ASSERT_EQ(lface.x, rface.x);
		ASSERT_EQ(lface.y, rface.y);
		ASSERT_EQ(lface.z, rface.z);
	}

	void test_equal(const std::vector<trimesh::TriMesh::Face>& lfaces,
		const std::vector<trimesh::TriMesh::Face>& rfaces)
	{
		ASSERT_EQ(lfaces.size(), rfaces.size());
		size_t size = lfaces.size();
		for (size_t i = 0; i < size; ++i)
			test_equal(lfaces.at(i), rfaces.at(i));
	}
}