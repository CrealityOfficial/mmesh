#ifndef MMESH_POLYSTACKTESTUTIL_1620527117250_H
#define MMESH_POLYSTACKTESTUTIL_1620527117250_H
#include "mmesh/trimesh/polygonstack.h"

namespace mmesh
{
	bool test_vector_face(const std::vector<trimesh::TriMesh::Face>& lfaces ,
		const std::vector<trimesh::TriMesh::Face>& rfaces);

	struct PolyStackTestBase
	{
		const char* input;
		const char* output;
	};

	class PolystackTestUtil
	{
	public:
		PolystackTestUtil();
		~PolystackTestUtil();

		void setup(const std::string& input);
		void run();
		void output(const std::string& output);
		void test(const std::string& bench);

		std::vector<std::vector<int>> m_polygons;
		std::vector<trimesh::dvec2> m_points;
		mmesh::PolygonStack m_stack;
	};
}

#endif // MMESH_POLYSTACKTESTUTIL_1620527117250_H