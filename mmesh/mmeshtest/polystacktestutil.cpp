#include "polystacktestutil.h"
#include "mmesh/trimesh/savepolygonstack.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include "mmeshtest/testhelper.h"

#include "ccglobal/spycc.h"
namespace boost 
{
	namespace serialization 
	{
		template <typename Archive>
		void serialize(Archive& ar, trimesh::TriMesh::Face& face, const unsigned int version)
		{
			ar& face.x;
			ar& face.y;
			ar& face.z;
		}
	}
}

namespace mmesh
{
	PolystackTestUtil::PolystackTestUtil()
	{

	}

	PolystackTestUtil::~PolystackTestUtil()
	{

	}

	void PolystackTestUtil::setup(const std::string& input)
	{
		m_stack.clear();
		mmesh::stackLoad(input.c_str(), m_polygons, m_points);
		m_stack.prepare(m_polygons, m_points);
	}

	void PolystackTestUtil::run()
	{
		std::vector<trimesh::TriMesh::Face> faces;
		m_stack.generate(faces);
	}

	void PolystackTestUtil::output(const std::string& output)
	{
		std::vector<trimesh::TriMesh::Face> faces;
		m_stack.generate(faces);

		std::ofstream ofs(output, std::ios::binary | std::ios::out);

		if (ofs.is_open())
		{
			boost::archive::binary_oarchive oa(ofs);
			oa << faces;
		}
	}

	void PolystackTestUtil::test(const std::string& bench)
	{
		std::vector<trimesh::TriMesh::Face> faces;
		m_stack.generate(faces);

		std::vector<trimesh::TriMesh::Face> tfaces;
		std::ifstream ifs(bench, std::ios::binary | std::ios::in);
		if (ifs.is_open())
		{
			boost::archive::binary_iarchive oa(ifs);
			oa >> tfaces;

			test_equal(faces, tfaces);
		}
	}
}