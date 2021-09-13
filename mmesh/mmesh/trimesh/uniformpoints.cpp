#include "uniformpoints.h"

namespace mmesh
{
	UniformPoints::UniformPoints()
	{

	}

	UniformPoints::~UniformPoints()
	{

	}

	int UniformPoints::add(const trimesh::vec3& point)
	{
		int index = -1;
		point_iterator it = upoints.find(point);
		if (it != upoints.end())
		{
			index = (*it).second;
		}
		else
		{
			index = (int)upoints.size();
			upoints.insert(unique_point::value_type(point, index));
		}

		return index;
	}

	int UniformPoints::uniformSize()
	{
		return (int)upoints.size();
	}

	void UniformPoints::toVector(std::vector<trimesh::vec3>& points)
	{
		int size = uniformSize();
		if (size == 0)
			return;

		points.resize(size);
		for (auto it = upoints.begin(); it != upoints.end(); ++it)
		{
			points.at((*it).second) = (*it).first;
		}
	}

	void UniformPoints::clear()
	{
		upoints.clear();
	}
}