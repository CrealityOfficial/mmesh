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

	//
	void mergeIndexPolygon(std::vector<IndexPolygon>& indexPolygons)
	{
		size_t indexPolygonSize = indexPolygons.size();
		std::map<int, IndexPolygon*> IndexPolygonMap;
		for (size_t i = 0; i < indexPolygonSize; ++i)
		{
			IndexPolygon& p1 = indexPolygons.at(i);
			if (!p1.closed())
				IndexPolygonMap.insert(std::pair<int, IndexPolygon*>(p1.start, &p1));
		}

		//combime
		for (size_t i = 0; i < indexPolygonSize; ++i)
		{
			IndexPolygon& p1 = indexPolygons.at(i);

			if (p1.polygon.size() == 0 || p1.closed())
				continue;

			auto it = IndexPolygonMap.find(p1.end);
			while (it != IndexPolygonMap.end())
			{

				IndexPolygon& p2 = *(*it).second;
				if (p2.polygon.size() == 0)
					break;

				bool merged = false;
				if (p1.start == p2.end)
				{
					p1.start = p2.start;
					for (auto iter = p2.polygon.rbegin(); iter != p2.polygon.rend(); ++iter)
					{
						if ((*iter) != p1.polygon.front()) p1.polygon.push_front((*iter));
					}
					merged = true;
				}
				else if (p1.end == p2.start)
				{
					p1.end = p2.end;
					for (auto iter = p2.polygon.begin(); iter != p2.polygon.end(); ++iter)
					{
						if ((*iter) != p1.polygon.back()) p1.polygon.push_back((*iter));
					}
					merged = true;
				}

				if (merged)
				{
					p2.polygon.clear();
				}
				else
					break;

				it = IndexPolygonMap.find(p1.end);
			}
		}

		std::vector<IndexPolygon> validIndexPolygons;
		for (size_t i = 0; i < indexPolygons.size(); ++i)
		{
			if (indexPolygons.at(i).polygon.size() > 0)
			{
				validIndexPolygons.push_back(indexPolygons.at(i));
			}
		}

		indexPolygons.swap(validIndexPolygons);
	}

}