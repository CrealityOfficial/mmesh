#include "savepolygonstack.h"
#include <iostream>

namespace mmesh
{
	void stackSave(const char* name, std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points)
	{
		std::fstream out(name, std::ios::out | std::ios::binary);
		if (out.is_open())
		{
			int pNum = (int)points.size();
			out.write((const char*)&pNum, sizeof(int));
			if (pNum > 0)
				out.write((const char*)&points.at(0), sizeof(trimesh::dvec2) * pNum);

			int plNum = (int)polygons.size();
			out.write((const char*)&plNum, sizeof(int));
			if (plNum > 0)
			{
				for (int i = 0; i < plNum; ++i)
				{
					int num = (int)polygons.at(i).size();
					out.write((const char*)&num, sizeof(int));
					if(num)
						out.write((const char*)&polygons.at(i).at(0), num * sizeof(int));
				}
			}
		}
		out.close();
	}

	void stackLoad(const char* name, std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points)
	{

	}
}