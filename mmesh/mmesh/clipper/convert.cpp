#include "convert.h"

namespace mmesh
{
	void clipperConvert(const ClipperLib::Path& path, std::vector<trimesh::vec3>& vpath)
	{
		int size = (int)path.size();
		if (size < 3)
			return;

		auto f = [](const ClipperLib::IntPoint& p)->trimesh::vec3 {
			return trimesh::vec3(INT2MM(p.X), INT2MM(p.Y), 0.0f);
		};
		for (int i = 0; i < size - 1; ++i)
		{
			int index1 = i;
			int index2 = (i + 1) % size;
			vpath.push_back(f(path.at(index1)));
			vpath.push_back(f(path.at(index2)));
		}

		//trimesh::vec3 startPoint;
		//for (int n = 0; n < path.size(); n++)
		//{
		//	trimesh::vec3 aVec3;
		//	aVec3.x = INT2MM(path[n].X);
		//	aVec3.y = INT2MM(path[n].Y);
		//	aVec3.z = 0.0;
		//	vpath.push_back(aVec3);
		//	if (n == 0)
		//	{
		//		startPoint = aVec3;
		//	}
		//	else //²»ÊÇÊ×
		//	{
		//		vpath.push_back(aVec3);
		//	}
		//	if (n == path.size() - 1)
		//	{
		//		vpath.push_back(aVec3);
		//		vpath.push_back(startPoint);
		//	}
		//}
	}

	void clipperConvert(const ClipperLib::Paths& paths, std::vector<trimesh::vec3>& vpath)
	{
		for (const ClipperLib::Path& apath : paths)
		{
			clipperConvert(apath, vpath);
		}
	}

	void clipperConvert(const std::vector<ClipperLib::Paths*>& pathses, std::vector<trimesh::vec3>& vpath)
	{
		for (const ClipperLib::Paths* apaths : pathses)
		{
			clipperConvert(*apaths, vpath);
		}
	}
}