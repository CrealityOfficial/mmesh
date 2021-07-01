#include "convert.h"

namespace mmesh
{
	void clipperConvert(const ClipperLib::Path& path, std::vector<trimesh::vec3>& vpath)
	{
		if (path.size()<3)
		{
			return;
		}

		trimesh::vec3 startPoint;
		for (int n = 0; n < path.size(); n++)
		{
			trimesh::vec3 aVec3;
			aVec3.x = INT2MM(path[n].X);
			aVec3.y = INT2MM(path[n].Y);
			aVec3.z = 0.0;
			vpath.push_back(aVec3);
			if ((n != 0) && (n != path.size() - 1))//²»ÊÇÊ×Î²
			{
				vpath.push_back(aVec3);
			}
			if (n == 0)
			{
				startPoint = aVec3;
			}
			if (n == path.size() - 1)
			{
				vpath.push_back(startPoint);
			}
		}
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