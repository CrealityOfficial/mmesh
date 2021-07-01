#ifndef MMESH_CONVERT_1625103040213_H
#define MMESH_CONVERT_1625103040213_H
#include <clipper/clipper.hpp>
#include "trimesh2/Vec.h"

namespace mmesh
{
	void clipperConvert(const ClipperLib::Path& path, std::vector<trimesh::vec3>& vpath);
	void clipperConvert(const ClipperLib::Paths& paths, std::vector<trimesh::vec3>& vpath);
	void clipperConvert(const std::vector<ClipperLib::Paths*>& pathses, std::vector<trimesh::vec3>& vpath);
}

#endif // MMESH_CONVERT_1625103040213_H