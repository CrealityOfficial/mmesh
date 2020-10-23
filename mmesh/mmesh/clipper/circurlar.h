#ifndef MMESH_CIRCURLAR_1603379885432_H
#define MMESH_CIRCURLAR_1603379885432_H
#include <clipper/clipper.hpp>

namespace mmesh
{
	void loopPolyTree(polyNodeFunc func, ClipperLib::PolyTree* polyTree);
}

#endif // MMESH_CIRCURLAR_1603379885432_H