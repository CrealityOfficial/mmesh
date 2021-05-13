#ifndef MMESH_ZSPLIT_1612342117768_H
#define MMESH_ZSPLIT_1612342117768_H
#include "split.h"

namespace mmesh
{
	class ZSplit
	{
	public:
		ZSplit();
		virtual ~ZSplit();

		trimesh::TriMesh* splitPolygon(float z);
	};
}

#endif // MMESH_ZSPLIT_1612342117768_H