#ifndef MMESH_SPLIT_1612341980784_H
#define MMESH_SPLIT_1612341980784_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	class SplitBase
	{
	public:
		SplitBase();
		~SplitBase();

		void setInputMesh(trimesh::TriMesh* mesh);
	protected:
		trimesh::TriMesh* m_mesh;
	};
}

#endif // MMESH_SPLIT_1612341980784_H