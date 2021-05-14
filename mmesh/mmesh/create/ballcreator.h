#ifndef MMESH_BALLCREATOR_1620831280553_H
#define MMESH_BALLCREATOR_1620831280553_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	class BallCreator
	{
	public:
		BallCreator();
		~BallCreator();

		trimesh::TriMesh* create();
	protected:
	};
}

#endif // MMESH_BALLCREATOR_1620831280553_H