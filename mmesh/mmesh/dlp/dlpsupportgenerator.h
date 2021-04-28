#ifndef MMESH_DLPSUPPORTGENERATOR_1619594835240_H
#define MMESH_DLPSUPPORTGENERATOR_1619594835240_H
#include <trimesh2/TriMesh.h>

namespace mmesh
{
	class DLPSupportGenerator
	{
	public:
		DLPSupportGenerator();
		~DLPSupportGenerator();

		void setInput(trimesh::TriMesh* mesh);
		trimesh::TriMesh* generate();
	};
}

#endif // MMESH_DLPSUPPORTGENERATOR_1619594835240_H