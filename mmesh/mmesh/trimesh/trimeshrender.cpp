#include "trimeshrender.h"
#include "ccglobal/log.h"

namespace mmesh
{
	void traitTriangles(trimesh::TriMesh* mesh, const std::vector<int>& indices, std::vector<trimesh::vec3>& tris)
	{
		if (!mesh)
			return;

		int nFace = (int)mesh->faces.size();
		int nVert = (int)mesh->vertices.size();
		for (int i : indices)
		{
			if (i < 0 || i > nFace)
			{
				LOGW("traitTriangles invalid face index [%d]", i);
				continue;
			}

			const trimesh::TriMesh::Face& f = mesh->faces.at(i);
			for (int j = 0; j < 3; ++j)
				tris.push_back(mesh->vertices.at(f[j]));
		}
	}
}