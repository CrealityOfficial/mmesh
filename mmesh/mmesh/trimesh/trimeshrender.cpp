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

	void traitTrianglesFromMeshes(const std::vector<trimesh::TriMesh*>& meshes, std::vector<trimesh::vec3>& tris)
	{
		for (const trimesh::TriMesh* mesh : meshes)
		{
			if (!mesh)
				continue;

			if (mesh->faces.size() > 0)
			{
				for (const trimesh::TriMesh::Face& f : mesh->faces)
				{
					tris.push_back(mesh->vertices.at(f.x));
					tris.push_back(mesh->vertices.at(f.y));
					tris.push_back(mesh->vertices.at(f.z));
				}
			}
			else
			{
				tris.insert(tris.end(), mesh->vertices.begin(), mesh->vertices.end());
			}
		}
	}
}