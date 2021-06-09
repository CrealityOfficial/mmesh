#include "booleanserail.h"

namespace mmesh
{
	void saveBoolean(const std::string& fileName, trimesh::TriMesh* mesh1, trimesh::TriMesh* mesh2)
	{
		if (!mesh1 || !mesh2)
			return;

		std::fstream out(fileName, std::ios::binary | std::ios::out);
		if (out.is_open())
		{
			saveTriMesh(mesh1, out);
			saveTriMesh(mesh2, out);
		}
		out.close();
	}

	void loadBoolean(const std::string& fileName, trimesh::TriMesh*& mesh1, trimesh::TriMesh*& mesh2)
	{
		std::fstream in(fileName, std::ios::binary | std::ios::in);
		if (in.is_open())
		{
			mesh1 = new trimesh::TriMesh();
			mesh2 = new trimesh::TriMesh();

			loadTriMesh(mesh1, in);
			loadTriMesh(mesh2, in);
		}
		in.close();
	}

	void saveTriMesh(trimesh::TriMesh* mesh, std::fstream& out)
	{
		saveVectorT<trimesh::vec3>(mesh->vertices, out);
		saveVectorT<trimesh::TriMesh::Face>(mesh->faces, out);
	}

	void loadTriMesh(trimesh::TriMesh* mesh, std::fstream& in)
	{
		loadVectorT<trimesh::vec3>(mesh->vertices, in);
		loadVectorT<trimesh::TriMesh::Face>(mesh->faces, in);
	}
}