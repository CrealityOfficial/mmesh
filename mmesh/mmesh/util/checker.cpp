#include "checker.h"

namespace mmesh
{
	bool checkDegenerateFace(trimesh::TriMesh* mesh, bool remove)
	{
		if (!mesh || mesh->faces.size() <= 0)
			return false;

		size_t size = mesh->faces.size();

		std::vector<int> newFaces;
		newFaces.reserve(size);
		for (size_t i = 0; i < size; ++i)
		{
			const trimesh::TriMesh::Face& face = mesh->faces.at(i);
			if ((face.x == face.y) || (face.x == face.z) || (face.y == face.z))
				continue;

			newFaces.push_back(i);
		}

		bool have = newFaces.size() != mesh->faces.size();
		if (have && remove)
		{
			bool hasColor = mesh->colors.size() > 0;

			std::vector<trimesh::TriMesh::Face> faces;
			std::vector<trimesh::Color> colors;
			for (int i : newFaces)
			{
				faces.push_back(mesh->faces.at(i));
				if(hasColor)
					colors.push_back(mesh->colors.at(i));
			}
			mesh->faces.swap(faces);

			if(hasColor)
				mesh->colors.swap(colors);
		}
		return have;
	}
}