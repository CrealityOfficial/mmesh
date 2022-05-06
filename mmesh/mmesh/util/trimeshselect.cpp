#include "trimeshselect.h"
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	int trimeshSelect(trimesh::TriMesh* mesh, int faceid, std::vector<trimesh::vec3>& vertexData, float offset)
	{
		if (!mesh || (mesh->faces.size() == 0) || (mesh->vertices.size() == 0))
			return -1;

		trimesh::TriMesh::Face& f = mesh->faces.at(faceid);

		trimesh::vec3 vertexVec3[3];
		for (int j = 0; j < 3; ++j)
		{
			vertexVec3[j] = mesh->vertices.at(f[j]);
		}

		trimesh::vec3 v01 = mesh->vertices.at(f[1]) - mesh->vertices.at(f[0]);
		trimesh::vec3 v02 = mesh->vertices.at(f[2]) - mesh->vertices.at(f[0]);
		trimesh::vec3 n = v01 TRICROSS v02;
		trimesh::normalize(n);

		trimesh::vec3 nOffset = n * offset;
		for (int j = 0; j < 3; ++j)
		{
			vertexVec3[j] += nOffset;
		}

		for (int i = 0; i < 3; ++i)
		{
			//trimesh::vec3& v = vertexVec3[i];
			vertexData.push_back(vertexVec3[i]);
		}
		return 3;
	}
}





