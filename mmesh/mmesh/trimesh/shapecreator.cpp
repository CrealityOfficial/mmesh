#include "shapecreator.h"

namespace mmesh
{
	trimesh::TriMesh* createBox(const trimesh::box3& box)
	{
		trimesh::TriMesh* mesh = new trimesh::TriMesh();
		mesh->vertices.resize(8);
		mesh->faces.resize(12);

		trimesh::vec3 bmin = box.min;
		trimesh::vec3 bmax = box.max;

		mesh->vertices[0] = bmin;
		mesh->vertices[1] = trimesh::vec3(bmax.x, bmin.y, bmin.z);
		mesh->vertices[2] = trimesh::vec3(bmax.x, bmax.y, bmin.z);
		mesh->vertices[3] = trimesh::vec3(bmin.x, bmax.y, bmin.z);
		mesh->vertices[4] = trimesh::vec3(bmin.x, bmin.y, bmax.z);
		mesh->vertices[5] = trimesh::vec3(bmax.x, bmin.y, bmax.z);
		mesh->vertices[6] = bmax;
		mesh->vertices[7] = trimesh::vec3(bmin.x, bmax.y, bmax.z);

		mesh->faces[0] = trimesh::TriMesh::Face(2, 3, 0);
		mesh->faces[1] = trimesh::TriMesh::Face(2, 0, 1);
		mesh->faces[2] = trimesh::TriMesh::Face(0, 1, 5);
		mesh->faces[3] = trimesh::TriMesh::Face(0, 5, 4);
		mesh->faces[4] = trimesh::TriMesh::Face(1, 2, 6);
		mesh->faces[5] = trimesh::TriMesh::Face(1, 6, 5);
		mesh->faces[6] = trimesh::TriMesh::Face(2, 3, 7);
		mesh->faces[7] = trimesh::TriMesh::Face(2, 7, 6);
		mesh->faces[8] = trimesh::TriMesh::Face(3, 0, 4);
		mesh->faces[9] = trimesh::TriMesh::Face(3, 4, 7);
		mesh->faces[10] = trimesh::TriMesh::Face(4, 5, 6);
		mesh->faces[11] = trimesh::TriMesh::Face(4, 6, 7);
		return mesh;
	}
}