#include "createsphere.h"
#include <map>

namespace mmesh
{
	

	void subdivison(trimesh::TriMesh* mesh, int num_iter)
	{
		for (size_t i = 0; i < num_iter; i++)
		{
			size_t num_faces = mesh->faces.size();

			for (size_t fid = 0; fid < num_faces; fid++)
			{
				int vid1 = mesh->faces[fid][0];
				int vid2 = mesh->faces[fid][1];
				int vid3 = mesh->faces[fid][2];

				size_t num_vertices = mesh->vertices.size();
				mesh->vertices.emplace_back(
					(mesh->vertices[vid1].x + mesh->vertices[vid2].x) * 0.5f,
					(mesh->vertices[vid1].y + mesh->vertices[vid2].y) * 0.5f,
					(mesh->vertices[vid1].z + mesh->vertices[vid2].z) * 0.5f);
				mesh->vertices.emplace_back(
					(mesh->vertices[vid2].x + mesh->vertices[vid3].x) * 0.5f,
					(mesh->vertices[vid2].y + mesh->vertices[vid3].y) * 0.5f,
					(mesh->vertices[vid2].z + mesh->vertices[vid3].z) * 0.5f);
				mesh->vertices.emplace_back(
					(mesh->vertices[vid3].x + mesh->vertices[vid1].x) * 0.5f,
					(mesh->vertices[vid3].y + mesh->vertices[vid1].y) * 0.5f,
					(mesh->vertices[vid3].z + mesh->vertices[vid1].z) * 0.5f);

				// replace current face with 4 small faces
				mesh->faces.emplace_back(vid1, num_vertices, num_vertices + 2);
				mesh->faces.emplace_back(vid2, num_vertices + 1, num_vertices);
				mesh->faces.emplace_back(vid3, num_vertices + 2, num_vertices + 1);
				mesh->faces[fid].set(num_vertices, num_vertices + 1, num_vertices + 2);
			}
		}
	}
	trimesh::TriMesh* Icosahedron( int num_iter)
	{
		trimesh::TriMesh* mesh = new trimesh::TriMesh();
		const double t = (1.0 + std::sqrt(5.0)) / 2.0;

		// Vertices
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(-1.0, t, 0.0)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(1.0, t, 0.0)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(-1.0, -t, 0.0)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(1.0, -t, 0.0)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(0.0, -1.0, t)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(0.0, 1.0, t)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(0.0, -1.0, -t)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(0.0, 1.0, -t)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(t, 0.0, -1.0)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(t, 0.0, 1.0)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(-t, 0.0, -1.0)));
		mesh->vertices.emplace_back(trimesh::normalized(trimesh::vec3(-t, 0.0, 1.0)));

		// Faces
		mesh->faces.emplace_back(trimesh::TriMesh::Face(0, 11, 5));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(0, 5, 1));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(0, 1, 7));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(0, 7, 10));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(0, 10, 11));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(1, 5, 9));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(5, 11, 4));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(11, 10, 2));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(10, 7, 6));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(7, 1, 8));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(3, 9, 4));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(3, 4, 2));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(3, 2, 6));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(3, 6, 8));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(3, 8, 9));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(4, 9, 5));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(2, 4, 11));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(6, 2, 10));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(8, 6, 7));
		mesh->faces.emplace_back(trimesh::TriMesh::Face(9, 8, 1));

		subdivison(mesh, num_iter);
		

		return mesh;
	}

	trimesh::TriMesh* SpherifiedCube(int divisions)
	{

		Mesh_sphere mesh_sphere;

		const double step = 1.0 / double(divisions);
		const trimesh::vec3 step3(step, step, step);

		for (uint32_t face = 0; face < 6; ++face)
		{
			const trimesh::vec3 origin = CubeToSphere::origins[face];
			const trimesh::vec3 right = CubeToSphere::rights[face];
			const trimesh::vec3 up = CubeToSphere::ups[face];
			for (uint32_t j = 0; j < divisions + 1; ++j)
			{
				const trimesh::vec3 j3(j, j, j);
				for (uint32_t i = 0; i < divisions + 1; ++i)
				{
					const trimesh::vec3 i3(i, i, i);
					const trimesh::vec3 p = origin + step3 * (i3 * right + j3 * up);
					const trimesh::vec3 p2 = p * p;
					const trimesh::vec3 n
					(
						p.x * std::sqrt(1.0 - 0.5 * (p2.y + p2.z) + p2.y * p2.z / 3.0),
						p.y * std::sqrt(1.0 - 0.5 * (p2.z + p2.x) + p2.z * p2.x / 3.0),
						p.z * std::sqrt(1.0 - 0.5 * (p2.x + p2.y) + p2.x * p2.y / 3.0)
					);
					mesh_sphere.vertices.emplace_back(n);
				}
			}
		}

		const uint32_t k = divisions + 1;
		for (uint32_t face = 0; face < 6; ++face)
		{
			for (uint32_t j = 0; j < divisions; ++j)
			{
				const bool bottom = j < (divisions / 2);
				for (uint32_t i = 0; i < divisions; ++i)
				{
					const bool left = i < (divisions / 2);
					const uint32_t a = (face * k + j) * k + i;
					const uint32_t b = (face * k + j) * k + i + 1;
					const uint32_t c = (face * k + j + 1) * k + i;
					const uint32_t d = (face * k + j + 1) * k + i + 1;
					if (bottom ^ left) mesh_sphere.addQuadAlt(a, c, d, b);
					else mesh_sphere.addQuad(a, c, d, b);
				}
			}
		}
		return mesh_sphere.convert();
	}

	trimesh::TriMesh* createSphere( float radius, int num_iter)
	{
		//trimesh::TriMesh * rMesh  = Icosahedron(num_iter);
		trimesh::TriMesh * rMesh = SpherifiedCube(num_iter);
		 
		for (size_t i = 0; i < rMesh->vertices.size(); i++)
		{
			float scale = radius / trimesh::length(rMesh->vertices[i]);
			rMesh->vertices[i].x *= scale;
			rMesh->vertices[i].y *= scale;
			rMesh->vertices[i].z *= scale;
		}

		return rMesh;
	}

}