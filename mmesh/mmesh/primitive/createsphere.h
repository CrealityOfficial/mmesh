#ifndef VMESH_CREATE_SPHERE_163662851SDS4_H
#define VMESH_CREATE_SPHERE_163662851SDS4_H

#include "trimesh2/TriMesh.h"

namespace mmesh
{
	namespace CubeToSphere
	{
		static const trimesh::vec3 origins[6] =
		{
			 trimesh::vec3(-1.0, -1.0, -1.0),
			 trimesh::vec3(1.0, -1.0, -1.0),
			 trimesh::vec3(1.0, -1.0, 1.0),
			 trimesh::vec3(-1.0, -1.0, 1.0),
			 trimesh::vec3(-1.0, 1.0, -1.0),
			 trimesh::vec3(-1.0, -1.0, 1.0)
		};
		static const trimesh::vec3 rights[6] =
		{
			 trimesh::vec3(2.0, 0.0, 0.0),
			 trimesh::vec3(0.0, 0.0, 2.0),
			 trimesh::vec3(-2.0, 0.0, 0.0),
			 trimesh::vec3(0.0, 0.0, -2.0),
			 trimesh::vec3(2.0, 0.0, 0.0),
			 trimesh::vec3(2.0, 0.0, 0.0)
		};
		static const trimesh::vec3 ups[6] =
		{
			 trimesh::vec3(0.0, 2.0, 0.0),
			 trimesh::vec3(0.0, 2.0, 0.0),
			 trimesh::vec3(0.0, 2.0, 0.0),
			 trimesh::vec3(0.0, 2.0, 0.0),
			 trimesh::vec3(0.0, 0.0, 2.0),
			 trimesh::vec3(0.0, 0.0, -2.0)
		};
	};
	struct Mesh_sphere
	{
		std::vector<trimesh::vec3> vertices;
		std::vector<uint32_t> triangles;
		void addQuad(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
		{
			triangles.emplace_back(a);
			triangles.emplace_back(b);
			triangles.emplace_back(c);
			triangles.emplace_back(a);
			triangles.emplace_back(c);
			triangles.emplace_back(d);
		}

		void addQuadAlt(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
		{
			triangles.emplace_back(a);
			triangles.emplace_back(b);
			triangles.emplace_back(d);
			triangles.emplace_back(b);
			triangles.emplace_back(c);
			triangles.emplace_back(d);
		}
		trimesh::TriMesh*  convert()
		{
			trimesh::TriMesh* mesh = new trimesh::TriMesh();
			for (int i = 0; i < this->triangles.size() / 3; i++)
			{
				trimesh::TriMesh::Face f(this->triangles.at(i * 3), this->triangles.at(i * 3 + 1), this->triangles.at(i * 3 + 2));
				mesh->faces.emplace_back(f);

			}
			for (int i = 0; i < this->vertices.size(); i++)
			{
				trimesh::vec3 p = this->vertices.at(i);
				mesh->vertices.emplace_back(trimesh::vec3(p.x, p.y, p.z));
			}
			return mesh;
		}
	};

	struct Edge
	{
		uint32_t v0;
		uint32_t v1;

		Edge(uint32_t v0, uint32_t v1)
			: v0(v0 < v1 ? v0 : v1)
			, v1(v0 < v1 ? v1 : v0)
		{
		}

		bool operator <(const Edge& rhs) const
		{
			return v0 < rhs.v0 || (v0 == rhs.v0 && v1 < rhs.v1);
		}
	};

	trimesh::TriMesh* createSphere(float radius, int num_iter);
}

#endif // VMESH_CREATE_ANNULUS_1636628517454_H