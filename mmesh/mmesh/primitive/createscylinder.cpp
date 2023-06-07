#include "createsphere.h"
#include "createscylinder.h"
#include <map>

namespace mmesh
{
	trimesh::TriMesh* createCylinder( float radius, float height, int resolution)
	{

		Mesh_sphere  mesh;
		// generate vertices
		std::vector<size_t> bottom_vertices;
		std::vector<size_t> top_vertices;
		trimesh::vec3 top(0.0f, 0.0f, height / 2.0f);
		trimesh::vec3 bottom(0.0f, 0.0f, -height / 2.0f);

		for (size_t i = 0; i < resolution; i++) {
			float ratio = (float)(i) / (resolution);
			float r = ratio * (M_PI * 2.0);
			float x = std::cos(r) * radius;
			float y = std::sin(r) * radius;
			trimesh::vec3 v = trimesh::vec3(x, y, -height / 2.0f);
			mesh.vertices.emplace_back(v);
			v = trimesh:: point(x, y, height / 2.0f);
			mesh.vertices.emplace_back(v);
		}

		// add faces around the cylinder
		for (size_t i = 0; i < resolution; i++) {
			auto ii = i * 2;
			auto jj = (ii + 2) % (resolution * 2);
			auto kk = (ii + 3) % (resolution * 2);
			auto ll = ii + 1;
			mesh.addQuad((ii), (jj), (kk), (ll));
		}

		
		mesh.vertices.emplace_back(bottom);

		for (size_t i = 0; i < mesh.vertices.size()-1; i+=2)
		{
			mesh.triangles.push_back(i);
			mesh.triangles.push_back(mesh.vertices.size()-1);
			mesh.triangles.push_back(i+2);
		}
		mesh.triangles.push_back((mesh.vertices.size()) - 3);
		mesh.triangles.push_back(mesh.vertices.size() - 1);
		mesh.triangles.push_back(0);

		mesh.vertices.emplace_back(top);
		for (size_t i = 1; i < mesh.vertices.size() - 1; i += 2)
		{
			mesh.triangles.push_back(i + 2);
			mesh.triangles.push_back(mesh.vertices.size() - 1);
			mesh.triangles.push_back(i);
		}
		mesh.triangles.push_back(mesh.vertices.size() - 1);
		mesh.triangles.push_back((mesh.vertices.size()) - 3);
		mesh.triangles.push_back(1);


		return mesh.convert();
	}

}