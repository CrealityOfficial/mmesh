#include "createsphere.h"
#include "createcone.h"


namespace mmesh
{
	trimesh::TriMesh* createCone(size_t resolution, float radius, float height )
	{

		Mesh_sphere  mesh;
		 //generate vertices
		 //add vertices subdividing a circle
		//std::vector<Vertex> base_vertices;

		// add the tip of the cone
		auto v0 = trimesh::point(0.0, 0.0, height / 2.0);
		auto vb = trimesh::point(0.0, 0.0, -height / 2.0);
		mesh.vertices.push_back(v0);
		for (size_t i = 0; i < resolution; i++) {
			float ratio = static_cast<float>(i) / (resolution);
			float r = ratio * (M_PI * 2.0);
			float x = std::cos(r) * radius;
			float y = std::sin(r) * radius;
			auto v = trimesh:: point(x, y, -height/2.0);
			mesh.vertices.push_back(v);
		}
		
		// generate triangular faces
		for (size_t i = 1; i < resolution; i++)
		{
			mesh.triangles.push_back(i);
			mesh.triangles.push_back(i+1);
			mesh.triangles.push_back(0);
		}
		mesh.triangles.push_back(1);
		mesh.triangles.push_back(0);
		mesh.triangles.push_back(resolution);
		
			
		mesh.vertices.emplace_back(vb);

		for (size_t i = 1; i < resolution+1; i++)
		{
			mesh.triangles.push_back(i + 1);
			mesh.triangles.push_back(i);
			mesh.triangles.push_back(resolution+1);
		}
		mesh.triangles.push_back(resolution);
		mesh.triangles.push_back(resolution + 1);
		mesh.triangles.push_back(1);

		return mesh.convert();
	}

}