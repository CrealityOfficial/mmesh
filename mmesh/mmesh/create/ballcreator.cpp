#include "ballcreator.h"

namespace mmesh
{
	BallCreator::BallCreator()
	{

	}

	BallCreator::~BallCreator()
	{

	}

	trimesh::TriMesh* BallCreator::create(float radius, unsigned int num_iter, int createUV)
	{
		trimesh::TriMesh* mesh = new trimesh::TriMesh();

		// golden ratio
		float phi = (sqrt(5.0f) + 1.0) * 0.5;

		// radius of circumscribes of the icosahedron
		//float radius_c = sqrt(phi + 2);

		// add verteces
		mesh->vertices.emplace_back(0.0f, phi, 1.0f);
		mesh->vertices.emplace_back(0.0f, phi, -1.0f);
		mesh->vertices.emplace_back(0.0f, -phi, 1.0f);
		mesh->vertices.emplace_back(0.0f, -phi, -1.0f);
		mesh->vertices.emplace_back(1.0f, 0.0f, phi);
		mesh->vertices.emplace_back(1.0f, 0.0f, -phi);
		mesh->vertices.emplace_back(-1.0f, 0.0f, phi);
		mesh->vertices.emplace_back(-1.0f, 0.0f, -phi);
		mesh->vertices.emplace_back(phi, 1.0f, 0.0);
		mesh->vertices.emplace_back(phi, -1.0f, 0.0);
		mesh->vertices.emplace_back(-phi, 1.0f, 0.0);
		mesh->vertices.emplace_back(-phi, -1.0f, 0.0);

		// add faces
		mesh->faces.emplace_back(0, 1, 10);
		mesh->faces.emplace_back(0, 8, 1);
		mesh->faces.emplace_back(0, 4, 8);
		mesh->faces.emplace_back(0, 6, 4);
		mesh->faces.emplace_back(0, 10, 6);
		mesh->faces.emplace_back(6, 10, 11);
		mesh->faces.emplace_back(11, 10, 7);
		mesh->faces.emplace_back(7, 10, 1);
		mesh->faces.emplace_back(1, 5, 7);
		mesh->faces.emplace_back(1, 8, 5);
		mesh->faces.emplace_back(5, 8, 9);
		mesh->faces.emplace_back(8, 4, 9);
		mesh->faces.emplace_back(4, 2, 9);
		mesh->faces.emplace_back(2, 4, 6);
		mesh->faces.emplace_back(6, 11, 2);
		mesh->faces.emplace_back(11, 3, 2);
		mesh->faces.emplace_back(3, 11, 7);
		mesh->faces.emplace_back(7, 5, 3);
		mesh->faces.emplace_back(3, 5, 9);
		mesh->faces.emplace_back(9, 2, 3);

		// split face into four small faces
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

		// project all vertecs to the sphere that radius equals argument <radius>
		for (size_t i = 0; i < mesh->vertices.size(); i++)
		{
			float scale = radius / trimesh::length(mesh->vertices[i]);
			mesh->vertices[i].x *= scale;
			mesh->vertices[i].y *= scale;
			mesh->vertices[i].z *= scale;
		}

		// create uv coords
		if (createUV)
		{
			for (size_t i = 0; i < mesh->vertices.size(); i++)
			{
				float alpha = atan2(mesh->vertices[i].y, mesh->vertices[i].x); // [-pi, pi]
				float beta = acos(mesh->vertices[i].z / radius);               // [0, pi]

				if (createUV == 1)
				{
					if (-3.0f * M_PI_4f <= alpha && alpha < -M_PI_4f)
					{
						mesh->cornerareas.emplace_back(0.5f - beta / (M_2PIf * tan(alpha)), 0.5f - beta / M_2PIf, 0.0f);
					}
					else if (-M_PI_4f <= alpha && alpha < M_PI_4f)
					{
						mesh->cornerareas.emplace_back(0.5f + beta / M_2PIf, 0.5f + beta * tan(alpha) / M_2PIf, 0.0f);
					}
					else if (M_PI_4f <= alpha && alpha < 3.0f * M_PI_4f)
					{
						mesh->cornerareas.emplace_back(0.5f + beta / (M_2PIf * tan(alpha)), 0.5f + beta / M_2PIf, 0.0f);
					}
					else
					{
						mesh->cornerareas.emplace_back(0.5f - beta / M_2PIf, 0.5f - beta * tan(alpha) / M_2PIf, 0.0f);
					}
				}
				else if (createUV == 2)
				{
					mesh->cornerareas.emplace_back(0.5f + beta * cos(alpha) / M_2PIf, 0.5f + beta * sin(alpha) / M_2PIf, 0.0f);
				}
				else if (createUV == 3)
				{
					mesh->cornerareas.emplace_back((alpha + M_PIf) / M_2PIf, (M_PIf - beta) / M_PIf, 0.0f);
				}
			}
		}

		//mesh->write("F:\\repos\\mesh_test\\data\\sphere_1.stl");

		return mesh;
	}
}