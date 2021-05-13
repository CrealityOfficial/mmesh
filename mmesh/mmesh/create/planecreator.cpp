#include "planecreator.h"

namespace mmesh
{
	trimesh::TriMesh* defaultPlane(int width, int height, float pixel, float h, bool createUV)
	{
		if (width <= 0 || height <= 0 || pixel <= 0.0f)
			return nullptr;

		trimesh::TriMesh* mesh = new trimesh::TriMesh();
		int vertexNum = (width + 1) * (height + 1) + 2 * (width + height);
		int triangleNum = 2 * width * height + 4 * (width + height) + 2 * width;
		mesh->vertices.resize(vertexNum);
		mesh->normals.resize(vertexNum);
		mesh->faces.resize(triangleNum);

		for (int i = 0; i <= height; ++i)
		{
			int lineIndex = i * (width + 1);
			float y = (float)i * pixel;
			for (int j = 0; j <= width; ++j)
			{
				int index = lineIndex + j;
				float x = (float)j * pixel;
				mesh->normals.at(index) = trimesh::vec3(0.0f, 0.0f, 1.0f);
				mesh->vertices.at(index) = trimesh::vec3(x, y, 0.0f);
			}
		}

		int triIndex = 0;
		for (int i = 0; i < height; ++i)
		{
			int lineIndex = i * (width + 1);
			for (int j = 0; j < width; ++j)
			{
				int v0 = lineIndex + j;
				int v1 = v0 + 1;
				int v2 = v1 + width + 1;
				int v3 = v0 + width + 1;

				trimesh::TriMesh::Face& f1 = mesh->faces.at(triIndex++);
				trimesh::TriMesh::Face& f2 = mesh->faces.at(triIndex++);
				f1.x = v0; f1.y = v1; f1.z = v2;
				f2.x = v0; f2.y = v2; f2.z = v3;
			}
		}



		///////
		int bottomStart = (width + 1) * (height + 1);
		int vindex = bottomStart;
		{
			float y1 = 0.0f;
			for (int i = 0; i < width; ++i)
			{
				int v0 = vindex;
				int v1 = v0 + 1;
				int v2 = i + 1;
				int v3 = i;

				trimesh::TriMesh::Face& f1 = mesh->faces.at(triIndex++);
				trimesh::TriMesh::Face& f2 = mesh->faces.at(triIndex++);
				f1.x = v0; f1.y = v1; f1.z = v2;
				f2.x = v0; f2.y = v2; f2.z = v3;

				float x = (float)i * pixel;
				mesh->vertices.at(vindex++) = trimesh::vec3(x, y1, -h);
			}
		}

		{
			float x = (float)width * pixel;

			for (int i = 0; i < height; ++i)
			{
				int v0 = vindex;
				int v1 = v0 + 1;
				int v2 = (i + 2) * (width + 1) - 1;
				int v3 = (i + 1) * (width + 1) - 1;

				trimesh::TriMesh::Face& f1 = mesh->faces.at(triIndex++);
				trimesh::TriMesh::Face& f2 = mesh->faces.at(triIndex++);
				f1.x = v0; f1.y = v1; f1.z = v2;
				f2.x = v0; f2.y = v2; f2.z = v3;

				float y = (float)i * pixel;
				mesh->vertices.at(vindex++) = trimesh::vec3(x, y, -h);
			}
		}

		int topStart = vindex;
		{
			float y = (float)height * pixel;
			float tx = (float)width * pixel;
			for (int i = 0; i < width; ++i)
			{
				int v0 = vindex;
				int v1 = v0 + 1;
				int v3 = (width + 1) * height + width - i;
				int v2 = v3 - 1;

				trimesh::TriMesh::Face& f1 = mesh->faces.at(triIndex++);
				trimesh::TriMesh::Face& f2 = mesh->faces.at(triIndex++);
				f1.x = v0; f1.y = v1; f1.z = v2;
				f2.x = v0; f2.y = v2; f2.z = v3;

				float x = tx - (float)i * pixel;
				mesh->vertices.at(vindex++) = trimesh::vec3(x, y, -h);
			}
		}

		{
			float x = 0.0f;
			float ty = (float)height * pixel;
			for (int i = 0; i < height; ++i)
			{
				int v0 = vindex;
				int v1 = v0 + 1;
				if (i == height - 1)
					v1 = bottomStart;

				int v2 = (height - i - 1) * (width + 1);
				int v3 = (height - i) * (width + 1);

				trimesh::TriMesh::Face& f1 = mesh->faces.at(triIndex++);
				trimesh::TriMesh::Face& f2 = mesh->faces.at(triIndex++);
				f1.x = v0; f1.y = v1; f1.z = v2;
				f2.x = v0; f2.y = v2; f2.z = v3;

				float y = ty - (float)i * pixel;
				mesh->vertices.at(vindex++) = trimesh::vec3(x, y, -h);
			}
		}


		{
			for (int i = 0; i < width; ++i)
			{
				int v0 = topStart + (width - i);
				int v1 = v0 - 1;
				int v3 = bottomStart + i;
				int v2 = v3 + 1;

				trimesh::TriMesh::Face& f1 = mesh->faces.at(triIndex++);
				trimesh::TriMesh::Face& f2 = mesh->faces.at(triIndex++);
				f1.x = v0; f1.y = v1; f1.z = v2;
				f2.x = v0; f2.y = v2; f2.z = v3;
			}
		}

		mesh->need_bbox();

		if (createUV)
		{
			mesh->cornerareas.resize(vertexNum, trimesh::vec3(0.0f, 0.0f, 0.0f));

			float dx = 1.0f / (float)width;
			float dy = 1.0f / (float)height;
			for (int i = 0; i <= height; ++i)
			{
				int lineIndex = i * (width + 1);
				float y = (float)i * dy;
				for (int j = 0; j <= width; ++j)
				{
					int index = lineIndex + j;
					float x = (float)j * dx;
					mesh->cornerareas.at(index) = trimesh::vec3(x, y, 0.0f);
				}
			}
		}
		return mesh;
	}
}