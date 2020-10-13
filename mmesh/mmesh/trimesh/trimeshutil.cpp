#include "mmesh/trimesh/trimeshutil.h"
#include "trimesh2/TriMesh.h"

#include <unordered_map>
#include <assert.h>

namespace mmesh
{
	void mergeTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes)
	{
		assert(outMesh);
		size_t totalVertexSize = outMesh->vertices.size();
		size_t totalTriangleSize = outMesh->faces.size();
		size_t addVertexSize = 0;
		size_t addTriangleSize = 0;
		size_t meshSize = inMeshes.size();
		for (size_t i = 0; i < meshSize; ++i)
		{
			addVertexSize += inMeshes.at(i)->vertices.size();
			addTriangleSize += inMeshes.at(i)->faces.size();
		}
		totalVertexSize += addVertexSize;
		totalTriangleSize += addTriangleSize;

		if (addVertexSize > 0 && addTriangleSize > 0)
		{
			outMesh->vertices.reserve(totalVertexSize);
			outMesh->faces.reserve(totalTriangleSize);

			int startFaceIndex = outMesh->faces.size();
			int startVertexIndex = outMesh->vertices.size();;
			for (size_t i = 0; i < meshSize; ++i)
			{
				trimesh::TriMesh* mesh = inMeshes.at(i);
				int vertexNum = (int)mesh->vertices.size();
				int faceNum = (int)mesh->faces.size();
				if (vertexNum > 0 && faceNum > 0)
				{
					outMesh->vertices.insert(outMesh->vertices.end(), mesh->vertices.begin(), mesh->vertices.end());
					outMesh->faces.insert(outMesh->faces.end(), mesh->faces.begin(), mesh->faces.end());

					int endFaceIndex = startFaceIndex + faceNum;
					if (startVertexIndex > 0)
					{
						for (int ii = startFaceIndex; ii < endFaceIndex; ++ii)
						{
							trimesh::TriMesh::Face& face = outMesh->faces.at(ii);
							for (int j = 0; j < 3; ++j)
								face[j] += startVertexIndex;
						}
					}

					startFaceIndex += faceNum;
					startVertexIndex += vertexNum;
				}
			}
		}
	}

	void reverseTriMesh(trimesh::TriMesh* Mesh)
	{
		for (size_t i = 0; i < Mesh->faces.size(); i++)
		{
			int temp = Mesh->faces[i].at(1);
			Mesh->faces[i].at(1) = Mesh->faces[i].at(2);
			Mesh->faces[i].at(2) = temp;
		}
	}

	void mergeTrianglesTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes)
	{
		int totalVertexSize = 0;
		size_t meshSize = inMeshes.size();
		for (size_t i = 0; i < meshSize; ++i)
		{
			trimesh::TriMesh* mesh = inMeshes.at(i);
			if (mesh->faces.size() == 0)
				totalVertexSize += (int)mesh->vertices.size();
			else
				totalVertexSize += mesh->vertices.size();
		}

		if (totalVertexSize > 0)
		{
			outMesh->vertices.reserve(totalVertexSize);
			for (size_t i = 0; i < meshSize; ++i)
			{
				trimesh::TriMesh* mesh = inMeshes.at(i);
				if (mesh->faces.size() == 0)
				{
					outMesh->vertices.insert(outMesh->vertices.end(), mesh->vertices.begin(), mesh->vertices.end());
				}
				else
				{
					size_t fnum = mesh->faces.size();
					if (fnum > 0)
					{
						for (size_t ii = 0; ii < fnum; ++ii)
						{
							trimesh::TriMesh::Face& face = mesh->faces.at(ii);
							for (int j = 0; j < 3; ++j)
							{
								int index = face[j];
								outMesh->vertices.push_back(mesh->vertices.at(index));
							}
						}
					}
				}
			}
		}
	}

	trimesh::TriMesh* partMesh(const std::vector<int>& indices, trimesh::TriMesh* inMesh)
	{
		trimesh::TriMesh* outMesh = nullptr;
		if (inMesh && indices.size() > 0 && inMesh->vertices.size() > 0)
		{
			size_t vertexSize = inMesh->vertices.size();
			outMesh = new trimesh::TriMesh();
			outMesh->faces.resize(indices.size());
			outMesh->vertices.reserve(3 * indices.size());

			std::vector<int> flags(vertexSize, -1);
			int startIndex = 0;
			int k = 0;
			for (int i : indices)
			{
				trimesh::TriMesh::Face& f = inMesh->faces.at(i);
				trimesh::TriMesh::Face& of = outMesh->faces.at(k);
				for (int j = 0; j < 3; ++j)
				{
					int index = f[j];
					if (flags.at(index) < 0)
					{
						flags.at(index) = startIndex++;
						outMesh->vertices.push_back(inMesh->vertices.at(index));
					}

					of[j] = flags.at(index);
				}

				++k;
			}
		}

		return outMesh;
	}

	void dumplicateMesh(trimesh::TriMesh* mesh)
	{
		if (!mesh)
			return;

		size_t vertexNum = mesh->vertices.size();

		struct hash_vec3
		{
			size_t operator()(const trimesh::vec3& v)const
			{
				return abs(v.x) * 10000 / 23 + abs(v.y) * 10000 / 19 + abs(v.z) * 10000 / 17;
			}
		};
		struct equal_vec3
		{
			bool operator()(const trimesh::vec3& v1, const trimesh::vec3& v2) const
			{
				return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
			}
		};
		typedef std::unordered_map<trimesh::vec3, int, hash_vec3, equal_vec3> unique_point;
		unique_point points(vertexNum * 3 / 10 + 1);

		typedef unique_point::iterator point_iterator;

		size_t faceNum = mesh->faces.size();

		if (vertexNum == 0 || faceNum == 0)
			return;

		trimesh::TriMesh* optimizeMesh = new trimesh::TriMesh();

		std::vector<int> vertexMapper;
		vertexMapper.resize(vertexNum, -1);

		for (size_t i = 0; i < vertexNum; ++i)
		{
			trimesh::point p = mesh->vertices.at(i);
			point_iterator it = points.find(p);
			if (it != points.end())
			{
				int index = (*it).second;
				vertexMapper.at(i) = index;
			}
			else
			{
				int index = (int)points.size();
				points.insert(unique_point::value_type(p, index));

				vertexMapper.at(i) = index;
			}
		}

		trimesh::TriMesh* omesh = optimizeMesh;
		omesh->vertices.resize(points.size());
		for (point_iterator it = points.begin(); it != points.end(); ++it)
		{
			omesh->vertices.at(it->second) = it->first;
		}

		omesh->faces = mesh->faces;
		for (size_t i = 0; i < faceNum; ++i)
		{
			trimesh::TriMesh::Face& of = omesh->faces.at(i);
			for (int j = 0; j < 3; ++j)
			{
				int index = of[j];
				of[j] = vertexMapper[index];
			}
		}

		mesh->vertices.swap(omesh->vertices);
		mesh->faces.swap(omesh->faces);
		mesh->need_bbox();

		delete omesh;
	}
}