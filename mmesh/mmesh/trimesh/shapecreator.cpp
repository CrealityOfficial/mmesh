#include "shapecreator.h"
#include "trimesh2/quaternion.h"
#include <cmath>

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

	ShapeCreator::ShapeCreator()
	{
	}

	ShapeCreator::~ShapeCreator()
	{
	}

	trimesh::TriMesh* ShapeCreator::createCuboidMesh(trimesh::vec3 size)
	{
		trimesh::TriMesh* mesh = new trimesh::TriMesh();
		//QFile file(":/support/mesh/cube.stl");
		//if(!file.open(QFile::ReadOnly))
		//{
		//    return NULL;
		//}
		//file.seek(0);
		//char header[80];
		//file.read(header, 80);
		//
		//uint32_t faceCount(0);
		//file.read((char*)&faceCount, sizeof(uint32_t));
		//
		//mesh->faces.reserve(faceCount);
		//mesh->vertices.reserve(3 * faceCount);
		//for (int i = 0; i < faceCount; i++)
		//{
		//    float fbuf[12];
		//    file.read((char*)fbuf, 48);
		//    int v = mesh->vertices.size();
		//    mesh->vertices.push_back(trimesh::point(fbuf[3], fbuf[4], fbuf[5]));
		//    mesh->vertices.push_back(trimesh::point(fbuf[6], fbuf[7], fbuf[8]));
		//    mesh->vertices.push_back(trimesh::point(fbuf[9], fbuf[10], fbuf[11]));
		//    mesh->faces.push_back(trimesh::TriMesh::Face(v, v + 1, v + 2));
		//    unsigned char att[2];
		//    file.read((char*)att, 2);
		//}
		//
		//mesh->need_bbox();
		//
		//file.close();
		mesh->vertices.resize(8);
		mesh->faces.resize(12);
		memcpy(&mesh->vertices.at(0), static_box_position, 24 * sizeof(float));
		memcpy(&mesh->faces.at(0), static_box_triangles_indices, 36 * sizeof(unsigned));

		mesh->need_normals();
		mesh->need_bbox();

		return mesh;

	}

	trimesh::TriMesh* ShapeCreator::createCuboidMesh(trimesh::vec3 bottom, trimesh::vec3 top, int r, float radius, float startTheta)
	{
		trimesh::TriMesh* mesh = new trimesh::TriMesh();
		trimesh::vec3 start = bottom;
		trimesh::vec3 end = top;

		trimesh::vec3 dir = end - start;
		//dir.normalize();
		trimesh::normalize(dir);
		trimesh::quaternion q = trimesh::quaternion::fromDirection(dir, trimesh::vec3(0.0f, 0.0f, 1.0f));

		float deltaTheta = 2.0f * 3.1415926f / (float)r;
		std::vector<float> cosValue;
		std::vector<float> sinValue;
		for (int i = 0; i < r; ++i)
		{
			//cosValue.push_back(qCos(deltaTheta * (float)i + startTheta));
			//sinValue.push_back(qSin(deltaTheta * (float)i + startTheta));

			cosValue.push_back(std::cos(deltaTheta * (float)i + startTheta));
			sinValue.push_back(std::sin(deltaTheta * (float)i + startTheta));
		}

		std::vector<trimesh::vec3> baseNormals;
		for (int i = 0; i < r; ++i)
		{
			baseNormals.push_back(trimesh::vec3(cosValue[i], sinValue[i], 0.0f));
		}

		unsigned vertexNum = 2 * r;
		mesh->vertices.resize(vertexNum);
		unsigned triangleNum = 2 * r;
		mesh->faces.resize(triangleNum + 2 * (r - 2));

		unsigned vertexIndex = 0;
		unsigned triangleIndex = 0;
		for (int i = 0; i < r; ++i)
		{
			trimesh::vec3 n = q * baseNormals[i];
			trimesh::vec3 s = start + n * radius;
			mesh->vertices.at(vertexIndex) = trimesh::vec3(s.x, s.y, s.z);
			++vertexIndex;
			trimesh::vec3 e = end + n * radius;
			mesh->vertices.at(vertexIndex) = trimesh::vec3(e.x, e.y, e.z);
			++vertexIndex;
		}

		for (int i = 0; i < r; ++i)
		{
			trimesh::TriMesh::Face f1(2 * i, ((2 * i + 3) % (2 * r)), 2 * i + 1);
			mesh->faces.at(triangleIndex) = f1;
			++triangleIndex;

			trimesh::TriMesh::Face f2(2 * i, ((2 * i + 2) % (2 * r)), ((2 * i + 3) % (2 * r)));
			mesh->faces.at(triangleIndex) = f2;
			++triangleIndex;
		}

		for (int i = 1; i < r - 1; ++i)
		{
			trimesh::TriMesh::Face f1(0, 2 * (i + 1), 2 * i);
			mesh->faces.at(triangleIndex) = f1;
			++triangleIndex;
		}

		for (int i = 1; i < r - 1; ++i)
		{
			trimesh::TriMesh::Face f1(1, 2 * i + 1, 2 * (i + 1) + 1);
			mesh->faces.at(triangleIndex) = f1;
			++triangleIndex;
		}

		return mesh;
	}

	trimesh::TriMesh* ShapeCreator::createCuboidMesh(std::vector<mmesh::VerticalSeg>& segments, std::vector<int>& chunks, trimesh::fxform& matrix, int nn, float radius, float startTheta)
	{
		trimesh::TriMesh* mesh = new trimesh::TriMesh();

		//QMatrix4x4 imatrix = matrix.inverted();

		trimesh::invert(matrix);
		//trimesh::quaternion imatrix = matrix;

		unsigned r = nn;
		float deltaTheta = 2.0f * 3.1415926f / (float)r;
		std::vector<float> cosValue;
		std::vector<float> sinValue;
		for (unsigned i = 0; i < r; ++i)
		{
			//cosValue.push_back(qCos(deltaTheta * (float)i + startTheta));
			//sinValue.push_back(qSin(deltaTheta * (float)i + startTheta));

			cosValue.push_back(std::cos(deltaTheta * (float)i + startTheta));
			sinValue.push_back(std::sin(deltaTheta * (float)i + startTheta));
		}

		std::vector<trimesh::vec3> baseNormals;
		for (unsigned i = 0; i < r; ++i)
		{
			baseNormals.push_back(trimesh::vec3(cosValue[i], sinValue[i], 0.0f));
		}

		int count = (int)segments.size();
		unsigned vertexNum = count * 2 * r;
		mesh->vertices.resize(vertexNum);
		int chunkSize = (2 * r + 2 * (r - 2));
		unsigned triangleNum = count * chunkSize;
		mesh->faces.resize(triangleNum);
		chunks.resize(count);
		for (int i = 0; i < count; ++i)
		{
			chunks.at(i) = chunkSize * i;
		}

		unsigned vertexIndex = 0;
		unsigned triangleIndex = 0;

		for (mmesh::VerticalSeg& verticalSeg : segments)
		{
			unsigned startVertexIndex = vertexIndex;
			unsigned startTriangleIndex = triangleIndex;

			trimesh::vec3 start = matrix * trimesh::vec3(verticalSeg.b.x, verticalSeg.b.y, verticalSeg.b.z);
			trimesh::vec3 end = matrix * trimesh::vec3(verticalSeg.t.x, verticalSeg.t.y, verticalSeg.t.z);

			trimesh::vec3 dir = end - start;
			//dir.normalize();
			trimesh::normalize(dir);
			trimesh::quaternion q = trimesh::quaternion::fromDirection(dir, trimesh::vec3(0.0f, 0.0f, 1.0f));
			for (unsigned i = 0; i < r; ++i)
			{
				trimesh::vec3 n = q * baseNormals[i];
				trimesh::vec3 s = start + n * radius;

				trimesh::vec3& v1 = mesh->vertices.at(startVertexIndex);
				v1[0] = s.x;
				v1[1] = s.y;
				v1[2] = s.z;

				++startVertexIndex;
				trimesh::vec3 e = end + n * radius;

				trimesh::vec3& v2 = mesh->vertices.at(startVertexIndex);
				v2[0] = e.x;
				v2[1] = e.y;
				v2[2] = e.z;

				++startVertexIndex;
			}
			unsigned triIndex = startTriangleIndex;
			for (unsigned i = 0; i < r; ++i)
			{
				trimesh::TriMesh::Face& f1 = mesh->faces.at(triIndex++);
				f1.at(0) = vertexIndex + 2 * i;
				f1.at(2) = vertexIndex + 2 * i + 1;
				f1.at(1) = vertexIndex + ((2 * i + 3) % (2 * r));
				trimesh::TriMesh::Face& f2 = mesh->faces.at(triIndex++);
				f2.at(0) = vertexIndex + 2 * i;
				f2.at(2) = vertexIndex + ((2 * i + 3) % (2 * r));
				f2.at(1) = vertexIndex + ((2 * i + 2) % (2 * r));
			}
			for (int i = 1; i < r - 1; ++i)
			{
				trimesh::TriMesh::Face f1(vertexIndex, vertexIndex + 2 * (i + 1), vertexIndex + 2 * i);
				mesh->faces.at(triIndex) = f1;
				++triIndex;
			}

			for (int i = 1; i < r - 1; ++i)
			{
				trimesh::TriMesh::Face f1(vertexIndex + 1, vertexIndex + 2 * i + 1, vertexIndex + 2 * (i + 1) + 1);
				mesh->faces.at(triIndex) = f1;
				++triIndex;
			}

			vertexIndex += 2 * r;
			triangleIndex += chunkSize;
		}

		return mesh;
	}

	trimesh::TriMesh* ShapeCreator::createCylinderMeshFromCenter(const trimesh::vec3& position, const trimesh::vec3& normal,
		float depth, float radius, int num, float theta)
	{
		trimesh::vec3 _normal = trimesh::normalized(normal);

		trimesh::vec3 bottom = position + 2.0f * _normal;
		trimesh::vec3 top = position - (depth + 2.0f) * _normal;

		return createCylinderMesh(top, bottom, radius, num);
	}

	trimesh::TriMesh* ShapeCreator::createCylinderMesh(const trimesh::vec3& top, const trimesh::vec3& bottom, float radius, int num, float theta)
	{
		trimesh::TriMesh* mesh = new trimesh::TriMesh();

		int hPart = num;

		trimesh::vec3 start = bottom;
		trimesh::vec3 end = top;

		trimesh::vec3 dir = end - start;
		//dir.normalize();
		trimesh::normalize(dir);
		trimesh::quaternion q = trimesh::quaternion::fromDirection(dir, trimesh::vec3(0.0f, 0.0f, 1.0f));

		theta *= M_PIf / 180.0f;
		float deltaTheta = M_PIf * 2.0f / (float)(hPart);
		std::vector<float> cosValue;
		std::vector<float> sinValue;
		for (int i = 0; i < hPart; ++i)
		{
			//cosValue.push_back(qCos(deltaTheta * (float)i + theta));
			//sinValue.push_back(qSin(deltaTheta * (float)i + theta));

			cosValue.push_back(std::cos(deltaTheta * (float)i + theta));
			sinValue.push_back(std::sin(deltaTheta * (float)i + theta));
		}

		std::vector<trimesh::vec3> baseNormals;
		for (int i = 0; i < hPart; ++i)
		{
			baseNormals.push_back(trimesh::vec3(cosValue[i], sinValue[i], 0.0f));
		}

		int vertexNum = 2 * hPart;
		std::vector<trimesh::vec3> points(vertexNum);
		int faceNum = 4 * hPart - 4;
		mesh->faces.resize(faceNum);

		int vertexIndex = 0;
		for (int i = 0; i < hPart; ++i)
		{
			trimesh::vec3 n = q * baseNormals[i];
			trimesh::vec3 s = start + n * radius;
			points.at(vertexIndex) = trimesh::vec3(s.x, s.y, s.z);
			++vertexIndex;
			trimesh::vec3 e = end + n * radius;
			points.at(vertexIndex) = trimesh::vec3(e.x, e.y, e.z);
			++vertexIndex;
		}
		mesh->vertices.swap(points);

		auto fvindex = [&hPart](int layer, int index)->int {
			return layer + 2 * index;
		};

		int faceIndex = 0;
		for (int i = 0; i < hPart; ++i)
		{
			int v1 = fvindex(0, i);
			int v2 = fvindex(1, i);
			int v3 = fvindex(0, (i + 1) % hPart);
			int v4 = fvindex(1, (i + 1) % hPart);

			trimesh::TriMesh::Face& f1 = mesh->faces.at(faceIndex);
			f1[0] = v1;
			f1[1] = v3;
			f1[2] = v2;
			++faceIndex;
			trimesh::TriMesh::Face& f2 = mesh->faces.at(faceIndex);
			f2[0] = v2;
			f2[1] = v3;
			f2[2] = v4;
			++faceIndex;
		}

		for (int i = 1; i < hPart - 1; ++i)
		{
			trimesh::TriMesh::Face& f1 = mesh->faces.at(faceIndex);
			f1[0] = 0;
			f1[1] = fvindex(0, i + 1);
			f1[2] = fvindex(0, i);
			++faceIndex;
		}

		for (int i = 1; i < hPart - 1; ++i)
		{
			trimesh::TriMesh::Face& f1 = mesh->faces.at(faceIndex);
			f1[0] = 1;
			f1[1] = fvindex(1, i);
			f1[2] = fvindex(1, i + 1);
			++faceIndex;
		}

		return mesh;
	}

	trimesh::TriMesh* ShapeCreator::createPyramidMesh(int nbBottomVertex, float bottomRadius, float height)
	{
		if (nbBottomVertex < 3 || bottomRadius < 0.0f || height < 0.0f)
		{
			return nullptr;
		}

		trimesh::TriMesh* pyramid = new trimesh::TriMesh();

		pyramid->vertices.emplace_back(0.0f, 0.0f, 0.0f);
		float theta = 0.0f;
		for (int i = 0; i < nbBottomVertex; i++)
		{
			pyramid->vertices.emplace_back(cosf(theta) * bottomRadius, sinf(theta) * bottomRadius, 0.0f);
			theta += M_2PI / nbBottomVertex;
		}
		pyramid->vertices.emplace_back(0.0f, 0.0f, height);

		int next = 0;
		for (int i = 1; i <= nbBottomVertex; i++)
		{
			next = i + 1;
			if (next > nbBottomVertex)
			{
				next = next % nbBottomVertex;
			}
			pyramid->faces.emplace_back(0, next, i);
			pyramid->faces.emplace_back(nbBottomVertex + 1, i, next);
		}

		return pyramid;
	}
}