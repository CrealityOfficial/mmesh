#include "createcylinder.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "trimesh2/TriMesh_algo.h"
#include "trimesh2/quaternion.h"
#define  PI 3.141592 

namespace mmesh
{

	trimesh::TriMesh* createCylinder(float _radius, float _height)
	{
		trimesh::TriMesh* cylinderMesh = new trimesh::TriMesh();
		int degrees = 4;
		int  trianglesCount = int(360 / degrees);//每4度一个三角形，一共有90个三角形
		float radian = degrees * (PI / 180.0);//4度换算成弧度
	
		for (int i=0;i< trianglesCount;i++)
		{
			//top face
			trimesh::point point0(0, 0, _height);
			trimesh::point point1(_radius * cos(i * radian), _radius * sin(i * radian), _height);
			trimesh::point point2(_radius * cos((i+1)* radian), _radius *sin((i+1)*radian), _height);
			int v = cylinderMesh->vertices.size();
			cylinderMesh->vertices.push_back(point0);
			cylinderMesh->vertices.push_back(point1);
			cylinderMesh->vertices.push_back(point2);
			cylinderMesh->faces.push_back(trimesh::TriMesh::Face(v, v + 1, v + 2));
			//Side 1a
			point0 = trimesh::point(_radius * cos(i * radian), _radius * sin(i * radian), _height);
			point1 = trimesh::point(_radius * cos(i * radian), _radius * sin(i * radian), 0);
			point2 = trimesh::point(_radius * cos((i + 1) * radian), _radius * sin((i + 1) * radian), _height);
			v = cylinderMesh->vertices.size();
			cylinderMesh->vertices.push_back(point0);
			cylinderMesh->vertices.push_back(point1);
			cylinderMesh->vertices.push_back(point2);
			cylinderMesh->faces.push_back(trimesh::TriMesh::Face(v, v + 1, v + 2));
			//#Side 1b
			point0 = trimesh::point(_radius * cos(i * radian), _radius * sin(i * radian), 0);
			point1 = trimesh::point(_radius * cos((i + 1) * radian), _radius * sin((i + 1) * radian), 0);
			point2 = trimesh::point(_radius * cos((i + 1) * radian), _radius * sin((i + 1) * radian), _height);
			v = cylinderMesh->vertices.size();
			cylinderMesh->vertices.push_back(point0);
			cylinderMesh->vertices.push_back(point1);
			cylinderMesh->vertices.push_back(point2);
			cylinderMesh->faces.push_back(trimesh::TriMesh::Face(v, v + 1, v + 2));
			//#Bottom 
			point0 = trimesh::point(0, 0, 0);
			point1 = trimesh::point(_radius * cos(i * radian), _radius * sin(i * radian), 0);
			point2 = trimesh::point(_radius * cos((i + 1) * radian), _radius * sin((i + 1) * radian), 0);
			v = cylinderMesh->vertices.size();
			cylinderMesh->vertices.push_back(point0);
			cylinderMesh->vertices.push_back(point1);
			cylinderMesh->vertices.push_back(point2);
			cylinderMesh->faces.push_back(trimesh::TriMesh::Face(v, v + 1, v + 2));
		}
		mmesh::dumplicateMesh(cylinderMesh);
		return cylinderMesh;
	}

	trimesh::TriMesh* createSoupCylinder(int count, float _radius, float _height)
	{
		int  trianglesCount = count;
		if (trianglesCount < 3)
			trianglesCount = 3;
		trimesh::TriMesh* cylinderMesh = new trimesh::TriMesh();
		float radian = 2.0f * PI / (float)trianglesCount;//4度换算成弧度

		trimesh::vec3 top(0.0f, 0.0f, _height / 2.0f);
		trimesh::vec3 bottom(0.0f, 0.0f, - _height / 2.0f);
		trimesh::vec3 offset(0.0f, 0.0f, _height);
		std::vector<trimesh::vec3> circles(trianglesCount, trimesh::vec3());
		for (int i = 0; i < trianglesCount; ++i)
			circles[i] = trimesh::vec3(_radius * cos(i * radian),
				_radius * sin(i * radian), -_height / 2.0f);
		circles.push_back(circles[0]);
		cylinderMesh->vertices.resize(12 * trianglesCount);
		int index = 0;

		auto f = [&index, &cylinderMesh](const trimesh::vec3& v1, const trimesh::vec3& v2, const trimesh::vec3& v3) {
			cylinderMesh->vertices[index++] = v1;
			cylinderMesh->vertices[index++] = v2;
			cylinderMesh->vertices[index++] = v3;
		};
		for (int i = 0; i < trianglesCount; ++i)
		{
			f(top, circles[i] + offset, circles[i + 1] + offset);
			f(circles[i] + offset, circles[i], circles[i + 1] + offset);
			f(circles[i + 1] + offset, circles[i], circles[i + 1]);
			f(bottom, circles[i + 1], circles[i]);
		}

		fillTriangleSoupFaceIndex(cylinderMesh);
		return cylinderMesh;
	}

	trimesh::TriMesh* createSoupCylinder(int count, float _radius, float _height,
		const trimesh::vec3& centerPoint, const trimesh::vec3& normal)
	{
		trimesh::TriMesh* mesh = createSoupCylinder(count, _radius, _height);

		const trimesh::vec3 cyOriginNormal(0.0f, 0.0f, 1.0f);
		trimesh::quaternion q = trimesh::quaternion::rotationTo(normal, cyOriginNormal);
		trimesh::fxform xf = trimesh::fxform::trans(centerPoint) * fromQuaterian(q);
		trimesh::apply_xform(mesh, trimesh::xform(xf));

		return mesh;
	}
}