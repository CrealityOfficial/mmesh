#include "createcylinder.h"
#include "mmesh/trimesh/trimeshutil.h"
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

}