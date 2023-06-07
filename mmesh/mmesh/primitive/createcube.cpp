#include "createcube.h"

namespace mmesh
{
	trimesh::TriMesh* createCuboid(float x, float y, float z)
	{
		trimesh::TriMesh* boxMesh = new trimesh::TriMesh();
		trimesh::vec v0(-x / 2., -y / 2., -z / 2.);
		trimesh::vec v1(x / 2., y / 2., z / 2.);
		trimesh::box3 bb(v0,v1);
		trimesh::point point0(bb.min[0], bb.min[1], bb.min[2]);
		trimesh::point point1(bb.max[0], bb.min[1], bb.min[2]);
		trimesh::point point2(bb.max[0], bb.min[1], bb.max[2]);
		trimesh::point point3(bb.min[0], bb.min[1], bb.max[2]);
		trimesh::point point4(bb.min[0], bb.max[1], bb.min[2]);
		trimesh::point point5(bb.max[0], bb.max[1], bb.min[2]);
		trimesh::point point6(bb.max[0], bb.max[1], bb.max[2]);
		trimesh::point point7(bb.min[0], bb.max[1], bb.max[2]);
		boxMesh->vertices.push_back(point0);
		boxMesh->vertices.push_back(point1);
		boxMesh->vertices.push_back(point2);
		boxMesh->vertices.push_back(point3);
		boxMesh->vertices.push_back(point4);
		boxMesh->vertices.push_back(point5);
		boxMesh->vertices.push_back(point6);
		boxMesh->vertices.push_back(point7);

		//正面add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(0, 1, 2));
		//正面 add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(2, 3, 0));

		//右侧面add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(1, 5, 6));
		//右侧面 add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(6, 2, 1));

		//背面add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(4, 6, 5));
		//背面 add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(6, 4, 7));

		//左侧面add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(4, 0, 3));
		//左侧面 add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(3, 7, 4));

		//顶面add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(3, 2, 6));
		//顶面 add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(6, 7, 3));

		//底面add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(5, 1, 0));
		//底面 add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(5, 0, 4));

		return boxMesh;
	}

}