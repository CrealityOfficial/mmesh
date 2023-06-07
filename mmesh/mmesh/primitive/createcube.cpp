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

		//����add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(0, 1, 2));
		//���� add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(2, 3, 0));

		//�Ҳ���add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(1, 5, 6));
		//�Ҳ��� add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(6, 2, 1));

		//����add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(4, 6, 5));
		//���� add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(6, 4, 7));

		//�����add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(4, 0, 3));
		//����� add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(3, 7, 4));

		//����add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(3, 2, 6));
		//���� add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(6, 7, 3));

		//����add face 1
		boxMesh->faces.push_back(trimesh::TriMesh::Face(5, 1, 0));
		//���� add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(5, 0, 4));

		return boxMesh;
	}

}