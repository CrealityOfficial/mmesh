#include "createcube.h"

namespace mmesh
{
	trimesh::TriMesh* createCube(const trimesh::box3& box, float gap)
	{
		trimesh::TriMesh* boxMesh = new trimesh::TriMesh();
		trimesh::point point0(box.min.x -gap, box.min.y -gap, box.min.z);
		trimesh::point point1(box.max.x + gap, box.min.y - gap, box.min.z);
		trimesh::point point2(box.max.x + gap, box.min.y - gap, box.max.z + gap);
		trimesh::point point3(box.min.x - gap, box.min.y - gap, box.max.z + gap);
		trimesh::point point4(box.min.x - gap, box.max.y + gap, box.min.z);
		trimesh::point point5(box.max.x + gap, box.max.y + gap, box.min.z);
		trimesh::point point6(box.max.x + gap, box.max.y + gap, box.max.z + gap);
		trimesh::point point7(box.min.x - gap, box.max.y + gap, box.max.z + gap);
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
		boxMesh->faces.push_back(trimesh::TriMesh::Face(0, 1, 5));
		//���� add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(5, 4, 0));

		return boxMesh;
	}
}