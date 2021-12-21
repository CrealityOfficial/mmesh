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
		boxMesh->faces.push_back(trimesh::TriMesh::Face(0, 1, 5));
		//底面 add face 2
		boxMesh->faces.push_back(trimesh::TriMesh::Face(5, 4, 0));

		return boxMesh;
	}
}