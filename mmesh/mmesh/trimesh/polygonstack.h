#ifndef CREATIVE_KERNEL_POLYGONSTACK_1592554738233_H
#define CREATIVE_KERNEL_POLYGONSTACK_1592554738233_H
#include "trimesh2/Vec.h"
#include "trimesh2/TriMesh.h"
#include <stack>

namespace mmesh
{
	class Polygon2;
	class PolygonStack
	{
	public:
		PolygonStack();
		~PolygonStack();

		void generates(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points, std::vector<trimesh::TriMesh::Face>& triangles);
		void generatesWithoutTree(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points, std::vector<trimesh::TriMesh::Face>& triangles);

		void prepareWithoutTree(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points);
		void prepare(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points);
		void generate(std::vector<trimesh::TriMesh::Face>& triangles);

		bool earClipping(trimesh::TriMesh::Face& face, std::vector<int>* earIndices = nullptr);
		void getEars(std::vector<int>* earIndices = nullptr);

		Polygon2* validIndexPolygon(int index);
		int validPolygon();
	protected:
		std::vector<Polygon2*> m_polygon2s;

		int m_currentPolygon;
	};
}
#endif // CREATIVE_KERNEL_POLYGONSTACK_1592554738233_H