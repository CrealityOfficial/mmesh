#ifndef CREATIVE_KERNEL_POLYGON_1592554738233_H
#define CREATIVE_KERNEL_POLYGON_1592554738233_H
#include "trimesh2/Vec.h"
#include "trimesh2/TriMesh.h"

#include <list>

namespace mmesh
{
	struct VNode
	{
		VNode* prev;
		VNode* next;

		int index;
		int type;  // 0 colliear 1 concave 2 convex 3 ear
		double dot;
	};

	class Polygon2
	{
	public:
		Polygon2();
		~Polygon2();

		void setup(const std::vector<int>& polygon, std::vector<trimesh::dvec2>& points);
		std::vector<int> toIndices();

		void earClipping(std::vector<trimesh::TriMesh::Face>& triangles);

		bool earClipping(trimesh::TriMesh::Face& face, std::vector<int>* earIndices = nullptr);
		void getEars(std::vector<int>* earIndices = nullptr);
	protected:
		trimesh::TriMesh::Face nodeTriangle(VNode* node);
		void calType(VNode* node, bool testContainEdge = true);

		void releaseNode();
	protected:
		bool m_close;
		std::vector<int> m_indexes;
		std::vector<trimesh::dvec2>* m_points;

		VNode* m_root;
		int m_circleSize;
		std::list<VNode*> m_ears;

#ifdef _DEBUG
		std::vector<VNode*> m_debugNodes;
#endif
	};
}
#endif // CREATIVE_KERNEL_POLYGON_1592554738233_H