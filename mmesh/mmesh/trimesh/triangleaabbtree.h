#ifndef CREATIVE_KERNEL_TRIANGLEAABBTREE_1593566614758_H
#define CREATIVE_KERNEL_TRIANGLEAABBTREE_1593566614758_H
#include "mmesh/trimesh/triangleaabbnode.h"
#include "trimesh2/TriMesh.h"
#include "trimesh2/XForm.h"

namespace mmesh
{

	struct ZQueryResult
	{
		trimesh::vec3 p;  // global position
		int index;  // triangle index

		trimesh::vec3 n;
	};

	class TriangleAABBTree
	{
	public:
		TriangleAABBTree();
		virtual ~TriangleAABBTree();

		void build(trimesh::TriMesh* mesh, trimesh::fxform& xf);
		void clear();

		void zQuery(trimesh::vec3& p, std::vector<ZQueryResult>& collides);  // global position
	protected:
		void lineQuery(trimesh::vec3& p, trimesh::vec3& n, std::vector<int>& indices);
	private:
		void build();
		void clearNodes();
	protected:
		trimesh::TriMesh* m_mesh;
		trimesh::fxform m_xf;

		std::vector<TriangleAABBNode> m_nodes;
	};

}
#endif // CREATIVE_KERNEL_TRIANGLEAABBTREE_1593566614758_H