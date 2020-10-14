#ifndef CREATIVE_KERNEL_TRIANGLEAABBNODE_1593566614758_H
#define CREATIVE_KERNEL_TRIANGLEAABBNODE_1593566614758_H
#include "trimesh2/Box.h"
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	class TriangleAABBNode
	{
	public:
		TriangleAABBNode();
		virtual ~TriangleAABBNode();

		const trimesh::box3& box();
		void expand(int* first, int* beyond, size_t range, std::vector<trimesh::box3>& boxMap);
		void traversal(trimesh::vec3& p, trimesh::vec3& n, size_t range, std::vector<int>& indices);

		void ztraversal(trimesh::vec3& p, size_t range, std::vector<int>& indices);
	private:
		TriangleAABBNode& left();
		TriangleAABBNode& right();
		int lData();
		int rData();

		trimesh::box3 calBox(int* first, int* beyond, std::vector<trimesh::box3>& boxMap);
		void sortPrimitive(int* first, int* beyond, std::vector<trimesh::box3>& boxMap);
		bool intersect(trimesh::vec3& p, trimesh::vec3& n);
		bool zintersect(trimesh::vec3& p);
	protected:
		trimesh::box3 m_box;
		void* m_left;
		void* m_right;
	};
}

#endif // CREATIVE_KERNEL_TRIANGLEAABBNODE_1593566614758_H
