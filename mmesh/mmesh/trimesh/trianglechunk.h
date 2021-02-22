#ifndef CREATIVE_KERNEL_TRIANGLECHUNK_1595984973500_H
#define CREATIVE_KERNEL_TRIANGLECHUNK_1595984973500_H
#include <vector>

#include "trimesh2/Vec.h"
#include "trimesh2/Box.h"

namespace mmesh
{
	class TriangleChunk
	{
	public:
		TriangleChunk();
		~TriangleChunk();

		void build(std::vector<trimesh::box3>& boxes, trimesh::box3& globalBox, float gridSize);
		void clear();

		std::vector<int>& cell(trimesh::ivec2& idx);

		inline trimesh::ivec2 index(trimesh::vec3& point)
		{
			int nx = (int)floorf((point.x - m_globalBox.min.x) / m_gridSize);
			int ny = (int)floorf((point.y - m_globalBox.min.y) / m_gridSize);
			return trimesh::ivec2(nx, ny);
		}

		inline trimesh::ivec4 index(trimesh::box3& box)
		{
			trimesh::ivec2 imin = index(box.min);
			trimesh::ivec2 imax = index(box.max);
			return trimesh::ivec4(imin.x, imin.y, imax.x, imax.y);
		}

		inline bool contain(trimesh::vec3& point)
		{
			trimesh::ivec2 idx = index(point);
			return contain(idx);
		}

		inline bool contain(trimesh::ivec2& idx)
		{
			return idx.x >= 0 && idx.x < m_width&& idx.y >= 0 && idx.y < m_height;
		}

	public:
		trimesh::box3 m_globalBox;

		int m_width;
		int m_height;
		float m_gridSize;

		std::vector<std::vector<int>> m_cells;//存储XY栅格化后模型面投影在栅格中的索引值
	};
}
#endif // CREATIVE_KERNEL_TRIANGLECHUNK_1595984973500_H