#include "mmesh/trimesh/trianglechunk.h"

using namespace trimesh;
namespace mmesh
{
	TriangleChunk::TriangleChunk()
		: m_gridSize(2.0f)
		, m_width(0)
		, m_height(0)
	{
	}
	
	TriangleChunk::~TriangleChunk()
	{
	}

	void TriangleChunk::build(std::vector<box3>& boxes, box3& globalBox, float gridSize)
	{
		clear();

		m_gridSize = gridSize;
		m_globalBox = globalBox;

		m_cells.clear();
		int faceNum = (int)boxes.size();

		vec3 globalSize = m_globalBox.size();
		m_width = (int)(ceilf(globalSize.x / m_gridSize) + 1);
		m_height = (int)(ceilf(globalSize.y / m_gridSize) + 1);

		if (m_width > 0 && m_height > 0)
			m_cells.resize(m_width * m_height);

		for (size_t i = 0; i < faceNum; ++i)
		{
			ivec4 idx = index(boxes.at(i));
			for (int x = idx[0]; x <= idx[2]; ++x)
			{
				for (int y = idx[1]; y <= idx[3]; ++y)
				{
					int iindex = x + y * m_width;
					m_cells.at(iindex).push_back((int)i);
				}
			}
		}
	}

	void TriangleChunk::clear()
	{
		m_cells.clear();
	}

	std::vector<int>& TriangleChunk::cell(trimesh::ivec2& idx)
	{
		int iindex = idx.x + idx.y * m_width;
		return m_cells.at(iindex);
	}
}