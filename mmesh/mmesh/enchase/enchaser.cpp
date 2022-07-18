#include "enchaser.h"
#include "mapper.h"

namespace enchase
{
	Enchaser::Enchaser()
		:m_source(nullptr)
	{

	}

	Enchaser::~Enchaser()
	{

	}

	trimesh::TriMesh* Enchaser::source()
	{
		return m_source;
	}

	void Enchaser::setSource(trimesh::TriMesh* source)
	{
		m_source = source;
	}

	trimesh::TriMesh* Enchaser::enchase(Mapper* mapper, int index)
	{
		if (!m_source || !mapper)
			return nullptr;

		trimesh::TriMesh* dest = new trimesh::TriMesh();
		*dest = *m_source;
		mapper->map(dest, index);

		return dest;
	}

	void Enchaser::write(const std::string& stlFile)
	{
		if (!m_source)
		{
			return;
		}

		m_source->write(stlFile);
	}

	void Enchaser::enchaseCache(Mapper* mapper, int index)
	{
		if (!mapper)
			return;

		if (!m_current)
		{
			m_current.reset(new trimesh::TriMesh());
			*m_current = *m_source;
		}

		mapper->map(m_current.get(), index);
	}

    void Enchaser::enchaseCache(Mapper* mapper, int index, float horizontalMargin, float verticalMargin, float maxThickness)
    {
        if (!mapper)
            return;

        if (!m_current)
        {
            m_current.reset(new trimesh::TriMesh());
            *m_current = *m_source;
        }

        mapper->map(m_current.get(), index, horizontalMargin, verticalMargin, maxThickness);
    }

	trimesh::TriMesh* Enchaser::takeCurrent()
	{
		return m_current.release();
	}
}
