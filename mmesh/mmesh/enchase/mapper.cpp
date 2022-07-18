#include "mapper.h"
#include "trimesh2/TriMesh.h"
#include "texturegroup.h"

namespace enchase
{
	Mapper::Mapper()
		: m_source(nullptr)
	{

	}

	Mapper::~Mapper()
	{
		for (size_t i = 0; i < m_textureGroups.size(); ++i)
			delete m_textureGroups.at(i);
		m_textureGroups.clear();
	}

	void Mapper::map(trimesh::TriMesh* dest, int index)
	{
		if (!dest || !m_source.get())
			return;

		TextureGroup* texture = textureGroup(index);
		if (!texture)
			return;

		size_t size = texture->m_indexes.size();
		int vertexSize = (int)dest->vertices.size();
		for (size_t i = 0; i < size; ++i)
		{
			int index = texture->m_indexes.at(i);
			if (index >= 0 && index < vertexSize)
			{
				trimesh::vec3& p = dest->vertices.at(index);
				trimesh::vec3& n = dest->normals.at(index);
				trimesh::vec2& xy = texture->m_texcoord.at(i);

				float t = 0.0f;
				if (m_source->tex(t, xy.x, xy.y))
				{
					p += n * t;
				}
			}
		}
	}

    void Mapper::map(trimesh::TriMesh* dest, int index, float horizontalMargin, float verticalMargin, float maxThickness)
    {
        if (!dest || !m_source.get())
            return;

        TextureGroup* texture = textureGroup(index);
        if (!texture)
            return;

        if (horizontalMargin <= 0.0 && verticalMargin <= 0.0) {
            map(dest, index);
            return;
        }
        
        size_t size = texture->m_indexes.size();
        int vertexSize = (int)dest->vertices.size();
        for (size_t i = 0; i < size; ++i)
        {
            int index = texture->m_indexes.at(i);
            if (index >= 0 && index < vertexSize)
            {
                trimesh::vec3& p = dest->vertices.at(index);
                trimesh::vec3& n = dest->normals.at(index);
                trimesh::vec2& xy = texture->m_texcoord.at(i);

                trimesh::vec2 range(1.0 + horizontalMargin * 2.0,
                                    1.0 + verticalMargin * 2.0);
                
                trimesh::vec2 newxy = (xy - 0.5) * range + 0.5;
                
                if (newxy.x < 0.0 || newxy.x > 1.0 || newxy.y < 0.0 || newxy.y > 1.0) {
                    p += n * maxThickness;
                } else {
                    float t = 0.0f;
                    if (m_source->tex(t, newxy.x, newxy.y))
                    {
                        p += n * t;
                    }
                }
            }
        }
    }

	void Mapper::setSource(Source* source)
	{
		m_source.reset(source);
	}

	void Mapper::addTextureGroup(TextureGroup* textureGroup)
	{
		if(textureGroup)
			m_textureGroups.push_back(textureGroup);
	}

	int Mapper::textureGroups()
	{
		return (int)m_textureGroups.size();
	}

	TextureGroup* Mapper::textureGroup(int index)
	{
		if (index < 0 || index >= (int)m_textureGroups.size())
			return nullptr;

		return m_textureGroups.at(index);
	}

	std::vector<TextureGroup*>& Mapper::allTextureGroup()
	{
		return m_textureGroups;
	}
}
