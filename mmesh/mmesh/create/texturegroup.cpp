#include "texturegroup.h"

namespace mmesh
{
	TextureGroup::TextureGroup()
	{

	}

	TextureGroup::~TextureGroup()
	{
	}

	bool TextureGroup::valid()
	{
		return m_indexes.size() > 0;
	}

	void TextureGroup::clear()
	{
		m_indexes.clear();
		m_texcoord.clear();
	}
}