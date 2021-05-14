#include "texturegroup.h"

namespace enchase
{
	TextureGroup::TextureGroup()
	{

	}

	TextureGroup::~TextureGroup()
	{
		m_indexes.clear();
		m_texcoord.clear();
	}
}