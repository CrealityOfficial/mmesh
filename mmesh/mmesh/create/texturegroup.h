#ifndef MMESH_TEXTUREGROUP_1600694875112_H
#define MMESH_TEXTUREGROUP_1600694875112_H
#include "trimesh2/Vec.h"
#include <vector>

namespace mmesh
{
	class TextureGroup
	{
	public:
		TextureGroup();
		~TextureGroup();

		bool valid();
		void clear();

		std::vector<int> m_indexes;
		std::vector<trimesh::vec2> m_texcoord;
	};
}

#endif // ENCHASE_TEXTUREGROUP_1600694875112_H