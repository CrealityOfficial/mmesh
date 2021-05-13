#ifndef ENCHASE_MAPPER_1600262114238_H
#define ENCHASE_MAPPER_1600262114238_H
#include <memory>
#include "source.h"
#include "trimesh2/Vec.h"
#include <vector>

namespace trimesh
{
	class TriMesh;
}

namespace enchase
{
	class TextureGroup;
	class Mapper
	{
	public:
		Mapper();
		~Mapper();

		void map(trimesh::TriMesh* dest, int index);
		void setSource(Source* source);
		void addTextureGroup(TextureGroup* textureGroup);
		int textureGroups();
		TextureGroup* textureGroup(int index);
		std::vector<TextureGroup*>& allTextureGroup();
	protected:
		std::unique_ptr<Source> m_source;
		std::vector<TextureGroup*> m_textureGroups;
	};
}

#endif // ENCHASE_MAPPER_1600262114238_H