#include "trimeshgenerator.h"
#include "mapper.h"
#include "enchaser.h"
#include "texturegroup.h"
#include "imagematrixfsource.h"
#include "mmesh/trimesh/trimeshutil.h"

namespace enchase
{
	trimesh::TriMesh* enchaseGenerate(trimesh::TriMesh* input, SurfacePtr surface, bool ball)
	{
		if (!input || input->cornerareas.size() == 0)
			return nullptr;

		MatrixF* mat = surface->matrix();
		if (!surface || !mat)
			return nullptr;

		if(ball && (float)mat->width()/(float)mat->height() > 1.5f)
			mmesh::convertUV2Equ(input);

		enchase::Mapper mapper;
		enchase::Enchaser enchaser;
		enchaser.setSource(input);

		enchase::TextureGroup* texture = new enchase::TextureGroup();
		size_t size = input->vertices.size();
		if (size > 0)
		{
			texture->m_indexes.resize(size);
			texture->m_texcoord.resize(size);

			for (size_t i = 0; i < size; ++i)
			{
				texture->m_indexes.at(i) = (int)i;
				texture->m_texcoord.at(i) = input->cornerareas.at(i);
			}
		}

		mapper.allTextureGroup().push_back(texture);

		mapper.setSource(new enchase::MatrixFSource(mat));
		enchaser.enchaseCache(&mapper, 0);

		trimesh::TriMesh* mesh = enchaser.takeCurrent();
		if (mesh)
			mesh->cornerareas.clear();

		return mesh;
	}
}