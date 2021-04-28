#include "split.h"

namespace mmesh
{
	SplitBase::SplitBase()
		:m_mesh(nullptr)
	{

	}

	SplitBase::~SplitBase()
	{

	}

	void SplitBase::setInputMesh(trimesh::TriMesh* mesh)
	{
		m_mesh = mesh;
	}
}