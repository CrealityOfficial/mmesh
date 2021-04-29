#include "dlpmodeltrunk.h"
#include "mmesh/trimesh/shapefiller.h"

namespace mmesh
{
	DLPModelTrunk::DLPModelTrunk(const ModelTrunkParam& param)
		:m_modelTrunkParam(param)
	{
	}
	
	DLPModelTrunk::~DLPModelTrunk()
	{
	}

	void DLPModelTrunk::setModelTrunkParam(ModelTrunkParam& trunkParam)
	{
		m_modelTrunkParam = trunkParam;
	}

	ModelTrunkParam DLPModelTrunk::param()
	{
		return m_modelTrunkParam;
	}

	int DLPModelTrunk::create(trimesh::vec3* position)
	{
		int validSize = 0;
		
		trimesh::vec3 top = m_modelTrunkParam.bottomBottom + trimesh::vec3(0.0f, 0.0f, m_modelTrunkParam.bottomHeight);
		validSize += ShapeFiller::fillCylinder(position + validSize, m_modelTrunkParam.bottomBRadius, m_modelTrunkParam.bottomBottom,
			m_modelTrunkParam.middleRadius, top, 8, 0.0f);

		return validSize;
	}

	DLPUserType DLPModelTrunk::type()
	{
		return DLPUserType::eDLPModelTrunk;
	}

	int DLPModelTrunk::count()
	{
		return 84;
	}
}