#include "dlpplatformtrunk.h"
#include "mmesh/trimesh/shapefiller.h"

namespace mmesh
{
	DLPPlatformTrunk::DLPPlatformTrunk(const PlatformTrunkParam& param)
		:m_platformTrunkParam(param)
	{
	}
	
	DLPPlatformTrunk::~DLPPlatformTrunk()
	{
	}

	void DLPPlatformTrunk::setPlatformTrunkParam(PlatformTrunkParam& trunkParam)
	{
		m_platformTrunkParam = trunkParam;
	}

	PlatformTrunkParam DLPPlatformTrunk::param()
	{
		return m_platformTrunkParam;
	}

	int DLPPlatformTrunk::create(trimesh::vec3* position)
	{
		int validSize = 0;

		trimesh::vec3 top = m_platformTrunkParam.bottomBottom + trimesh::vec3(0.0f, 0.0f, m_platformTrunkParam.bottomHeight);
		validSize += ShapeFiller::fillCylinder(position, m_platformTrunkParam.bottomBRadius, m_platformTrunkParam.bottomBottom,
			m_platformTrunkParam.bottomTRadius, top, 8, 0.0f);
		
		return validSize;
	}

	DLPUserType DLPPlatformTrunk::type()
	{
		return DLPUserType::eDLPPlatformTrunk;
	}

	int DLPPlatformTrunk::count()
	{
		return 84;
	}
}