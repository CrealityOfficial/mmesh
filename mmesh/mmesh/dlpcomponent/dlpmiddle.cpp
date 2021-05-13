#include "dlpmiddle.h"
#include "mmesh/trimesh/shapefiller.h"

namespace mmesh
{
	DLPMiddle::DLPMiddle(const MiddleParam& param)
		:m_middleParam(param)
		, m_isPlatform(false)
	{
	}
	
	DLPMiddle::~DLPMiddle()
	{
	}

	void DLPMiddle::setParam(MiddleParam& param)
	{
		m_middleParam = param;
	}

	MiddleParam& DLPMiddle::param()
	{
		return m_middleParam;
	}

	void DLPMiddle::setIsPlatform(bool platform)
	{
		m_isPlatform = platform;
	}

	bool DLPMiddle::isPlatform()
	{
		return m_isPlatform;
	}

	int DLPMiddle::create(trimesh::vec3* position)
	{
		int validSize = 0;

		validSize += ShapeFiller::fillCylinder(position + validSize, m_middleParam.radius, m_middleParam.up,
			m_middleParam.radius, m_middleParam.down, 8, 0.0f);
		validSize += ShapeFiller::fillSphere(position + validSize, m_middleParam.radius, m_middleParam.up);

		return validSize;
	}

	DLPUserType DLPMiddle::type()
	{
		return DLPUserType::eDLPMiddle;
	}

	int DLPMiddle::count()
	{
		return 684;
	}
}