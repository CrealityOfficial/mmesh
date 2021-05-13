#include "dlptop.h"
#include "mmesh/trimesh/shapefiller.h"

namespace mmesh
{
	DLPTop::DLPTop()
	{
	}

	DLPTop::DLPTop(const TopParam& topParam)
		:m_topParam(topParam)
	{

	}
	
	DLPTop::~DLPTop()
	{
	}

	void DLPTop::setTopParam(TopParam& topParam)
	{
		m_topParam = topParam;
	}

	TopParam DLPTop::param()
	{
		return m_topParam;
	}

	int DLPTop::create(trimesh::vec3* position)
	{
		int validSize = 0;
		validSize += ShapeFiller::fillCylinder(position + validSize, m_topParam.topLinkTRadius, m_topParam.contactCenter,
			m_topParam.topLinkBRadius, m_topParam.linkBottom, 8, 0.0f);
		validSize += ShapeFiller::fillSphere(position + validSize, m_topParam.topContactRadius, m_topParam.contactCenter);

		return validSize;
	}

	DLPUserType DLPTop::type()
	{
		return DLPUserType::eDLPTop;
	}

	int DLPTop::count()
	{
		return 684;
	}
}