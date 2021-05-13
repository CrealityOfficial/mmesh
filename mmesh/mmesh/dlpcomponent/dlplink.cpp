#include "dlplink.h"
#include "mmesh/trimesh/shapefiller.h"

namespace mmesh
{
	DLPLink::DLPLink(const LinkParam& linkParam)
		:m_linkParam(linkParam)
	{
	}
	
	DLPLink::~DLPLink()
	{
	}

	void DLPLink::setDLPParam(LinkParam& linkParam)
	{
		m_linkParam = linkParam;
	}

	LinkParam DLPLink::param()
	{
		return m_linkParam;
	}

	int DLPLink::create(trimesh::vec3* position)
	{
		int validSize = 0;

		validSize += ShapeFiller::fillCylinder(position + validSize, m_linkParam.startRadius, m_linkParam.start,
			m_linkParam.endRadius, m_linkParam.end, 8, 0.0f);
		return validSize;
	}

	DLPUserType DLPLink::type()
	{
		return DLPUserType::eDLPLink;
	}

	int DLPLink::count()
	{
		return 84;
	}
}