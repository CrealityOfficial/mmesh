#ifndef CREATIVE_KERNEL_DLPLINK_1597831108876_H
#define CREATIVE_KERNEL_DLPLINK_1597831108876_H
#include "mmesh/dlpcomponent/dlpcomponent.h"
#include <memory>

namespace mmesh
{
	struct LinkParam
	{
		trimesh::vec3 start;
		trimesh::vec3 end;

		float startRadius;
		float endRadius;
	};

	class DLPLink : public DLPComponent
	{
	public:
		DLPLink(const LinkParam& linkParam);
		virtual ~DLPLink();

		void setDLPParam(LinkParam& linkParam);
		LinkParam param();

	protected:
		int create(trimesh::vec3* position) override;
		DLPUserType type() override;
		int count() override;
	protected:
		LinkParam m_linkParam;
	};
}
#endif // CREATIVE_KERNEL_DLPLINK_1597831108876_H