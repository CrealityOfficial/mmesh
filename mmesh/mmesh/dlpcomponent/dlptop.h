#ifndef MMESH_DLPTOP_1597831108866_H
#define MMESH_DLPTOP_1597831108866_H
#include "mmesh/dlpcomponent/dlpcomponent.h"

namespace mmesh
{
	struct TopParam
	{
		trimesh::vec3 contactCenter;
		trimesh::vec3 linkBottom;
		float topContactDepth;
		float topContactRadius;
		float topLinkTRadius;
		float topLinkBRadius;
	};

	class DLPTop : public DLPComponent
	{
	public:
		DLPTop();
		DLPTop(const TopParam& topParam);
		virtual ~DLPTop();

		void setTopParam(TopParam& topParam);
		TopParam param();
	protected:
		int create(trimesh::vec3* position) override;
		DLPUserType type() override;
		int count() override;
	protected:
		TopParam m_topParam;
	};

	typedef std::shared_ptr<DLPTop> DLPTopPtr;
}
#endif // CREATIVE_KERNEL_DLPTOP_1597831108866_H