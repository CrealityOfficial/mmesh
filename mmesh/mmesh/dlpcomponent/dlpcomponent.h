#ifndef MMESH_DLPCOMPONENT_1597889809542_H
#define MMESH_DLPCOMPONENT_1597889809542_H
#include "trimesh2/Vec.h"

namespace mmesh
{
	enum class DLPUserType
	{
		eDLPTop,
		eDLPMiddle,
		eDLPPlatformTrunk,
		eDLPModelTrunk,
		eDLPLink,
		eDLPNum
	};

	class DLPComponent
	{
	public:
		DLPComponent();
		virtual ~DLPComponent();

		virtual int create(trimesh::vec3* position) = 0;
		virtual DLPUserType type() = 0;
		virtual int count() = 0;
	};

	typedef std::shared_ptr<DLPComponent> DLPComponentPtr;
}

#endif // CREATIVE_KERNEL_DLPCOMPONENT_1597889809542_H