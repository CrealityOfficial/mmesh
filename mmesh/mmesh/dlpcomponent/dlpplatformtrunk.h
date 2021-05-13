#ifndef CREATIVE_KERNEL_DLPPLATFORMTRUNK_1597831108867_H
#define CREATIVE_KERNEL_DLPPLATFORMTRUNK_1597831108867_H
#include "mmesh/dlpcomponent/dlpcomponent.h"

namespace mmesh
{
	struct PlatformTrunkParam
	{
		trimesh::vec3 bottomBottom;

		float bottomHeight;
		float bottomBRadius;
		float bottomTRadius;
	};

	class DLPPlatformTrunk : public DLPComponent
	{
	public:
		DLPPlatformTrunk(const PlatformTrunkParam& param);
		virtual ~DLPPlatformTrunk();

		void setPlatformTrunkParam(PlatformTrunkParam& trunkParam);
		PlatformTrunkParam param();
	protected:
		int create(trimesh::vec3* position) override;
		DLPUserType type() override;
		int count() override;
	protected:
		PlatformTrunkParam m_platformTrunkParam;
	};
}
#endif // CREATIVE_KERNEL_DLPPLATFORMTRUNK_1597831108867_H