#ifndef CREATIVE_KERNEL_DLPMODELTRUNK_1597831108876_H
#define CREATIVE_KERNEL_DLPMODELTRUNK_1597831108876_H
#include "mmesh/dlpcomponent/dlpcomponent.h"

namespace mmesh
{
	struct ModelTrunkParam
	{
		trimesh::vec3 bottomBottom;
		float middleRadius;
		float bottomHeight;
		float bottomBRadius;
	};

	class DLPModelTrunk : public DLPComponent
	{
	public:
		DLPModelTrunk(const ModelTrunkParam& param);
		virtual ~DLPModelTrunk();

		void setModelTrunkParam(ModelTrunkParam& trunkParam);
		ModelTrunkParam param();
	protected:
		int create(trimesh::vec3* position) override;
		DLPUserType type() override;
		int count() override;
	protected:
		ModelTrunkParam m_modelTrunkParam;
	};
}
#endif // CREATIVE_KERNEL_DLPMODELTRUNK_1597831108876_H