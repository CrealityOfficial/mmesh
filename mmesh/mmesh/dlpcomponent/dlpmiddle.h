#ifndef CREATIVE_KERNEL_DLPMIDDLE_1598835581230_H
#define CREATIVE_KERNEL_DLPMIDDLE_1598835581230_H
#include "mmesh/dlpcomponent/dlpcomponent.h"

namespace mmesh
{
	struct MiddleParam
	{
		trimesh::vec3 up;
		trimesh::vec3 down;

		float radius;
	};

	class DLPMiddle : public DLPComponent
	{
	public:
		DLPMiddle(const MiddleParam& param);
		virtual ~DLPMiddle();

		void setParam(MiddleParam& param);

		MiddleParam& param();

		void setIsPlatform(bool platform);
		bool isPlatform();
	protected:
		int create(trimesh::vec3* position) override;
		DLPUserType type() override;
		int count() override;
	protected:
		MiddleParam m_middleParam;
		bool m_isPlatform;
	};
}
#endif // CREATIVE_KERNEL_DLPMIDDLE_1598835581230_H