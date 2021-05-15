#ifndef MMESH_EULARMOUSEMANIPULATOR_1620107480743_H
#define MMESH_EULARMOUSEMANIPULATOR_1620107480743_H
#include "mmesh/camera/cameramanipulator.h"

namespace mmesh
{
	class EularMouseManipulator : public ScreenCameraManipulator
	{
	public:
		EularMouseManipulator();
		virtual ~EularMouseManipulator();

	protected:
		void onRightStart(const trimesh::ivec2& pos, const trimesh::ivec2& size) override;
		void onRightMove(const trimesh::ivec2& pos) override;
		void onMiddleStart(const trimesh::ivec2& pos, const trimesh::ivec2& size) override;
		void onMiddleMove(const trimesh::ivec2& pos) override;

		void performTranslate(const trimesh::ivec2& pos);
		void performRotate(const trimesh::ivec2& pos);
	};
}

#endif // MMESH_EULARMOUSEMANIPULATOR_1620107480743_H