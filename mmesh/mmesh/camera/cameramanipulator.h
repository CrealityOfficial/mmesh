#ifndef MMESH_CAMERAMANIPULATOR_1619922752768_H
#define MMESH_CAMERAMANIPULATOR_1619922752768_H
#include "trimesh2/Vec.h"

namespace mmesh
{
	class ScreenCamera;
	class ScreenCameraManipulator
	{
	public:
		ScreenCameraManipulator();
		virtual ~ScreenCameraManipulator();

		void setScreenCamera(ScreenCamera* camera);

		virtual void onRightStart(const trimesh::ivec2& pos, const trimesh::ivec2& size) = 0;
		virtual void onRightMove(const trimesh::ivec2& pos) = 0;
		virtual void onMiddleStart(const trimesh::ivec2& pos, const trimesh::ivec2& size) = 0;
		virtual void onMiddleMove(const trimesh::ivec2& pos) = 0;

		void store(const trimesh::ivec2& pos, const trimesh::ivec2& size);
	protected:
		ScreenCamera* m_camera;

		trimesh::ivec2 m_savePoint;
		trimesh::ivec2 m_saveSize;
		trimesh::vec3 m_saveUp;
		trimesh::vec3 m_saveCenter;
		trimesh::vec3 m_savePosition;
	};
}

#endif // MMESH_CAMERAMANIPULATOR_1619922752768_H