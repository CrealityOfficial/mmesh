#include "cameramanipulator.h"
#include "mmesh/camera/screencamera.h"

namespace mmesh
{
	ScreenCameraManipulator::ScreenCameraManipulator()
		:m_camera(nullptr)
	{

	}

	ScreenCameraManipulator::~ScreenCameraManipulator()
	{

	}

	void ScreenCameraManipulator::setScreenCamera(ScreenCamera* camera)
	{
		m_camera = camera;
	}

	void ScreenCameraManipulator::store(const trimesh::ivec2& pos, const trimesh::ivec2& size)
	{
		m_savePoint = pos;
		m_saveSize = size;

		if (m_camera)
		{
			m_saveUp = m_camera->upVector();
			m_saveCenter = m_camera->viewCenter();
			m_savePosition = m_camera->position();
		}
	}
}