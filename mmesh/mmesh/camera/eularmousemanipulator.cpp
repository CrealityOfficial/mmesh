#include "eularmousemanipulator.h"
#include "mmesh/camera/screencamera.h"
#include "mmesh/camera/ray.h"
#include "mmesh/camera/space3d.h"

namespace mmesh
{
	EularMouseManipulator::EularMouseManipulator()
	{

	}

	EularMouseManipulator::~EularMouseManipulator()
	{

	}

	void EularMouseManipulator::onRightStart(const trimesh::ivec2& pos, const trimesh::ivec2& size)
	{
		store(pos, size);
	}

	void EularMouseManipulator::onRightMove(const trimesh::ivec2& pos)
	{
		if (m_savePoint == pos || !m_camera)
			return;

		performRotate(pos);
		m_savePoint = pos;

		m_camera->updateViewMatrix();
	}

	void EularMouseManipulator::onMiddleStart(const trimesh::ivec2& pos, const trimesh::ivec2& size)
	{
		store(pos, size);
	}

	void EularMouseManipulator::onMiddleMove(const trimesh::ivec2& pos)
	{
		if (m_savePoint == pos || !m_camera)
			return;

		performTranslate(pos);
		m_savePoint = pos;

		m_camera->updateViewMatrix();
	}

	void EularMouseManipulator::performTranslate(const trimesh::ivec2& pos)
	{
		trimesh::vec3 viewCenter = m_camera->viewCenter();
		trimesh::vec3 position = m_camera->position();
		
		trimesh::vec3 dir = viewCenter - position;
		trimesh::normalize(dir);
		
		mmesh::Ray ray0 = m_camera->screenRay(m_savePoint, m_saveSize);
		mmesh::Ray ray = m_camera->screenRay(pos, m_saveSize);
		
		trimesh::vec3 c0, c;
		lineCollidePlane(viewCenter, dir, ray0, c0);
		lineCollidePlane(viewCenter, dir, ray, c);
		
		trimesh::vec3 delta = c0 - c;
		trimesh::vec3 newPosition = position + delta;
		trimesh::vec3 newViewCenter = viewCenter + delta;
		
		m_camera->setPosition(newPosition);
		m_camera->setViewCenter(newViewCenter);
		
		//m_screenCamera->updateNearFar();
	}

	void EularMouseManipulator::performRotate(float angle, const trimesh::vec3& axis)
	{
		trimesh::quaternion hq = trimesh::quaternion::fromAxisAndAngle(axis, angle);
	}

	void EularMouseManipulator::performRotate(const trimesh::ivec2& pos)
	{
		trimesh::ivec2 idelta = pos - m_savePoint;
		trimesh::vec2 delta = trimesh::vec2((float)idelta.x, (float)idelta.y);

		float hangle = -0.1f * (float)delta.x;
		trimesh::quaternion hq = trimesh::quaternion::fromAxisAndAngle(trimesh::vec3(0.0f, 0.0f, 1.0f), hangle);

		trimesh::vec3 viewCenter = m_camera->viewCenter();
		trimesh::vec3 position = m_camera->position();
		trimesh::vec3 horizontal = m_camera->horizontal();

		trimesh::vec3 h = hq * horizontal;
		trimesh::normalize(h);

		float vangle = m_camera->verticalAngle() + 0.003f * (float)delta.y;
		if (vangle < 0.0f) vangle = 0.0f;
		if (vangle > M_PI) vangle = (float)M_PI;

		trimesh::vec3 dir;
		trimesh::vec3 right = h;
		if (vangle > 0.0f && vangle < M_PI)
		{
			dir = trimesh::vec3(-h.y, h.x, 0.0f);
			float z = trimesh::len(dir) / (tanf(vangle));
			dir.z = z;
			trimesh::normalize(dir);
		}
		else if (vangle <= 0.0f)
		{
			dir = trimesh::vec3(0.0f, 0.0f, 1.0f);
		}
		else if (vangle >= M_PI)
		{
			dir = trimesh::vec3(0.0f, 0.0f, -1.0f);
		}

		trimesh::vec3 saveDir = viewCenter - position;
		float distance = trimesh::len(saveDir);

		trimesh::vec3 newPosition = viewCenter - dir * distance;
		trimesh::vec3 up = right TRICROSS dir;

		if (up.x == 0.0f && up.y == 0.0f && up.z == 0.0f)
			return;

		m_camera->setUpVector(up);
		m_camera->setPosition(newPosition);
	}
}