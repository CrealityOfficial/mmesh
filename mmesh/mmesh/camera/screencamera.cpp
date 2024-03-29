#include "screencamera.h"
#include <math.h>
#include "mmesh/camera/rectutil.h"

namespace mmesh
{
	ScreenCamera::ScreenCamera()
		:m_viewMatrixDirty(true)
		, m_projectMatrixDirty(true)
		, m_fovy(30.0f)
		, m_near(1.0f)
		, m_far(100.0f)
		, m_aspectRatio(1.0f)
		, m_projectionType(ScreenCameraProjectionType::ePerspective)
		, m_top(1.0f)
		, m_bottom(-1.0f)
		, m_left(-1.0f)
		, m_right(1.0f)
	{

	}

	ScreenCamera::~ScreenCamera()
	{

	}

	void ScreenCamera::notifyViewMatrix(const trimesh::fxform& xform)
	{
		notify([&xform](ScreenCameraTracer* tracer) {
			tracer->onViewMatrixChanged(xform);
			});
	}

	trimesh::vec3 ScreenCamera::viewCenter()
	{
		return m_viewCenter;
	}

	trimesh::vec3 ScreenCamera::upVector()
	{
		return m_upVector;
	}

	trimesh::vec3 ScreenCamera::position()
	{
		return m_position;
	}

	void ScreenCamera::setViewCenter(const trimesh::vec3& viewCenter)
	{
		m_viewCenter = viewCenter;
		m_viewMatrixDirty = true;
	}

	void ScreenCamera::setUpVector(const trimesh::vec3& upVector)
	{
		m_upVector = upVector;
		m_viewMatrixDirty = true;
	}

	void ScreenCamera::setPosition(const trimesh::vec3& position)
	{
		m_position = position;
		m_viewMatrixDirty = true;
	}

	void ScreenCamera::updateViewMatrix()
	{
		if (m_viewMatrixDirty)
		{
			m_viewMatrix = trimesh::fxform::lookat(m_position, m_viewCenter, m_upVector);
			notifyViewMatrix(m_viewMatrix);
			m_viewMatrixDirty = false;
		}
	}

	void ScreenCamera::notifyProjectionMatrix(const trimesh::fxform& xform)
	{
		notify([&xform](ScreenCameraTracer* tracer) {
			tracer->onProjectionMatrixChanged(xform);
			});
	}

	float ScreenCamera::nearPlane()
	{
		return m_near;
	}

	float ScreenCamera::farPlane()
	{
		return m_far;
	}

	float ScreenCamera::fovy()
	{
		return m_fovy;
	}

	float ScreenCamera::aspectRatio()
	{
		return m_aspectRatio;
	}

	void ScreenCamera::setNearPlane(float nearP)
	{
		m_near = nearP;
		m_projectMatrixDirty = true;
	}

	void ScreenCamera::setFarPlane(float farP)
	{
		m_far = farP;
		m_projectMatrixDirty = true;
	}

	void ScreenCamera::setFovy(float fov)
	{
		m_fovy = fov;
		m_projectMatrixDirty = true;
	}

	void ScreenCamera::setAspectRatio(float ratio)
	{
		m_aspectRatio = ratio;
		m_projectMatrixDirty = true;
	}

	void ScreenCamera::updateProjectMatrix()
	{
		if (m_projectMatrixDirty)
		{
			if(m_projectionType == ScreenCameraProjectionType::ePerspective)
				m_projectMatrix = trimesh::fxform::perspective(m_fovy, m_aspectRatio, m_near, m_far);
			else
			{
				m_projectMatrix = trimesh::fxform::ortho(0.0f, 1.0f, 0.0f, 1.0f, m_near, m_far);
			}
			notifyProjectionMatrix(m_projectMatrix);
			m_projectMatrixDirty = false;
		}
	}

	void ScreenCamera::fittingBox(const trimesh::box3& box)
	{
		trimesh::vec3 center = box.center();
		float radius = box.radius();

		float fovy = m_fovy;
		float len = radius / sin(fovy * 3.1415926f / 180.0f / 2.0f);
		trimesh::vec3 eye = center + trimesh::vec3(0.0f, -1.0f, 0.0f) * len;
		trimesh::vec3 up = trimesh::vec3(0.0f, 0.0f, 1.0f);

		setView(eye, center, up);
		updateViewMatrix();

		setNearFar(1.0f, len + 3.0f * radius);
		updateProjectMatrix();
	}

	void ScreenCamera::setView(const trimesh::vec3& position, const trimesh::vec3& center, const trimesh::vec3& up)
	{
		m_position = position;
		m_viewCenter = center;
		m_upVector = up;
		m_viewMatrixDirty = true;
	}

	void ScreenCamera::setNearFar(float n, float f)
	{
		m_near = n;
		m_far = f;
		m_projectMatrixDirty = true;
	}

	bool ScreenCamera::zoom(float scale)
	{
		float factor = scale;

		float maxFovy = 50.0f;
		float minFovy = 2.0f;
		float fovy = m_fovy;
		fovy /= factor;
		if (fovy >= minFovy && fovy <= maxFovy)
		{
			setFovy(fovy);
			if (m_projectionType == ScreenCameraProjectionType::eOrth)
			{
				float fnearPlane = m_near;
				float ffarPlane = m_far;

				float r = (ffarPlane - fnearPlane) / 2 + fnearPlane;
				float flen = r * tan(m_fovy * M_PI / 360);
				float top = flen;
				float bottom = -flen;

				float fratio = m_aspectRatio;
				float left = -fratio * flen;
				float right = fratio * flen;

				setTop(top);
				setBottom(bottom);
				setLeft(left);
				setRight(right);
			}

			updateProjectMatrix();
			return true;
		}
		return false;
	}

	bool ScreenCamera::translate(const trimesh::vec3& trans)
	{
		trimesh::vec3 cameraPosition = m_position;
		trimesh::vec3 viewCenter = m_viewCenter;
		cameraPosition += trans;
		viewCenter += trans;

		setPosition(cameraPosition);
		setViewCenter(viewCenter);
		updateViewMatrix();

		//_updateNearFar(m_box);
		return true;
	}

	void ScreenCamera::translate(const trimesh::vec2& transR)
	{
		trimesh::vec3 cameraPosition = m_position;
		trimesh::vec3 viewCenter = m_viewCenter;
		trimesh::vec3 dir = viewCenter - cameraPosition;
		float D = trimesh::len(dir);
		trimesh::normalize(dir);
		trimesh::vec3 left = dir TRICROSS m_upVector;

		D *= tan(M_PI_180f * m_fovy / 2.0f) * 2.0f;
		float pixelY = D * transR.y;
		float pixelX = D * m_aspectRatio * transR.x;

		trimesh::vec3 t = pixelY * m_upVector - pixelX * left;
		translate(t);
	}

	bool ScreenCamera::rotate(const trimesh::vec3& axis, float angle)
	{
		trimesh::quaternion q = trimesh::quaternion::fromAxisAndAngle(axis, angle);
		return rotate(q);
	}

	trimesh::vec3 ScreenCamera::horizontal()
	{
		trimesh::vec3 viewCenter = m_viewCenter;
		trimesh::vec3 position = m_position;
		trimesh::vec3 dir = viewCenter - position;
		trimesh::normalize(dir);
		trimesh::vec3 up = m_upVector;

		trimesh::vec3 h = dir TRICROSS up;
		h.z = 0.0f;
		if (trimesh::len(h) > 0.0f)
		{
			trimesh::normalize(h);
		}
		if (trimesh::len(h) == 0.0f)
		{
			std::cout << " screen horizontal error";
		}
		return h;
	}

	trimesh::vec3 ScreenCamera::vertical()
	{
		trimesh::vec3 v = trimesh::vec3(0.0f, 1.0f, 0.0f);
		trimesh::vec3 viewCenter = m_viewCenter;
		trimesh::vec3 position = m_position;
		trimesh::vec3 dir = viewCenter - position;
		float y = v DOT dir;
		float z = sqrtf(trimesh::len2(dir) - y * y);
		trimesh::vec3 d(0.0f, y, z);
		if (trimesh::len2(d) > 0.0f)
			v = trimesh::normalized(d);
		return v;
	}

	float ScreenCamera::verticalAngle()
	{
		trimesh::vec3 v = trimesh::vec3(0.0f, 0.0f, 1.0f);
		trimesh::vec3 viewCenter = m_viewCenter;
		trimesh::vec3 position = m_position;
		trimesh::vec3 dir = viewCenter - position;
		trimesh::normalize(dir);
		float angle = acosf(v DOT dir);
		return angle;
	}

	bool ScreenCamera::rotate(const trimesh::quaternion& q)
	{
		trimesh::vec3 up = m_upVector;
		trimesh::vec3 viewCenter = m_viewCenter;
		trimesh::vec3 position = m_position;
		trimesh::vec3 dir = viewCenter - position;
		float distance = trimesh::len(dir);

		trimesh::normalize(dir);
		trimesh::vec3 newDir = q * dir;
		trimesh::normalize(newDir);
		trimesh::vec3 newPosition = viewCenter - newDir * distance;

		trimesh::vec3 newUp = q * up;
		trimesh::normalize(newUp);
		setUpVector(newUp);
		setPosition(newPosition);

		//_updateNearFar(m_box);
		return true;
	}

	float ScreenCamera::top()
	{
		return m_top;
	}

	float ScreenCamera::bottom()
	{
		return m_bottom;
	}

	float ScreenCamera::left()
	{
		return m_left;
	}

	float ScreenCamera::right()
	{
		return m_right;
	}

	void ScreenCamera::setTop(float top)
	{
		m_top = top;
		m_projectMatrixDirty = true;
	}

	void ScreenCamera::setBottom(float bottom)
	{
		m_bottom = bottom;
		m_projectMatrixDirty = true;
	}

	void ScreenCamera::setLeft(float left)
	{
		m_left = left;
		m_projectMatrixDirty = true;
	}

	void ScreenCamera::setRight(float right)
	{
		m_right = right;
		m_projectMatrixDirty = true;
	}

	Ray ScreenCamera::screenRay(const trimesh::ivec2& point, const trimesh::ivec2& size)
	{
		trimesh::vec2 p(0.0, 0.0);
		p = viewportRatio(point, size);

		if (m_projectionType == ScreenCameraProjectionType::ePerspective)
			return screenRay(p);
		else if (m_projectionType == ScreenCameraProjectionType::eOrth)
			return screenRayOrthographic(p);
		else
			return screenRay(p);
	}

	Ray ScreenCamera::screenRay(const trimesh::vec2& point)
	{
		trimesh::vec3 position = m_position;
		trimesh::vec3 center = m_viewCenter;
		trimesh::vec3 dir = trimesh::normalized(center - position);
		trimesh::vec3 nearCenter = position + dir * m_near;
		trimesh::vec3 left = dir TRICROSS m_upVector;
		trimesh::normalize(left);
		float near_height = 2.0f * m_near * tanf(m_fovy * 3.1415926f / 180.0f / 2.0f);
		float near_width = near_height * m_aspectRatio;
		trimesh::vec3 nearPoint = nearCenter + m_upVector * point.y * near_height + left * point.x * near_width;

		Ray ray;
		ray.start = m_position;
		ray.dir = nearPoint - ray.start;
		trimesh::normalize(ray.dir);
		return ray;
	}

	Ray ScreenCamera::screenRayOrthographic(const trimesh::vec2& point)
	{
		trimesh::vec3 position = m_position;
		trimesh::vec3 center = m_viewCenter;
		trimesh::vec3 dir = trimesh::normalized(center - position);
		trimesh::vec3 nearCenter = position + dir * m_near;
		
		float near_width = fabs(m_right - m_left) * point.x;
		float near_height = fabs(m_top - m_bottom) * point.y;
		trimesh::vec3 right = dir TRICROSS m_upVector;
		trimesh::normalize(right);
		trimesh::vec3 nearPoint = nearCenter + m_upVector * near_height + right * near_width;
		
		Ray ray;
		ray.dir = dir;// direction vector
		ray.start = nearPoint - dir * m_near;//start vector
		return ray;
	}

	//float ScreenCamera::screenSpaceRatio(const trimesh::vec3& position)
	//{
	//	float ratio = 1.0f;
	//
	//	//float nearPlane = m_camera->nearPlane();
	//	//float positionPlane = nearPlane;
	//	//
	//	//float h = 2.0f * nearPlane * tanf(m_camera->fieldOfView() * M_PI / 2.0f / 180.0f);
	//	//QVector3D cameraCenter = m_camera->position();
	//	//QVector3D cameraView = m_camera->viewCenter() - cameraCenter;
	//	//cameraView.normalize();
	//	//positionPlane = QVector3D::dotProduct(position - cameraCenter, cameraView);
	//	//
	//	//ratio = positionPlane * h / nearPlane / (float)m_size.height();
	//
	//	return ratio;
	//}
}