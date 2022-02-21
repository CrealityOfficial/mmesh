#include "screencamera.h"
#include <math.h>
#include "mmesh/camera/rectutil.h"
#include "mmesh/util/mnode.h"

namespace mmesh
{
	trimesh::fxform ScreenCameraMeta::viewMatrix()
	{
		return trimesh::fxform::lookat(position, viewCenter, upVector);
	}

	trimesh::fxform ScreenCameraMeta::posMatrix()
	{
		//trimesh::vec3 dir = viewCenter - position;
		//trimesh::normalize(dir);
		//trimesh::vec3 x = -(upVector TRICROSS dir);
		//trimesh::normalize(x);
		//
		//trimesh::quaternion q = trimesh::quaternion::fromAxes(x, upVector, dir);
		//q.normalize();
		//
		//trimesh::fxform rot = mmesh::fromQuaterian(q);
		trimesh::fxform view = viewMatrix();
		//trimesh::fxform rot = trimesh::rot_only(view);
		//return trimesh::fxform::trans(position) * rot;
		return trimesh::inv(view);
	}

	trimesh::fxform ScreenCameraMeta::projectionMatrix()
	{
		trimesh::fxform xf = trimesh::fxform::identity();
		if (type == ScreenCameraProjectionType::ePerspective) {
			xf = trimesh::fxform::perspective(fovy, aspectRatio, fNear, fFar);
		}
		else {
			xf = trimesh::fxform::ortho(left, right, bottom, top, fNear, fFar);
		}
		return xf;
	}

	void createCameraPoints(ScreenCameraMeta* meta, std::vector<trimesh::vec3>& positions)
	{
		positions.resize(9);
		positions.at(0) = trimesh::vec3(0.0f, 0.0f, 0.0f);
		float nZ = - meta->fNear;
		float fZ = - meta->fFar;
		float nY = meta->fNear * tanf(meta->fovy * M_PIf / 360.0f);
		float fY = meta->fFar * tanf(meta->fovy * M_PIf / 360.0f);
		float nX = meta->aspectRatio * nY;
		float fX = meta->aspectRatio * fY;
		
		positions.at(1) = trimesh::vec3(nX, nY, nZ);
		positions.at(2) = trimesh::vec3(-nX, nY, nZ);
		positions.at(3) = trimesh::vec3(-nX, -nY, nZ);
		positions.at(4) = trimesh::vec3(nX, -nY, nZ);
		positions.at(5) = trimesh::vec3(fX, fY, fZ);
		positions.at(6) = trimesh::vec3(-fX, fY, fZ);
		positions.at(7) = trimesh::vec3(-fX, -fY, fZ);
		positions.at(8) = trimesh::vec3(fX, -fY, fZ);
	}

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
		, m_maniFlag(7)
        , m_minFovy(2.0f)
        , m_maxFovy(50.0f)
	{
#ifdef WIN32
		m_aspectRatio = 1920.0f / 1080.0f;
#elif __ANDROID__ || __APPLE__
		m_aspectRatio = 540.0f / 1080.0f;
#endif
	}

	ScreenCamera::~ScreenCamera()
	{

	}

	ScreenCameraMeta ScreenCamera::traitMeta()
	{
		ScreenCameraMeta meta;

		meta.type = m_projectionType;
		meta.viewCenter = m_viewCenter;
		meta.upVector = m_upVector;
		meta.position = m_position;

		meta.fNear = m_near;
		meta.fFar = m_far;
		meta.fovy = m_fovy;
		meta.aspectRatio = m_aspectRatio;

		meta.top = m_top;
		meta.bottom = m_bottom;
		meta.left = m_left;
		meta.right = m_right;

		return meta;
	}

	void ScreenCamera::enableTranslate(bool enable)
	{
		if(enable)
			m_maniFlag |= 1;
		else
			m_maniFlag &=~1;
	}

	void ScreenCamera::enableRotate(bool enable)
	{
		if(enable)
			m_maniFlag |= 2;
		else
			m_maniFlag &=~2;
	}

	void ScreenCamera::enableScale(bool enable)
	{
		if(enable)
			m_maniFlag |= 4;
		else
			m_maniFlag &=~4;
	}

	bool ScreenCamera::canTranslate()
	{
		return m_maniFlag & 1;
	}

	bool ScreenCamera::canRotate()
	{
		return m_maniFlag & 2;
	}

	bool ScreenCamera::canScale()
	{
		return m_maniFlag & 4;
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

    float ScreenCamera::maxFovy()
    {
        return m_maxFovy;
    }

    float ScreenCamera::minFovy()
    {
        return m_minFovy;
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

    void ScreenCamera::setMaxFovy(float maxfov)
    {
        m_maxFovy = maxfov;
        m_projectMatrixDirty = true;
    }

    void ScreenCamera::setMinFovy(float minfov)
    {
        m_minFovy = minfov;
        m_projectMatrixDirty = true;
    }

	void ScreenCamera::setAspectRatio(float ratio)
	{
		m_aspectRatio = ratio;
		if(m_centerBox.valid)
			fitNotChangeDir(m_box, m_centerBox);

		m_projectMatrixDirty = true;
		updateProjectMatrix();
	}

	void ScreenCamera::fitNotChangeDir(const trimesh::box3& box, const trimesh::box3& centerBox)
	{
		m_box = box;
		m_centerBox = centerBox;

		trimesh::vec3 size = m_centerBox.size();
		float radius = m_centerBox.radius();

		float fovy = 30.0f * M_PIf / 180.0f;

		auto f = [](float z, float x, float fovy)->float{
			float r = sqrtf( x * x + z * z) / 2.0f;
			return r / sinf(fovy / 2.0f);
		};

		float len1 = f(size.z, size.y, fovy);
		float len2 = f(size.x, size.y, 2.0f * atanf(m_aspectRatio * tanf(fovy / 2.0f)));
		float len = len1 > len2 ? len1 :len2;

		trimesh::vec3 dir = m_viewCenter - m_position;
		trimesh::normalize(dir);
		trimesh::vec3 eye = m_viewCenter - dir * len;
		trimesh::vec3 center = m_viewCenter;
		trimesh::vec3 up = m_upVector;
		setView(eye, center, up);
		updateViewMatrix();

		updateNearFar(m_box);
	}

	void ScreenCamera::updateProjectMatrix()
	{
		if (m_projectMatrixDirty)
		{
            if (m_projectionType == ScreenCameraProjectionType::ePerspective) {
                m_projectMatrix = trimesh::fxform::perspective(m_fovy, m_aspectRatio, m_near, m_far);
            } else {
				m_projectMatrix = trimesh::fxform::ortho(m_left, m_right, m_bottom, m_top, m_near, m_far);
			}
			notifyProjectionMatrix(m_projectMatrix);
			m_projectMatrixDirty = false;
		}
	}

	void ScreenCamera::fittingBox(const trimesh::box3& box)
	{
        m_box = box;
        m_centerBox = box;

		trimesh::vec3 center = box.center();
		trimesh::vec3 size = box.size();
		float radius = box.radius();

		m_fovy = 30.0f;
		float fovy = m_fovy * M_PIf / 180.0f;

		auto f = [](float z, float x, float fovy)->float{
		    float r = sqrtf( x * x + z * z) / 2.0f;
		    return r / sinf(fovy / 2.0f);
		};

		float len1 = f(size.z, size.y, fovy);
		float len2 = f(size.x, size.y, 2.0f * atanf(m_aspectRatio * tanf(fovy / 2.0f)));
		float len = len1 > len2 ? len1 :len2;

		trimesh::vec3 eye = center + trimesh::vec3(0.0f, -1.0f, 0.0f) * len;
		trimesh::vec3 up = trimesh::vec3(0.0f, 0.0f, 1.0f);

		setView(eye, center, up);
		updateViewMatrix();

		updateNearFar(m_box);
	}

	void ScreenCamera::adjustCamera(const trimesh::box3& box, int type)
	{
		m_box = box;
		m_centerBox = box;

		trimesh::vec3 size = m_centerBox.size();
		trimesh::vec3 center = m_centerBox.center();

//		if(type == 1)
//		{
//			float t = size.y;
//			size.y = size.z;
//			size.z = t;
//		}else if(type == 2)
//		{
//			float t = size.y;
//			size.y = size.x;
//			size.x = t;
//		}

		m_fovy = 30.0f;
		float fovy = m_fovy * M_PIf / 180.0f;

		auto f = [](float z, float x, float fovy)->float{
			float r = sqrtf( x * x + z * z) / 2.0f;
			return r / sinf(fovy / 2.0f);
		};

		float len1 = f(size.z, size.y, fovy);
		float len2 = f(size.x, size.y, 2.0f * atanf(m_aspectRatio * tanf(fovy / 2.0f)));
        
		float len = type == 0 ? len1 : len2;

		trimesh::vec3 eye = center + trimesh::vec3(0.0f, -1.0f, 0.0f) * len;
		trimesh::vec3 up = trimesh::vec3(0.0f, 0.0f, 1.0f);

		setView(eye, center, up);
		updateViewMatrix();

		updateNearFar(m_box);
	}

	void ScreenCamera::fittingBox(const trimesh::box3& box, const trimesh::box& centerBox, const trimesh::vec3& center)
	{
		m_box = box;
		m_centerBox = centerBox;

		trimesh::vec3 size = centerBox.size();
		float radius = centerBox.radius();

		m_fovy = 30.0f;
		float fovy = m_fovy * M_PIf / 180.0f;

		auto f = [](float z, float x, float fovy)->float{
			float r = sqrtf( x * x + z * z) / 2.0f;
			return r / sinf(fovy / 2.0f);
		};

		float len1 = f(size.z, size.y, fovy);
		float len2 = f(size.x, size.y, 2.0f * atanf(m_aspectRatio * tanf(fovy / 2.0f)));
		float len = len1 > len2 ? len1 :len2;

		trimesh::vec3 eye = center + trimesh::vec3(0.0f, -1.0f, 0.0f) * len;
		trimesh::vec3 up = trimesh::vec3(0.0f, 0.0f, 1.0f);

		setView(eye, center, up);
		updateViewMatrix();

		updateNearFar(m_box);
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
		if(!canScale())
			return false;

		float factor = scale;

		float maxFovy = m_maxFovy;
		float minFovy = m_minFovy;
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
		if(!canTranslate())
			return false;

		trimesh::vec3 cameraPosition = m_position;
		trimesh::vec3 viewCenter = m_viewCenter;
		cameraPosition += trans;
		viewCenter += trans;

		setPosition(cameraPosition);
		setViewCenter(viewCenter);
		updateViewMatrix();

		updateNearFar(m_box);
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
			return trimesh::vec3(1.0f, 0.0f, 0.0f);

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
		if(!canRotate())
			return false;

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

		updateViewMatrix();

		updateNearFar(m_box);
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

    void ScreenCamera::setProjectionType(ScreenCameraProjectionType projection)
    {
        m_projectionType = projection;
    }

    void ScreenCamera::viewFromBottom()
    {
        trimesh::vec3 dir(0.0f, 0.0f, 1.0f);
        trimesh::vec3 right(1.0f, 0.0f, 0.0f);
        view(dir, right);
    }

    void ScreenCamera::viewFromTop()
    {
        trimesh::vec3 dir(0.0f, 0.0f, -1.0f);
        trimesh::vec3 right(1.0f, 0.0f, 0.0f);
        view(dir, right);
    }

    void ScreenCamera::viewFromLeft()
    {
        trimesh::vec3 dir(1.0f, 0.0f, 0.0f);
        trimesh::vec3 right(0.0f, -1.0f, 0.0f);
        view(dir, right);
    }

    void ScreenCamera::viewFromRight()
    {
        trimesh::vec3 dir(-1.0f, 0.0f, 0.0f);
        trimesh::vec3 right(0.0f, 1.0f, 0.0f);
        view(dir, right);
    }

    void ScreenCamera::viewFromFront()
    {
        trimesh::vec3 dir(0.0f, 1.0f, 0.0f);
        trimesh::vec3 right(1.0f, 0.0f, 0.0f);
        view(dir, right);
    }

    void ScreenCamera::viewFromBack()
    {
        trimesh::vec3 dir(0.0f, -1.0f, 0.0f);
        trimesh::vec3 right(-1.0f, 0.0f, 0.0f);
        view(dir, right);
    }

    void ScreenCamera::view(const trimesh::vec3& dir, const trimesh::vec3& right)
    {
        trimesh::vec3 newUp = right TRICROSS dir;
        trimesh::normalize(newUp);

        trimesh::vec3 viewCenter = m_viewCenter;
        trimesh::vec3 position = m_position;
        float d = trimesh::len(viewCenter - position);
        trimesh::vec3 newPosition = viewCenter - trimesh::normalized(dir) * d;

        setView(newPosition, viewCenter, newUp);
        updateViewMatrix();

		updateNearFar(m_box);
    }

	void ScreenCamera::viewFromMeta(const ScreenCameraMeta& meta)
	{
		m_projectionType = meta.type;
		m_viewCenter	 = meta.viewCenter;
		m_upVector		 = meta.upVector;
		m_position		 = meta.position;
						 
		m_near			 = meta.fNear;
		m_far			 = meta.fFar;
		m_fovy			 = meta.fovy;
		m_aspectRatio	 = meta.aspectRatio;
						 
		m_top			 = meta.top;
		m_bottom		 = meta.bottom;
		m_left			 = meta.left;
		m_right          = meta.right;

		m_viewMatrixDirty = true;
		m_projectMatrixDirty = true;

		updateViewMatrix();
		updateProjectMatrix();
	}

	void ScreenCamera::updateNearFar(const trimesh::box3& box)
	{
		trimesh::vec3 cameraPosition = m_position;
		trimesh::vec3 cameraCenter = m_viewCenter;
		trimesh::vec3 cameraView = cameraCenter - cameraPosition;
		trimesh::normalize(cameraView);

		trimesh::vec3 center = box.center();
		float r = trimesh::len(box.size()) / 2.0f;
		float d = cameraView DOT (center - cameraPosition);
		float dmin = d - 1.2f * r;
		float dmax = d + 1.2f * r;

		float nearpos = dmin < 1.0f ? (2.0f * r > 1.0f ? 0.1f : dmin) : dmin;
		float farpos = dmax > 0.0f ? dmax : 3000.0f;

		setNearFar(nearpos, farpos);
		updateProjectMatrix();
	}
}
