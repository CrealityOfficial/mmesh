#ifndef MMESH_SCREENCAMERA_1619922752767_H
#define MMESH_SCREENCAMERA_1619922752767_H
#include "trimesh2/Vec.h"
#include "trimesh2/XForm.h"
#include "trimesh2/Box.h"
#include "mmesh/util/tcontainer.h"
#include "mmesh/trimesh/quaternion.h"
#include "mmesh/camera/ray.h"

namespace mmesh
{
	class ScreenCameraTracer
	{
	public:
		virtual ~ScreenCameraTracer() {}

		virtual void onViewMatrixChanged(const trimesh::fxform& xform) = 0;
		virtual void onProjectionMatrixChanged(const trimesh::fxform& xform) = 0;
	};

	enum class ScreenCameraProjectionType
	{
		ePerspective,
		eOrth
	};
	class ScreenCamera : public TContainer<ScreenCameraTracer>
	{
	public:
		ScreenCamera();
		~ScreenCamera();

		trimesh::vec3 viewCenter();
		trimesh::vec3 upVector();
		trimesh::vec3 position();

		void setViewCenter(const trimesh::vec3& viewCenter);
		void setUpVector(const trimesh::vec3& upVector);
		void setPosition(const trimesh::vec3& position);
		void updateViewMatrix();

		float nearPlane();
		float farPlane();
		float fovy();
		float aspectRatio();

		void setNearPlane(float nearPlane);
		void setFarPlane(float farPlane);
		void setFovy(float fov);
		void setAspectRatio(float ratio);
		void updateProjectMatrix();

		float top();
		float bottom();
		float left();
		float right();
		void setTop(float top);
		void setBottom(float bottom);
		void setLeft(float left);
		void setRight(float right);

		void fittingBox(const trimesh::box3& box);
		void setView(const trimesh::vec3& position, const trimesh::vec3& center, const trimesh::vec3& up);
		void setNearFar(float n, float f);

		bool zoom(float scale);
		bool translate(const trimesh::vec3& trans);
		bool rotate(const trimesh::vec3& axis, float angle);
		bool rotate(const trimesh::quaternion& q);

		Ray screenRay(const trimesh::ivec2& point, const trimesh::ivec2& size);
		Ray screenRay(const trimesh::vec2& point);
		Ray screenRayOrthographic(const trimesh::vec2& point);
		//float screenSpaceRatio(const trimesh::vec3& position);

		trimesh::vec3 horizontal();
		trimesh::vec3 vertical();
		float verticalAngle();
	protected:
		void notifyViewMatrix(const trimesh::fxform& xform);
		void notifyProjectionMatrix(const trimesh::fxform& xform);
	protected:
		trimesh::vec3 m_viewCenter;
		trimesh::vec3 m_upVector;
		trimesh::vec3 m_position;
		bool m_viewMatrixDirty;
		trimesh::fxform m_viewMatrix;

		float m_near;
		float m_far;
		float m_fovy;
		float m_aspectRatio;

		float m_top;
		float m_bottom;
		float m_left;
		float m_right;

		ScreenCameraProjectionType m_projectionType;
		bool m_projectMatrixDirty;
		trimesh::fxform m_projectMatrix;
	};
}

#endif // MMESH_SCREENCAMERA_1619922752767_H