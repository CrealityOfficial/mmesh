#ifndef MMESH_SCREENCAMERA_1619922752767_H
#define MMESH_SCREENCAMERA_1619922752767_H
#include "trimesh2/Vec.h"
#include "trimesh2/XForm.h"
#include "trimesh2/Box.h"
#include "mmesh/util/tcontainer.h"
#include "trimesh2/quaternion.h"
#include "mmesh/camera/ray.h"
#include <vector>

namespace mmesh
{
	enum class ScreenCameraProjectionType
	{
		ePerspective,
		eOrth
	};

	struct ScreenCameraMeta
	{
		ScreenCameraProjectionType type;

		trimesh::vec3 viewCenter;
		trimesh::vec3 upVector;
		trimesh::vec3 position;

		float fNear;
		float fFar;
		float fovy;
		float aspectRatio;

		float top;
		float bottom;
		float left;
		float right;

		trimesh::fxform viewMatrix();
		trimesh::fxform posMatrix();
		trimesh::fxform projectionMatrix();
	};

	void createCameraPoints(ScreenCameraMeta* meta, std::vector<trimesh::vec3>& positions);

	class ScreenCameraTracer
	{
	public:
		virtual ~ScreenCameraTracer() {}

		virtual void onViewMatrixChanged(const trimesh::fxform& xform) = 0;
		virtual void onProjectionMatrixChanged(const trimesh::fxform& xform) = 0;
	};

	class ScreenCamera : public TContainer<ScreenCameraTracer>
	{
	public:
		ScreenCamera();
		~ScreenCamera();

		ScreenCameraMeta traitMeta();

		trimesh::vec3 viewCenter();
		trimesh::vec3 upVector();
		trimesh::vec3 position();
        
        void setProjectionType(ScreenCameraProjectionType projection);
		void setViewCenter(const trimesh::vec3& viewCenter);
		void setUpVector(const trimesh::vec3& upVector);
		void setPosition(const trimesh::vec3& position);
		void updateViewMatrix();

		float nearPlane();
		float farPlane();
		float fovy();
        float maxFovy();
        float minFovy();
		float aspectRatio();

		void setNearPlane(float nearPlane);
		void setFarPlane(float farPlane);
		void setFovy(float fov);
        void setMaxFovy(float maxfov);
        void setMinFovy(float minfov);
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

		void fitNotChangeDir(const trimesh::box3& box, const trimesh::box3& centerBox);
		void fittingBox(const trimesh::box3& box);
		void fittingBox(const trimesh::box3& box, const trimesh::box& centerBox, const trimesh::vec3& center);
		void adjustCamera(const trimesh::box3& box, int type);
		void setView(const trimesh::vec3& position, const trimesh::vec3& center, const trimesh::vec3& up);
		void setNearFar(float n, float f);

		bool zoom(float scale);
		bool translate(const trimesh::vec3& trans);
		void translate(const trimesh::vec2& transR);
		bool rotate(const trimesh::vec3& axis, float angle);
		bool rotate(const trimesh::quaternion& q);

		Ray screenRay(const trimesh::ivec2& point, const trimesh::ivec2& size);
		Ray screenRay(const trimesh::vec2& point);
		Ray screenRayOrthographic(const trimesh::vec2& point);
		//float screenSpaceRatio(const trimesh::vec3& position);

		trimesh::vec3 horizontal();
		trimesh::vec3 vertical();
		float verticalAngle();
		void updateNearFar(const trimesh::box3& box);

		void enableTranslate(bool enable);
		void enableRotate(bool enable);
		void enableScale(bool enable);
		bool canTranslate();
		bool canRotate();
		bool canScale();

		void viewFromBottom();
		void viewFromTop();
		void viewFromLeft();
		void viewFromRight();
		void viewFromFront();
		void viewFromBack();
		void view(const trimesh::vec3 & dir, const trimesh::vec3 & right);
		void viewFromMeta(const ScreenCameraMeta& meta);
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
        float m_maxFovy;
        float m_minFovy;
		float m_aspectRatio;

		float m_top;
		float m_bottom;
		float m_left;
		float m_right;

		ScreenCameraProjectionType m_projectionType;
		bool m_projectMatrixDirty;
		trimesh::fxform m_projectMatrix;

		int m_maniFlag;
		trimesh::box3 m_box;
		trimesh::box3 m_centerBox;
	};
}

#endif // MMESH_SCREENCAMERA_1619922752767_H
