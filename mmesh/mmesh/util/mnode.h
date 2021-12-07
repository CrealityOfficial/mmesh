#ifndef MMESH_MNODE_1622032440408_H
#define MMESH_MNODE_1622032440408_H
#include "trimesh2/XForm.h"
#include "trimesh2/Vec.h"
#include "trimesh2/quaternion.h"
#include "mmesh/util/tcontainer.h"
#include <list>

namespace mmesh
{
	class MNode;
	class MNodeTracer
	{
	public:
		virtual ~MNodeTracer() {}
		virtual void notifyMatrixChanged(MNode* node) = 0;
	};

	class MNode : public TContainer<MNodeTracer>
	{
	public:
		MNode();
		~MNode();

		void setParent(MNode* node);
		MNode* parent();

		trimesh::vec3 center();
		void setCenter(const trimesh::vec3& center, bool update = true);

		void setLocalScale(const trimesh::vec3& scale, bool update = true);
		void resetLocalScale(bool update = true);
		trimesh::vec3 localScale();

		trimesh::quaternion localQuaternion();
		void setLocalQuaternion(const trimesh::quaternion& q, bool update = true);
		void resetLocalQuaternion(bool update = true);

		void setLocalPosition(const trimesh::vec3& position, bool update = true);
		void resetLocalPosition(bool update = true);
		trimesh::vec3 localPosition();

		trimesh::fxform globalMatrix();
		trimesh::fxform localMatrix();

		void translate(const trimesh::vec3& t);
		void scale(const trimesh::vec3& s);

		////// matrix apply
		void applyTranslate(const trimesh::vec3& t);
		void applyScale(const trimesh::vec3& s);
		void applyRotate(const trimesh::vec3& axis, float angle, bool local);
	protected:
		void applyXf(const trimesh::fxform& xf);
	protected:
		void updateMatrix();

		void addChild(MNode* node);
		void removeChild(MNode* node);
	protected:
		trimesh::vec3 m_localCenter;

		trimesh::vec3 m_localPosition;
		trimesh::vec3 m_localScale;
		trimesh::quaternion m_localRotate;

		trimesh::fxform m_globalMatrix;
		trimesh::fxform m_localMatrix;
		bool m_localDirty;

		MNode* m_parent;
		std::list<MNode*> m_children;
		bool m_updating;
	};

	trimesh::fxform fromQuaterian(const trimesh::quaternion& q);
}

#endif // MMESH_MNODE_1622032440408_H