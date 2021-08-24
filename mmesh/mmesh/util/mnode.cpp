#include "mnode.h"

namespace mmesh
{
	MNode::MNode()
		: m_localDirty(true)
		, m_updating(false)
		, m_parent(nullptr)
		, m_localScale(1.0f, 1.0f, 1.0f)
	{

	}

	MNode::~MNode()
	{
		setParent(nullptr);
	}

	void MNode::setParent(MNode* node)
	{
		if (m_parent == node)
			return;

		if (m_parent)
			m_parent->removeChild(this);

		m_parent = node;

		if (m_parent)
			m_parent->addChild(this);
	}

	MNode* MNode::parent()
	{
		return m_parent;
	}

	void MNode::addChild(MNode* node)
	{
		if (!node)
			return;

		std::list<MNode*>::iterator it = std::find(m_children.begin(), m_children.end(), node);
		if (it != m_children.end())
			return;

		m_children.push_back(node);
		node->updateMatrix();
	}

	void MNode::removeChild(MNode* node)
	{
		if (!node)
			return;

		std::list<MNode*>::iterator it = std::find(m_children.begin(), m_children.end(), node);
		if (it == m_children.end())
			return;

		m_children.erase(it);
		node->updateMatrix();
	}

	trimesh::vec3 MNode::center()
	{
		return m_localCenter;
	}

	void MNode::setCenter(const trimesh::vec3& center, bool update)
	{
		m_localCenter = center;
		m_localDirty = true;

		if(update)
			updateMatrix();
	}

	void MNode::setLocalScale(const trimesh::vec3& scale, bool update)
	{
		m_localScale = scale;
		m_localDirty = true;

		if (update)
			updateMatrix();
	}

	void MNode::resetLocalScale(bool update)
	{
		setLocalScale(trimesh::vec3(1.0f, 1.0f, 1.0f), update);
	}

	trimesh::vec3 MNode::localScale()
	{
		return m_localScale;
	}

	trimesh::quaternion MNode::localQuaternion()
	{
		return m_localRotate;
	}

	void MNode::setLocalQuaternion(const trimesh::quaternion& q, bool update)
	{
		m_localRotate = q;
		m_localDirty = true;

		if (update)
			updateMatrix();
	}

	void MNode::resetLocalQuaternion(bool update)
	{
		setLocalQuaternion(trimesh::quaternion(), update);
	}

	void MNode::setLocalPosition(const trimesh::vec3& position, bool update)
	{
		m_localPosition = position;
		m_localDirty = true;

		if(update)
			updateMatrix();
	}

	void MNode::resetLocalPosition(bool update)
	{
		setLocalPosition(trimesh::vec3(0.0f, 0.0f, 0.0f), update);
	}

	trimesh::vec3 MNode::localPosition()
	{
		return m_localPosition;
	}

	void MNode::updateMatrix()
	{
		if (m_updating)
		{
			//cycle update
			return;
		}

		m_updating = true;
		if (m_localDirty)
		{
			m_localMatrix = trimesh::fxform::identity();
			trimesh::fxform t1 = trimesh::fxform::trans(m_localPosition + m_localCenter);
			trimesh::fxform r = fromQuaterian(m_localRotate);
			trimesh::fxform s = trimesh::fxform::scale(m_localScale.x, m_localScale.y, m_localScale.z);
			trimesh::fxform t2 = trimesh::fxform::trans(-m_localCenter);

			m_localMatrix = t1 * s * r * t2;
			m_localDirty = false;
		}

		m_globalMatrix = m_localMatrix;
		if (m_parent)
		{
			trimesh::fxform m = m_parent->globalMatrix();
			m_globalMatrix = m * m_globalMatrix;
		}

		for (MNode* node : m_children)
			node->updateMatrix();

		notify([this](MNodeTracer* tracer) {
			tracer->notifyMatrixChanged(this);
			});
		m_updating = false;
	}

	trimesh::fxform MNode::globalMatrix()
	{
		return m_globalMatrix;
	}

	trimesh::fxform MNode::localMatrix()
	{
		return m_localMatrix;
	}

	void MNode::translate(const trimesh::vec3& t)
	{
		setLocalPosition(m_localPosition + t);
	}

	void MNode::scale(const trimesh::vec3& s)
	{
		setLocalScale(m_localScale * s);
	}

	void MNode::applyTranslate(const trimesh::vec3& t)
	{
		translate(t);
	}

	void MNode::applyScale(const trimesh::vec3& s)
	{
		scale(s);
	}

	void MNode::applyRotate(const trimesh::vec3& axis, float angle, bool local)
	{
		trimesh::vec3 _axis = axis;
		if(!local)
		{
			_axis = m_localRotate * axis;
			trimesh::normalize(_axis);
		}
		trimesh::quaternion q = trimesh::quaternion::fromAxisAndAngle(_axis, angle);
		setLocalQuaternion(q * m_localRotate);
	}

	void MNode::applyXf(const trimesh::fxform& xf)
	{
		m_localMatrix = xf * m_localMatrix;
		updateMatrix();
	}

	trimesh::fxform fromQuaterian(const trimesh::quaternion& q)
	{
		float x2 = q.xp * q.xp;
		float y2 = q.yp * q.yp;
		float z2 = q.zp * q.zp;
		float xy = q.xp * q.yp;
		float xz = q.xp * q.zp;
		float yz = q.yp * q.zp;
		float wx = q.wp * q.xp;
		float wy = q.wp * q.yp;
		float wz = q.wp * q.zp;


		// This calculation would be a lot more complicated for non-unit length quaternions
		// Note: The constructor of Matrix4 expects the Matrix in column-major format like expected by
		//   OpenGL
		return trimesh::fxform(1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy), 0.0f,
			2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx), 0.0f,
			2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
	}
}