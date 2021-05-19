#include "space3d.h"

namespace mmesh
{
	//QVector3D point3DFromVector2D(const QVector2D& point, const QVector2D& center, const QVector2D& size, bool skipz)
	//{
	//	return point3DFromVector2D(point, center, size.x(), size.y(), skipz);
	//}
	//
	//QVector3D point3DFromVector2D(const QVector2D& point, const QVector2D& center, float width, float height, bool skipz)
	//{
	//	QVector3D pt3D;
	//
	//	float dx = point.x() - center.x();
	//	float dy = point.y() - center.y();
	//
    //    float width2 = qMax(qFabs(dx), qMin(qFabs(width - center.x()), qFabs(center.x())));
    //    float height2 = qMax(qFabs(dy), qMin(qFabs(height - center.y()), qFabs(center.y())));
	//
    //    float maxDim = qMax(width2, height2);
	//
	//	pt3D[0] = dx / maxDim;
	//	pt3D[1] = dy / maxDim;
	//
	//	if (skipz)
	//	{
	//		pt3D[2] = 0.0f;
	//		pt3D.normalize();
	//	}
	//	else
	//	{
	//		/// Sphere equation : (x-x_o)^2 + (y-y_o)^2 + (z-z_o)^2 = R^2
	//		/// we take Center O(x_o=0, y_o=0, z_o=0) and R = 1
	//
	//		float x2_plus_y2 = (pt3D.x() * pt3D.x() + pt3D.y() * pt3D.y());
	//		if (x2_plus_y2 > 1.0f)
	//		{
	//			pt3D.setZ(0.0f);
	//
	//			// fast normalization
	//			float length = sqrt(x2_plus_y2);
	//			pt3D[0] /= length;
	//			pt3D[1] /= length;
	//		}
	//		else
	//		{
	//			pt3D[2] = sqrt(1.0f - x2_plus_y2);
	//		}
	//	}
	//
	//	return pt3D;
	//}
	//
	//void boxFittingBox(const Box3D& baseBounding, const Box3D& initBox, QVector3D& translate, QVector3D& scale)
	//{
	//	QVector3D baseSize = baseBounding.size();
	//	QVector3D initSize = initBox.size();
	//
	//	QVector3D ratio = baseSize / initSize;
	//	float fscale = qMin(ratio.x(), qMin(ratio.y(), ratio.z()));
	//
	//	scale = QVector3D(fscale, fscale, fscale);
	//
	//	QVector3D corner = initBox.center() - 0.5f * scale * initBox.size();
	//	translate = baseBounding.min - corner;
	//}
	//
	//Box3D transformBox(const QMatrix4x4& matrix, const Box3D& box)
	//{
	//	Box3D tbox;
	//	if (box.valid)
	//	{
	//		QVector3D bmin = box.min;
	//		QVector3D bmax = box.max;
	//
	//		tbox += matrix * QVector3D(bmin.x(), bmin.y(), bmin.z());
	//		tbox += matrix * QVector3D(bmin.x(), bmin.y(), bmax.z());
	//		tbox += matrix * QVector3D(bmin.x(), bmax.y(), bmax.z());
	//		tbox += matrix * QVector3D(bmin.x(), bmax.y(), bmin.z());
	//		tbox += matrix * QVector3D(bmax.x(), bmin.y(), bmin.z());
	//		tbox += matrix * QVector3D(bmax.x(), bmin.y(), bmax.z());
	//		tbox += matrix * QVector3D(bmax.x(), bmax.y(), bmax.z());
	//		tbox += matrix * QVector3D(bmax.x(), bmax.y(), bmin.z());
	//	}
	//
	//	return tbox;
	//}

	bool lineCollidePlane(const trimesh::vec3& planeCenter, const trimesh::vec3& planeDir, const Ray& ray, trimesh::vec3& collide)
	{
		float l = ray.dir DOT planeDir;
		if (l == 0.0f) return false;

		float t = (planeDir DOT (planeCenter - ray.start)) / l;
		collide = ray.start + ray.dir * t;

		return true;
	}
}
