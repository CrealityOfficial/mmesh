#ifndef _MMESH_SPACE3D_1588841647525_H
#define _MMESH_SPACE3D_1588841647525_H
#include "trimesh2/Vec.h"
#include "mmesh/camera/ray.h"

//#include "qtuser3d/math/box3d.h"
//#include <QtGui/QVector2D>
//#include <QtGui/QMatrix4x4>

namespace mmesh
{
	//QVector3D point3DFromVector2D(const QVector2D& point, const QVector2D& center, const QVector2D& size, bool skipz);

	//QVector3D point3DFromVector2D(const QVector2D& point, const QVector2D& center, float width, float height, bool skipz);

	//void boxFittingBox(const Box3D& baseBounding, const Box3D& initBox, QVector3D& translate, QVector3D& scale);

	//Box3D transformBox(const QMatrix4x4& matrix, const Box3D& box);

	bool lineCollidePlane(const trimesh::vec3& planeCenter, const trimesh::vec3& planeDir, const mmesh::Ray& ray, trimesh::vec3& collide);
}
#endif // _MMESH_SPACE3D_1588841647525_H
