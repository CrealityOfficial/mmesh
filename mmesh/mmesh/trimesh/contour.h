#ifndef MMESH_CONTOUR_1603376186170_H
#define MMESH_CONTOUR_1603376186170_H
#include "trimesh2/TriMesh.h"
#include "trimesh2/Box.h"

namespace mmesh
{
	void zoff(trimesh::vec3* svertex, trimesh::vec3* dvertex, size_t size, float z);
	void fillLoop(int sindex1, int sindex2, int size, trimesh::TriMesh::Face* faces);

	void sortBoxes(const trimesh::box3& box, const std::vector<trimesh::box3>& boxes, 
		float gap,std::vector<trimesh::vec3>& newPosition);
	void sortBoxes(const trimesh::box3& box, const std::vector<trimesh::box3>& boxes
		,float gapx,float gapy,std::vector<trimesh::vec3>& newPosition);
	void sortBoxes(const trimesh::box3& box, const std::vector<trimesh::box3>& boxes,
		std::vector<trimesh::vec3>& newPosition);

	void loopPolygons2Lines(const std::vector<std::vector<trimesh::vec3>>& polygons, std::vector<trimesh::vec3>& lines, bool loop = true);
	void loopPolygon2Lines(const std::vector<trimesh::vec3>& polygon, std::vector<trimesh::vec3>& lines, bool loop = true, bool append = false);
}

#endif // MMESH_CONTOUR_1603376186170_H