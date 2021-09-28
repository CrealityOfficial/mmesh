#ifndef MMESH_DRILL_1631339010743_H
#define MMESH_DRILL_1631339010743_H
#include "trimesh2/TriMesh.h"
#include <vector>

namespace ccglobal
{
	class Tracer;
}

namespace mmesh
{
	class DrillDebugger;
	trimesh::TriMesh* drill(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinderMesh,
		ccglobal::Tracer* tracer, DrillDebugger* debugger);

	// depth 设置为小于等于 0 时，则打洞只打穿一层壁，若大于 0，则打穿指定深度内的所有壁
	trimesh::TriMesh* drillCylinder(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinderMesh,
		double& radius, double& depth, const trimesh::vec3& startPosition, const trimesh::vec3& endPosition,
		ccglobal::Tracer* tracer, DrillDebugger* debugger);
}

#endif // MMESH_DRILL_1631339010743_H