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

	trimesh::TriMesh* drillCylinder(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinderMesh,
		const trimesh::vec3& startPosition, const trimesh::vec3& endPosition,
		ccglobal::Tracer* tracer, DrillDebugger* debugger);
}

#endif // MMESH_DRILL_1631339010743_H