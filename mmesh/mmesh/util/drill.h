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

	struct DrillParam
	{
		int cylinder_resolution;
		double cylinder_radius;
		double cylinder_depth;                         	// depth 设置为小于等于 0 时，则打洞只打穿一层壁，若大于 0，则打穿指定深度内的所有壁
		trimesh::vec3 cylinder_startPos;          //  打洞起点的世界坐标
		trimesh::vec3 cylinder_Dir;
	};

	trimesh::TriMesh* drill(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinderMesh,
		ccglobal::Tracer* tracer, DrillDebugger* debugger);

	// depth 设置为小于等于 0 时，则打洞只打穿一层壁，若大于 0，则打穿指定深度内的所有壁
	trimesh::TriMesh* drillCylinder(trimesh::TriMesh* mesh, DrillParam& param, ccglobal::Tracer* tracer, DrillDebugger* debugger);
	//打印多个洞
	trimesh::TriMesh* drillCylinder(trimesh::TriMesh* mesh, std::vector<DrillParam>& params, ccglobal::Tracer* tracer, DrillDebugger* debugger);

	struct DrillInputCache
	{
		trimesh::TriMesh* mesh;
		DrillParam param;
	};

	bool saveDrill(const std::string& fileName, const DrillInputCache& cache);
	bool loadDrill(const std::string& fileName, DrillInputCache& cache);
}

#endif // MMESH_DRILL_1631339010743_H