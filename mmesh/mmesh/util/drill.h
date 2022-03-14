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
		double cylinder_depth;                         	// depth ����ΪС�ڵ��� 0 ʱ�����ֻ��һ��ڣ������� 0�����ָ������ڵ����б�
		trimesh::vec3 cylinder_startPos;          //  ��������������
		trimesh::vec3 cylinder_Dir;
	};

	trimesh::TriMesh* drill(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinderMesh,
		ccglobal::Tracer* tracer, DrillDebugger* debugger);

	// depth ����ΪС�ڵ��� 0 ʱ�����ֻ��һ��ڣ������� 0�����ָ������ڵ����б�
	trimesh::TriMesh* drillCylinder(trimesh::TriMesh* mesh, DrillParam& param, ccglobal::Tracer* tracer, DrillDebugger* debugger);
	//��ӡ�����
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