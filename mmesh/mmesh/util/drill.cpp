#include "drill.h"
#include "trimesh2/TriMesh.h"
#include "ccglobal/tracer.h"

#include "mmesh/common/print.h"
#include "mmesh/trimesh/cylindercollide.h"
#include "mmesh/trimesh/trimeshutil.h"

#include <assert.h>

namespace mmesh
{
	trimesh::TriMesh* drill(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinderMesh,
		ccglobal::Tracer* tracer, DrillDebugger* debugger)
	{
		if (tracer)
		{
			tracer->progress(0.1f);
		}
		if (!mesh || !cylinderMesh)
		{
			if (tracer)
				tracer->failed("mesh or cylinder mesh is empty.");
			
			return nullptr;
		}
		int vertexNum = mesh->vertices.size();
		int faceNum = mesh->faces.size();

		tracerFormartPrint(tracer, "drill start.  vertexNum [%d] , faceNum [%d].", vertexNum, faceNum);
		if (vertexNum == 0 || faceNum == 0 || cylinderMesh->vertices.size() == 0
			|| cylinderMesh->faces.size() == 0)
		{
			if (tracer)
				tracer->failed("mesh or cylinder (vertex, face) is empty.");
			return nullptr;
		}


		//trimesh::point pointStart;
		//trimesh::point pointEnd;
		//float radius;
		//std::string filenemae ="F:/GitSource/Gerrit_DockerViz/DockerViz/data/drill/cyPosition.txt";
		//FILE* F2 = fopen(filenemae.c_str(), "rb");
		//if (F2)
		//{

		//	float X;
		//	float Y;
		//	float Z;

		//	fread(&X, sizeof(float), 1, F2);
		//	fread(&Y, sizeof(float), 1, F2);
		//	fread(&Z, sizeof(float), 1, F2);
		//	pointStart.at(0) = X;
		//	pointStart.at(1) = Y;
		//	pointStart.at(2) = Z;

		//	fread(&X, sizeof(float), 1, F2);
		//	fread(&Y, sizeof(float), 1, F2);
		//	fread(&Z, sizeof(float), 1, F2);
		//	pointEnd.at(0) = X;
		//	pointEnd.at(1) = Y;
		//	pointEnd.at(2) = Z;
		//	fread(&radius, sizeof(float), 1, F2);
		//	fclose(F2);
		//}
		//OptimizeCylinderCollide cylinderCollider(mesh, cylinderMesh, pointStart, pointEnd, radius,tracer, debugger);

		OptimizeCylinderCollide cylinderCollider(mesh, cylinderMesh, tracer, debugger);
		if (!cylinderCollider.valid())
		{
			if (tracer)
				tracer->failed("OptimizeCylinderCollide is not valid.");

			return nullptr;
		}
		if (tracer)
		{
			tracer->progress(0.2f);
		}
		return cylinderCollider.drill(tracer);
	}

	trimesh::TriMesh* drillCylinder(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinderMesh,
		double& radius, double& depth, const trimesh::vec3& startPosition, const trimesh::vec3& endPosition,
		ccglobal::Tracer* tracer, DrillDebugger* debugger)
	{
		if (!mesh || !cylinderMesh)
		{
			if (tracer)
				tracer->failed("mesh or cylinder mesh is empty.");

			return nullptr;
		}
		int vertexNum = mesh->vertices.size();
		int faceNum = mesh->faces.size();

		tracerFormartPrint(tracer, "drill start.  vertexNum [%d] , faceNum [%d].", vertexNum, faceNum);
		if (vertexNum == 0 || faceNum == 0 || cylinderMesh->vertices.size() == 0
			|| cylinderMesh->faces.size() == 0)
		{
			if (tracer)
				tracer->failed("mesh or cylinder (vertex, face) is empty.");
			return nullptr;
		}

		OptimizeCylinderCollide cylinderCollider(mesh, cylinderMesh, radius, depth, startPosition, endPosition, tracer, debugger);

		if (!cylinderCollider.valid())
		{
			if (tracer)
				tracer->failed("OptimizeCylinderCollide is not valid.");

			return nullptr;
		}

		return cylinderCollider.drill(tracer);
	}
}