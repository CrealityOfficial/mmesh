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
		
		OptimizeCylinderCollide cylinderCollider(mesh, cylinderMesh, tracer, debugger);
		if (!cylinderCollider.valid())
		{
			if (tracer)
				tracer->failed("OptimizeCylinderCollide is not valid.");

			return nullptr;
		}

		return cylinderCollider.drill();
	}
}