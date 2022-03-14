#include "drill.h"
#include "trimesh2/TriMesh.h"
#include "ccglobal/tracer.h"

#include "mmesh/trimesh/cylindercollide.h"
#include "mmesh/trimesh/trimeshutil.h"

#include <assert.h>
#include <fstream>

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

		SAFE_TRACER(tracer, "drill start.  vertexNum [%d] , faceNum [%d].", vertexNum, faceNum);
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
		if (tracer)
		{
			tracer->progress(0.2f);
		}
		return cylinderCollider.drill(tracer);
	}

	trimesh::TriMesh* drillCylinder(trimesh::TriMesh* mesh, DrillParam& param, ccglobal::Tracer* tracer, DrillDebugger* debugger)
	{
		if (!mesh)
		{
			if (tracer)
				tracer->failed("model mesh is empty.");

			return nullptr;
		}
		int vertexNum = mesh->vertices.size();
		int faceNum = mesh->faces.size();

		SAFE_TRACER(tracer, "drill start.  vertexNum [%d] , faceNum [%d].", vertexNum, faceNum);
		if (vertexNum == 0 || faceNum == 0)
		{
			if (tracer)
				tracer->failed("model mesh (vertex, face) is empty.");
			return nullptr;
		}

		OptimizeCylinderCollide cylinderCollider(mesh, param.cylinder_resolution, param.cylinder_radius, param.cylinder_depth, param.cylinder_startPos, param.cylinder_Dir, tracer, debugger);

		if (!cylinderCollider.valid())
		{
			if (tracer)
				tracer->failed("Depth Error or OptimizeCylinderCollide is not valid.");

			return nullptr;
		}

		return cylinderCollider.drill(tracer);
	}
	trimesh::TriMesh* drillCylinder(trimesh::TriMesh* mesh, std::vector<DrillParam>& params, ccglobal::Tracer* tracer, DrillDebugger* debugger)
	{
		if (!mesh)
		{
			if (tracer)
				tracer->failed("model mesh is empty.");
			return nullptr;
		}
		if (params.size() == 0) return nullptr;
		std::unique_ptr<trimesh::TriMesh> drillMesh(mesh);
		for (auto& param : params)
		{
			drillMesh.reset(drillCylinder(drillMesh.get(), param, tracer, debugger));
			if (!drillMesh) return nullptr;
		}
		return drillMesh.release();
	}

	bool saveDrill(const std::string& fileName, const DrillInputCache& cache)
	{
		if (!cache.mesh)
			return false;

		std::fstream out(fileName, std::ios::out | std::ios::binary);
		if (out.is_open())
		{
			saveTrimesh(out, *cache.mesh);
			saveT(out, cache.param);
		}

		out.close();
		return true;
	}

	bool loadDrill(const std::string& fileName, DrillInputCache& cache)
	{
		std::fstream in(fileName, std::ios::in | std::ios::binary);
		if (in.is_open())
		{
			cache.mesh = new trimesh::TriMesh();
			loadTrimesh(in, *cache.mesh);
			loadT(in, cache.param);
		}

		in.close();
		return true;
	}
}