#ifndef MMESH_CYLINDERCOLLIDE_1631348831075_H
#define MMESH_CYLINDERCOLLIDE_1631348831075_H
#include "trimesh2/TriMesh.h"

namespace ccglobal
{
	class Tracer;
}

namespace mmesh
{
	struct TriTri
	{
		trimesh::vec3 v1;
		trimesh::vec3 v2;
		int topIndex;
		float d[3];

		trimesh::ivec2 index; // -1 not edge, 1 edge
	};

	struct FaceCollide
	{
		std::vector<TriTri> tris;
		int flag;
	};

	typedef std::vector<trimesh::TriMesh::Face> FacePatch;
	typedef std::vector<int> IndexPatch;
	typedef std::vector<int> FlagPatch;
	typedef std::vector<trimesh::vec3> TriPatch;

	class DrillDebugger
	{
	public:
		virtual ~DrillDebugger() {}

		virtual void onCylinderBoxFocus(FacePatch& focus) = 0;
		virtual void onMeshOuter(trimesh::TriMesh* mesh) = 0;
		virtual void onMeshCollide(trimesh::TriMesh* mesh) = 0;
		virtual void onMeshInner(trimesh::TriMesh* mesh) = 0;
	};

	//flags -1 inner, 0 collide, 1 outer
	const int CylinderCollideOuter = 1;
	const int CylinderCollideCollide = 0;
	const int CylinderCollideInner = -1;

	class OptimizeCylinderCollide
	{
	public:
		OptimizeCylinderCollide(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinder,
			ccglobal::Tracer* tracer, DrillDebugger* debugger);
		~OptimizeCylinderCollide();

		bool valid();

		trimesh::TriMesh* drill();
	protected:
		void calculate();

		trimesh::TriMesh* postProcess(trimesh::TriMesh* Mout, trimesh::TriMesh* Cin);
	protected:
		trimesh::TriMesh* m_mesh;
		trimesh::TriMesh* m_cylinder;

		int focusTriangle;
		IndexPatch meshFocusFacesMapper;
		FacePatch meshFocusFaces;
		std::vector<FaceCollide> meshTris;
		std::vector<trimesh::vec3> focusNormals;

		int cylinderTriangles;
		std::vector<FaceCollide> cylinderTris;
		std::vector<trimesh::vec3> cylinderNormals;

		std::vector<int> totalMeshFlag;
		//TriPatch newCylinderTriangles;

		ccglobal::Tracer* m_tracer;
		DrillDebugger* m_debugger;
	};
}

#endif // MMESH_CYLINDERCOLLIDE_1631348831075_H