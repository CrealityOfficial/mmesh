#ifndef MMESH_CYLINDERCOLLIDE_1631348831075_H
#define MMESH_CYLINDERCOLLIDE_1631348831075_H
#include "trimesh2/TriMesh.h"
#include "mmesh/trimesh/trianglesplit.h"

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
		bool topPositive;

		trimesh::ivec2 index; // -1 not edge, 1 edge

		//cylinder 
		trimesh::ivec2 cindex;
		int cylinderTopIndex;
		bool cylinderTopPositive;
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
		virtual void onTriangleSplitCache(const SplitTriangleCache& cache) = 0;
	};

	//flags -1 inner, 0 collide, 1 outer
	const int CylinderCollideOuter = 1;
	const int CylinderCollideCollide = 0;
	const int CylinderCollideInner = -1;
	const int CylinderCollideInvalid = -2;

	class OptimizeCylinderCollide
	{
	public:
		OptimizeCylinderCollide(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinder,
			ccglobal::Tracer* tracer, DrillDebugger* debugger);
			
		// depth ����ΪС�ڵ��� 0 ʱ�����ֻ��һ��ڣ������� 0�����ָ������ڵ����б�
		OptimizeCylinderCollide(trimesh::TriMesh* mesh,
			int resolution, double radius, double depth, trimesh::point pointStart, trimesh::point dir,
			ccglobal::Tracer* tracer, DrillDebugger* debugger);
		~OptimizeCylinderCollide();

		bool valid();

		trimesh::TriMesh* drill(ccglobal::Tracer* tracer);
	protected:
		void calculate();
		void mycalculate();

		double getDrillDepth();
		bool lineCollideTriangle(trimesh::dvec3 linePos, trimesh::dvec3 lineDir, trimesh::dvec3 A, trimesh::dvec3 B, trimesh::dvec3 C, trimesh::dvec3& intersectedPos);
		trimesh::TriMesh* createCylinderMesh(trimesh::vec3 top, trimesh::vec3 bottom, float radius, int num = 20, float theta = 0.0);

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
		std::vector<int> cylinderFlag;

		ccglobal::Tracer* m_tracer;
		DrillDebugger* m_debugger;

		int m_cylinderResolution;
		double m_cylinderRadius;
		double m_cylinderDepth;
		trimesh::point m_cylinderPointStart; 
		trimesh::point m_cylinderDir; 
	};
}

#endif // MMESH_CYLINDERCOLLIDE_1631348831075_H