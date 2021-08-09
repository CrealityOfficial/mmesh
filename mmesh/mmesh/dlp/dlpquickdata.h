#ifndef MMESH_DLPQUICKDATA_1613700735863_H
#define MMESH_DLPQUICKDATA_1613700735863_H
#include "dlpheader.h"
#include "trimesh2/TriMesh.h"
#include "trimesh2/XForm.h"
#include <vector>
#include <functional>
namespace mmesh
{
	class TriangleChunk;
	class MeshTopo;
	struct AutoDLPSupportParam;
	struct DLPSupportParam;
	enum SupportType
	{
		SUPPORT_NO=0,
		SUPPORT_VERTEX=1,
		SUPPORT_EDGE=1<<1,
		SUPPORT_FACE=1<<2,
	};
	typedef struct Connect_Section_infor
	{
		std::vector<std::vector<int>> facesIndexes;
		std::vector<std::vector<int>> edgeFaces;
		std::vector < trimesh::vec3> centerPoint;
		std::vector < trimesh::box3> bBoxes;
	}ConnectSectionInfor;
	class CallBackParamsBase{
	public:
		CallBackParamsBase();
		 ~CallBackParamsBase();
	public:
		void* obj;
	};
	class CallBackParams:public  CallBackParamsBase {
	public:
		CallBackParams();
		~CallBackParams();
	public:
		float percentage;
	};

	class DLPQuickData
	{
	public:
		DLPQuickData();
		virtual ~DLPQuickData();

		void setMeshData(trimesh::TriMesh* mesh);
		void setXformData(trimesh::fxform& xf);
		trimesh::fxform getXformData() const;

		void build();
		void clear();
		bool check(VerticalC& point, trimesh::vec3& position);

		bool dirty() const;

		void autoDlpSources(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam, int flag= SUPPORT_VERTEX| SUPPORT_EDGE| SUPPORT_FACE, std::function<void(CallBackParams *)> callback=NULL, CallBackParams* cbParams=NULL);//7
		void autoDlpVertexSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam);
		void autoDlpEdgeSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam);
		void autoDlpFaceSource(std::vector<std::vector<DLPISource>> &sectionSources, AutoDLPSupportParam* autoParam);
		void extractFaceSectEdge(std::vector<std::vector<int>> SupportFaces, std::vector<std::vector<trimesh::vec3>>& edgeSectVertexs);
		bool sectionFaceNeedSupport(std::vector<int> SupportFaces, std::vector<int>& edgeFaces, trimesh::vec3& farPoint, trimesh::vec3& centerPoint);
		void searchSectionFaceEdgeFace(std::vector<int> SupportFaces, std::vector<int> & edgeFaces, float& sectionFaceArea);
	protected:
		bool autoTest(const trimesh::vec3& point);
		void takeAutoTest(const trimesh::vec3& point);
	private:
		CallBackParams* m_cbParamsPtr;
		std::function<void(CallBackParams*)> m_throwFunc;
		void dlpSourceCheck(std::vector<DLPISource> &SupportSources, DLPISources& clusteredSources);

	protected:
		mmesh::TriangleChunk* m_triangleChunk;
		mmesh::MeshTopo* m_meshTopo;

		trimesh::TriMesh* m_mesh;
		trimesh::fxform m_xf;
		trimesh::fxform m_invXf;
		bool m_dirty;

		std::vector<trimesh::box3> m_boxes;
		std::vector<trimesh::vec3> m_vertexes;
		std::vector<float> m_dotValues;
		std::vector<trimesh::vec3> m_faceNormals;

		trimesh::box3 m_xyBox;
		float m_pixel;
		int m_width;
		int m_height;
		

		std::vector<std::vector<trimesh::vec3>> m_VertexSampleInCell;
		std::vector<DLPISource> m_SupportFaceSources;
		std::vector<DLPISource> m_SupportEdgeSources;
		std::vector<DLPISource> m_SupportVertexSources;
		bool m_DLPISourceInited;
		AutoDLPSupportParam m_autoParam;
		ConnectSectionInfor m_ConnectSectionInfor;
	private:
		std::vector<int> m_supportFace;

	};
}

#endif // MMESH_DLPQUICKDATA_1613700735863_H