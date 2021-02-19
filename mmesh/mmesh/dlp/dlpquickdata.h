#ifndef MMESH_DLPQUICKDATA_1613700735863_H
#define MMESH_DLPQUICKDATA_1613700735863_H
#include "dlpheader.h"
#include "trimesh2/TriMesh.h"
#include "trimesh2/XForm.h"
#include <vector>

namespace mmesh
{
	class TriangleChunk;
	class MeshTopo;
	struct AutoDLPSupportParam;
	struct DLPSupportParam;
	class DLPQuickData
	{
	public:
		DLPQuickData();
		virtual ~DLPQuickData();

		void setMeshData(trimesh::TriMesh* mesh);
		void setXformData(trimesh::fxform& xf);

		void build();
		void clear();
		bool check(VerticalC& point, trimesh::vec3& position);

		bool dirty() const;

		void autoDlpSources(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam, DLPSupportParam* supportParam, int flag = 7);
		void autoDlpVertexSource(std::vector<DLPISource>& sources);
		void autoDlpEdgeSource(std::vector<DLPISource>& sources);
		void autoDlpFaceSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam);
	protected:
		bool autoTest(const trimesh::vec3& point);
		void takeAutoTest(const trimesh::vec3& point);
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
		std::vector<bool> m_flags;
	};
}

#endif // MMESH_DLPQUICKDATA_1613700735863_H