#ifndef CREATIVE_KERNEL_DLPQUICKDATA_SIMPLE_1597825964318_H
#define CREATIVE_KERNEL_DLPQUICKDATA_SIMPLE_1597825964318_H
#include "trimesh2/TriMesh.h"
#include "trimesh2/XForm.h"
#include "mmesh/dlp/dlpheader.h"

namespace mmesh
{
	class TriangleChunk;
	class MeshTopo;
	class DLPSimpleQuickData
	{
	public:
		DLPSimpleQuickData();
		~DLPSimpleQuickData();

		void setMeshData(trimesh::TriMesh* mesh);

		void clear();
		bool check(VerticalC& point, trimesh::vec3& position);

		void autoDlpSources(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam, int flag = 7);
		void autoDlpVertexSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam);
		void autoDlpEdgeSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam);
		void autoDlpFaceSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam);
	protected:
		void build();
	protected:
		mmesh::TriangleChunk* m_triangleChunk;
		mmesh::MeshTopo* m_meshTopo;

		trimesh::TriMesh* m_mesh;

		std::vector<trimesh::box3> m_boxes;
		std::vector<trimesh::vec3> m_vertexes;
		std::vector<float> m_dotValues;
		std::vector<trimesh::vec3> m_faceNormals;
	};
}
#endif // CREATIVE_KERNEL_DLPQUICKDATA_SIMPLE_1597825964318_H