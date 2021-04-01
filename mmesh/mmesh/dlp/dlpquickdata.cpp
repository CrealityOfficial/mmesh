#include "dlpquickdata.h"

#include "mmesh/trimesh/trianglechunk.h"
#include "mmesh/trimesh/meshtopo.h"
#include "mmesh/trimesh/algrithm3d.h"

#include "dlpcreatepolicy.h"
#include <algorithm>
using namespace trimesh;
namespace mmesh
{
static const trimesh::vec3 DOWN_NORMAL = trimesh::vec3(0.0, 0.0, -1.0);
static const trimesh::vec3 UP_NORMAL = trimesh::vec3(0.0, 0.0, 1.0);
static constexpr float EPSILON = 1e-4;
	CallBackParamsBase::CallBackParamsBase()
	{
		obj = NULL;
	}
	CallBackParamsBase::~CallBackParamsBase()
	{

	}
	CallBackParams::CallBackParams()
	{
		
	}
	CallBackParams::~CallBackParams()
	{

	}
	DLPQuickData::DLPQuickData()
		: m_dirty(true)
		, m_mesh(nullptr)
		, m_pixel(1.0f)
	{
		m_triangleChunk = new TriangleChunk();
		m_meshTopo = new MeshTopo();
		m_throwFunc=NULL;
	}

	DLPQuickData::~DLPQuickData()
	{
		delete m_triangleChunk;
		delete m_meshTopo;
	}

	void DLPQuickData::setMeshData(trimesh::TriMesh* mesh)
	{
		m_mesh = mesh;
		clear();
		m_dirty = true;
	}

	void DLPQuickData::setXformData(trimesh::fxform& xf)
	{
		m_xf = xf;
		m_invXf = trimesh::inv(xf);
		m_dirty = true;
		clear();
	}

	void DLPQuickData::clear()
	{
		m_boxes.clear();
		m_vertexes.clear();
		m_dotValues.clear();
		m_faceNormals.clear();
	}

	void DLPQuickData::build()
	{
		if (!m_dirty || !m_mesh)
			return;

		int vertexNum = (int)m_mesh->vertices.size();
		int faceNum = (int)m_mesh->faces.size();

		if (vertexNum > 0 && faceNum > 0)
		{
			m_vertexes.resize(vertexNum);
			for (int i = 0; i < vertexNum; ++i)
			{
				m_vertexes.at(i) = m_xf * m_mesh->vertices.at(i);
			}

			m_dotValues.resize(faceNum);
			m_faceNormals.resize(faceNum);
			for (int i = 0; i < faceNum; ++i)
			{
				TriMesh::Face& f = m_mesh->faces.at(i);
				vec3& v0 = m_vertexes.at(f[0]);
				vec3& v1 = m_vertexes.at(f[1]);
				vec3& v2 = m_vertexes.at(f[2]);

				vec3 v01 = v1 - v0;
				vec3 v02 = v2 - v0;
				vec3 n = v01 TRICROSS v02;
				normalize(n);

				m_faceNormals.at(i) = n;
				m_dotValues.at(i) = trimesh::dot(n, vec3(0.0f, 0.0f, 1.0f));
			}

			m_boxes.resize(faceNum);
			box3 globalBox;
			for (int i = 0; i < faceNum; ++i)
			{
				TriMesh::Face& f = m_mesh->faces.at(i);
				box3& b = m_boxes.at(i);
				for (int j = 0; j < 3; ++j)
				{
					b += m_vertexes.at(f[j]);
				}

				globalBox += b;
			}

			m_triangleChunk->build(m_boxes, globalBox, 2.0f);
			m_meshTopo->build(m_mesh);

			trimesh::vec3 size = globalBox.size();
			m_xyBox = globalBox;
			m_xyBox += 0.1f * size;
			m_xyBox += -0.1f * size;
		}

		m_dirty = false;
	}

	bool DLPQuickData::dirty() const
	{
		return m_dirty;
	}

	bool DLPQuickData::check(VerticalC& point, trimesh::vec3& position)
	{
		build();

		point.platform = true;
		point.position = position;
		point.position.z = 0.0f;
		point.indexCells = -1;

		ivec2 pointIndex = m_triangleChunk->index(position);
		if (m_triangleChunk->contain(pointIndex))
		{
			vec3 dir = DOWN_NORMAL;
			int index = -1;
			point.indexCells = pointIndex.x + pointIndex.y * m_triangleChunk->m_width;
			std::vector<int>& cells = m_triangleChunk->cell(pointIndex);
			for (int i : cells)
			{
				TriMesh::Face& face = m_mesh->faces.at(i);
				vec3& vertex0 = m_vertexes.at(face[0]);
				vec3& vertex1 = m_vertexes.at(face[1]);
				vec3& vertex2 = m_vertexes.at(face[2]);

				float t, u, v;
				if (rayIntersectTriangle(position, dir, vertex0, vertex1, vertex2, &t, &u, &v) && t > 0.0f)
				{
					vec3 c = position + t * dir;
					if ((fabs(c.z - position.z) > EPSILON) && (point.position.z < c.z))
					{
						point.position = c;
						index = i;
						break;
					}
				}
			}

			if (index < 0)
			{
				return true;
			}

			if (point.position.z > 0.0f && index >= 0 && (m_dotValues.at(index) > 0.0f))
			{
				point.platform = false;
				return true;
			}

			return false;
		}

		return true;
	}
	void DLPQuickData::autoDlpSources(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam, int flag,std::function<void(CallBackParams *)> callback, CallBackParams *cbParams)
	{
		std::vector<DLPISource> m_SupportFaceSources;
		std::vector<DLPISource> m_SupportEdgeSources;
		std::vector<DLPISource> m_SupportVertexSources;
		m_throwFunc = callback;
		m_cbParamsPtr = cbParams;
		build();
		if (m_throwFunc != NULL)
		{
			m_cbParamsPtr->percentage = 10.0;
			m_throwFunc(m_cbParamsPtr);

		}
		if ((flag & SUPPORT_VERTEX)== SUPPORT_VERTEX)
		{
			autoDlpVertexSource(m_SupportVertexSources, autoParam);
		}
		if ((flag & SUPPORT_FACE)== SUPPORT_FACE)
		{
			autoDlpFaceSource(m_SupportFaceSources, autoParam);
		}

		if ((flag & SUPPORT_EDGE) == SUPPORT_EDGE)
		{
			autoDlpEdgeSource(m_SupportEdgeSources, autoParam);
		}

		if ((flag & SUPPORT_VERTEX) == SUPPORT_VERTEX)
		{

			for (DLPISource src : m_SupportVertexSources)
			{
				sources.emplace_back(src);
			}
		}
		if ((flag & SUPPORT_EDGE) == SUPPORT_EDGE)
		{

			for (DLPISource src : m_SupportEdgeSources)
			{
				sources.emplace_back(src);
			}
		}
		if ((flag & SUPPORT_FACE) == SUPPORT_FACE)
		{

			for (DLPISource src : m_SupportFaceSources)
			{
				sources.emplace_back(src);
			}
		}
		/////
		if(1)
		{
			//去除相同点、邻近点
			auto cmp_elements_sort = [](const DLPISource& e1, const DLPISource& e2) -> bool {
				return e1.position < e2.position;

			};
			auto cmp_elements_unique = [](const DLPISource& e1, const DLPISource& e2)->bool {
				constexpr float EPSILON = 1e-4;

				return trimesh::dist(e1.position, e2.position) < EPSILON;
			};
			std::vector<DLPISource>::iterator it;
			std::sort(sources.begin(), sources.end(), cmp_elements_sort);
			it = std::unique(sources.begin(), sources.end(), cmp_elements_unique);
			sources.resize(std::distance(sources.begin(), it));
		}
		/////将支撑点进行网格映射
		DLPISources clusteredSource;
		dlpSourceCheck(sources, clusteredSource);
		///
		 sources.clear();
		//ClusterEl filtered_indices;
		for (auto& a : clusteredSource.clusteredSrcID) {
			// Here we keep only the front point of the cluster.
			//filtered_indices.emplace_back(a.front());
			sources.emplace_back(clusteredSource.sources[a.front()]);
		}

		if (m_throwFunc != NULL)
		{
			m_cbParamsPtr->percentage = 100.0;
			m_throwFunc(m_cbParamsPtr);

		}
	}

	void DLPQuickData::autoDlpVertexSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam)
	{
		std::vector<int> supportVertexes;
		float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);
		m_meshTopo->lowestVertex(m_vertexes, supportVertexes);
		int  percentage_Count = 0;
		for (int vertexID : supportVertexes)
		{
			std::vector<int>& vertexHalfs = m_meshTopo->m_outHalfEdges.at(vertexID);
			int halfSize = (int)vertexHalfs.size();
			vec3 dir(0.0f, 0.0f, 0.0f);
			for (int halfIndex = 0; halfIndex < halfSize; ++halfIndex)
			{
				int indexFace = m_meshTopo->faceid(vertexHalfs.at(halfIndex));
				dir += m_faceNormals[indexFace];
				//TriMesh::Face& f = m_mesh->faces.at(indexFace);
				//bool faceSupport = m_dotValues.at(indexFace) < (-faceCosValue);
				//if ((m_SupportFacesFlg[indexFace] == false)&&(m_SupportEdgeFlg[vertexID]==false))//包含该点的面不是支撑面
			}
			{

				vec3 vertex = m_vertexes.at(vertexID);
				ivec2 pointIndex = m_triangleChunk->index(vertex);
				int iindex = pointIndex.x + pointIndex.y * m_triangleChunk->m_width;
				std::vector<int>& faceVindex = m_triangleChunk->m_cells.at(iindex);
				int faceVindexSize = faceVindex.size();
				int crossn = 0;
				for(int i=0;i< faceVindexSize;i++)
				{
					//if (indexFace == faceVindex.at(i))
					//	continue;
					TriMesh::Face& tFace = m_mesh->faces.at(faceVindex.at(i));
					vec3& vertex1 = m_vertexes.at(tFace[0]);
					vec3& vertex2 = m_vertexes.at(tFace[1]);
					vec3& vertex3 = m_vertexes.at(tFace[2]);

					vec3 dir=DOWN_NORMAL;
					float t, u, v;
					if (vertex.z > vertex1.z || vertex.z > vertex2.z || vertex.z > vertex3.z)
					{
						if (rayIntersectTriangle(vertex, dir, vertex1, vertex2, vertex3, &t, &u, &v))
						{
							crossn += 1;
						}
					}
				}
				if (crossn % 2 == 0)//向下应与模型有偶数个效点
				{
					//DLPISource dlpSource = generateSource(vertex, vec3(0.0f, 0.0f, -1.0f));
					DLPISource dlpSource = generateSource(vertex, dir);
					dlpSource.typeflg = SUPPORT_VERTEX;
					sources.push_back(dlpSource);

				}


			}
		}
		percentage_Count++;

		if ((m_throwFunc != NULL) && (percentage_Count % 100 == 0))
		{
			m_cbParamsPtr->percentage = m_cbParamsPtr->percentage + (float)percentage_Count / (float)supportVertexes.size() * 10;
			m_throwFunc(m_cbParamsPtr);
		}

	}

	void DLPQuickData::autoDlpEdgeSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam)
	{
		std::vector<ivec2> supportEdges;
		float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);

		m_meshTopo->hangEdge(m_vertexes, m_faceNormals, m_dotValues, faceCosValue, supportEdges);

		float minDelta = m_triangleChunk->m_gridSize;

		int percentage_Count = 0;
		for (ivec2 vertexesID : supportEdges)
		{
			int vertexID1 = vertexesID.x;
			int vertexID2 = vertexesID.y;
			vec3& vertex1 = m_vertexes.at(vertexID1);
			vec3& vertex2 = m_vertexes.at(vertexID2);
			vec3 vertex21 = vertex2- vertex1;
			////////////////////////////////////////
			float edgeLenXY = trimesh::len(vertex21);
			if (edgeLenXY < minDelta)//长度小于自支撑长度
				continue;

			#if 1
			int sampleEdgeNum = ceilf(edgeLenXY / minDelta) +1;
			if (sampleEdgeNum > 0)
			{
				float dt = 1.0f / (float)sampleEdgeNum ;
				for (int sampleIndex = 0; sampleIndex <= sampleEdgeNum; sampleIndex++)
				{
					float t = sampleIndex  * dt;
					vec3 samplePoint = vertex1 * (1.0f - t) + vertex2 * t;
					DLPISource dlpSource = generateSource(samplePoint, DOWN_NORMAL);
					dlpSource.typeflg = SUPPORT_EDGE;
					sources.push_back(dlpSource);
				}
			}
			#endif
			//std::vector<int> NearFaceIDIgnored;
			//{
			//	std::vector<int> supportVertexes;
			//	supportVertexes.emplace_back(vertexID1);
			//	supportVertexes.emplace_back(vertexID2);
			//	for (int vertexID : supportVertexes)
			//	{
			//		std::vector<int>& vertexHalfs = m_meshTopo->m_outHalfEdges.at(vertexID);
			//		int halfSize = (int)vertexHalfs.size();
			//		for (int halfIndex = 0; halfIndex < halfSize; ++halfIndex)
			//		{
			//			int indexFace = m_meshTopo->faceid(vertexHalfs.at(halfIndex));
			//			NearFaceIDIgnored.emplace_back(indexFace);
			//		}
			//	}

			//}

			percentage_Count++;

			if ((m_throwFunc != NULL) && (percentage_Count % 100 == 0))
			{
				m_cbParamsPtr->percentage = m_cbParamsPtr->percentage + (float)percentage_Count / (float)supportEdges.size() * 10;
				m_throwFunc(m_cbParamsPtr);
			}

		}
	}

void DLPQuickData::autoDlpFaceSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam)
{
	float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);
	size_t count = sources.size();

	std::vector<std::vector<int>> supportFaces;
	m_meshTopo->chunkFace(m_dotValues, supportFaces, faceCosValue);
	int percentage_Count = 0;
	for (std::vector<int>& faceChunk : supportFaces)
	{
		//std::sort(faceChunk.begin(), faceChunk.end(), [this](int r1, int r2)->bool {
		//	return m_dotValues.at(r1) < m_dotValues.at(r2);
		//	});
			//if (testindex != 1)
			//	continue;
			//testindex++;
		trimesh::box3 connectSectBox;
		for (int faceID : faceChunk)
		{
			TriMesh::Face& tFace = m_mesh->faces.at(faceID);
			vec3& vertex1 = m_vertexes.at(tFace[0]);
			vec3& vertex2 = m_vertexes.at(tFace[1]);
			vec3& vertex3 = m_vertexes.at(tFace[2]);
			connectSectBox += vertex1;
			connectSectBox += vertex2;
			connectSectBox += vertex3;
		}
		vec3 SectBoxSize =connectSectBox.size();
		float gridSize = m_triangleChunk->m_gridSize;
		vec2 xypointMin;
		xypointMin.x = connectSectBox.min.x;
		xypointMin.y = connectSectBox.min.y;
		int sampleXn = ceilf((connectSectBox.max.x - connectSectBox.min.x) / gridSize) + 1;
		int sampleYn = ceilf((connectSectBox.max.y - connectSectBox.min.y) / gridSize) + 1;
		float deltx = (connectSectBox.max.x - connectSectBox.min.x) / sampleXn;
		float delty = (connectSectBox.max.y - connectSectBox.min.y) / sampleYn;
		//if ((abs(SectBoxSize.x- SectBoxSize.y)< gridSize*2)&&(sampleXn<3|| sampleYn <3))
		//	continue;
		for (int faceID : faceChunk)
		{
		TriMesh::Face& tFace = m_mesh->faces.at(faceID);
		vec3& vertex1 = m_vertexes.at(tFace[0]);
		vec3& vertex2 = m_vertexes.at(tFace[1]);
		vec3& vertex3 = m_vertexes.at(tFace[2]);
		vec3 normal = m_faceNormals.at(faceID);
		#if 0
		samplePoints.push_back(vertex1);
		samplePoints.push_back(vertex2);
		samplePoints.push_back(vertex3);
		sampleNormals.push_back(normal);
		sampleNormals.push_back(normal);
		sampleNormals.push_back(normal);
		#endif



			for (int sampleXindex = 0; sampleXindex < sampleXn; sampleXindex++)
			{
				float tx = xypointMin.x + sampleXindex * deltx;
				for (int sampleYindex = 0; sampleYindex < sampleYn; sampleYindex++)
				{
					float ty = xypointMin.y + sampleYindex * delty;
					vec3 xypoint0(tx, ty,0.0);
					vec3 dir=UP_NORMAL;
					float t, u, v;
					if (rayIntersectTriangle(xypoint0, dir,  vertex1, vertex2,vertex3, &t, &u, &v))
					{
						vec3 pointCross = xypoint0 + t * dir;
						DLPISource dlpSource = generateSource(pointCross, normal);
						dlpSource.typeflg = SUPPORT_FACE;
						sources.push_back(dlpSource);
					}
				}
			}

		}

		percentage_Count++;

		if ((m_throwFunc != NULL) && (percentage_Count % 100 == 0))
		{
			m_cbParamsPtr->percentage = m_cbParamsPtr->percentage + (float)percentage_Count / (float)supportFaces.size() * 50;
			m_throwFunc(m_cbParamsPtr);
		}

	}
}

bool DLPQuickData::autoTest(const trimesh::vec3& point)
{
	std::vector<bool> m_flags;
	int nx = (int)floorf((point.x - m_xyBox.min.x) / m_pixel);
	int ny = (int)floorf((point.y - m_xyBox.min.y) / m_pixel);

	if (nx >= 0 && nx < m_width && ny >= 0 && ny < m_height)
	{
		int iindex = nx + ny * m_width;
		return m_flags.at(iindex);
	}

	return true;


}

void DLPQuickData::takeAutoTest(const trimesh::vec3& point)
{
	std::vector<bool> m_flags;
	int nx = (int)floorf((point.x - m_xyBox.min.x) / m_pixel);
	int ny = (int)floorf((point.y - m_xyBox.min.y) / m_pixel);

	if (nx >= 0 && nx < m_width && ny >= 0 && ny < m_height)
	{
		int iindex = nx + ny * m_width;
		m_flags.at(iindex) = false;
	}
}


void DLPQuickData::extractFaceSectEdge(std::vector<std::vector<int>> SupportFaces, std::vector<std::vector<trimesh::vec3>> &edgeSectVertexs)
{
	edgeSectVertexs.resize(SupportFaces.size());
	for (int faceChunkIndex =0; faceChunkIndex<SupportFaces.size(); faceChunkIndex++)
	{
		std::vector<int> edgeFaces;
		std::vector<int>& faceChunk = SupportFaces.at(faceChunkIndex);
		for (int faceID : faceChunk)
		{
			ivec3& oppoHalfs = m_meshTopo->m_oppositeHalfEdges.at(faceID);
			for (int halfID = 0; halfID < 3; ++halfID)
			{
				int oppoHalf = oppoHalfs.at(halfID);
				if (oppoHalf>=0)
				{
					int edgeface = 0, edgeVertex = 0;
					m_meshTopo->halfdecode(oppoHalf, edgeface, edgeVertex);
					if (std::find(faceChunk.begin(), faceChunk.end(), edgeface) == faceChunk.end())
					{
						int edgeFacestemp = m_meshTopo->halfcode(faceID, halfID);
						edgeFaces.emplace_back(edgeFacestemp);
					}
				}
			}

		}
		for (int halfedge : edgeFaces)
		{
			int edgeface = 0, edgeVertex = 0;
			m_meshTopo->halfdecode(halfedge, edgeface, edgeVertex);
			TriMesh::Face& tFace = m_mesh->faces.at(edgeface);
			vec3& vertex1 = m_vertexes.at(m_meshTopo->startvertexid(halfedge));
			vec3& vertex2 = m_vertexes.at(m_meshTopo->endvertexid(halfedge));
			//vec3& vertex3 = m_vertexes.at(tFace[2]);
			edgeSectVertexs.at(faceChunkIndex).push_back(vertex1);
			edgeSectVertexs.at(faceChunkIndex).push_back(vertex2);
		}
	}
	///////////////////////////////////
}


// collision check
void DLPQuickData::dlpSourceCheck(std::vector<DLPISource> & SupportSources, DLPISources &clusteredSources)
{
	std::vector< trimesh::vec3> srcPts;
	for(int index=0;index< SupportSources.size();index++)
	{
		DLPISource &source = SupportSources[index];
		vec3& vertex = source.position;
		check(source.posHit, vertex);
		srcPts.emplace_back(vertex);
		clusteredSources.sources.emplace_back(source);
	}
	clusteredSources.clusteredSrcID = cluster(srcPts, 0.1, 2);
}

}