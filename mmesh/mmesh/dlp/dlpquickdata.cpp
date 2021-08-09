#include "dlpquickdata.h"

#include "mmesh/trimesh/trianglechunk.h"
#include "mmesh/trimesh/meshtopo.h"
#include "mmesh/trimesh/algrithm3d.h"

#include "dlpcreatepolicy.h"
#include <algorithm>
#include <math.h>

#ifdef USE_CGAL
#include "mmesh/cgal/clusterPoint.h"
#endif
#include "mmesh/vcg/trimesh_sampling .h"
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
		, m_DLPISourceInited(false)
		, m_mesh(nullptr)
		, m_pixel(1.0f)
	{
		m_triangleChunk = new TriangleChunk();
		m_meshTopo = new MeshTopo();
		m_throwFunc = NULL;
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
		m_DLPISourceInited = false;
	}

	void DLPQuickData::setXformData(trimesh::fxform& xf)
	{
		m_xf = xf;
		m_invXf = trimesh::inv(xf);
		m_dirty = true;
		m_DLPISourceInited = false;
		clear();
	}
	trimesh::fxform DLPQuickData::getXformData() const
	{
		return m_xf;
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

			//m_triangleChunk->build(m_boxes, globalBox, 2.0f);		
			//if (m_autoParam.space < 0.1)
			//	m_autoParam.space = 2.0;

			//m_triangleChunk->build(m_boxes, globalBox, 2.0f);		
			m_triangleChunk->build(m_boxes, globalBox, m_autoParam.baseSpace);
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
#ifdef USE_VCG_POISSON_SAMPLE
	void DLPQuickData::autoDlpSources(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam, int flag, std::function<void(CallBackParams*)> callback, CallBackParams* cbParams)
	{
		m_throwFunc = callback;
		m_cbParamsPtr = cbParams;
		if (m_autoParam.space != autoParam->space)
		{
			m_dirty = true;
			m_DLPISourceInited = false;
		}
		m_autoParam = *autoParam;
		build();
		sources.clear();
		if (m_throwFunc != NULL)
		{
			m_cbParamsPtr->percentage = 0.1;
			m_throwFunc(m_cbParamsPtr);

		}
		//flag = SUPPORT_FACE|SUPPORT_EDGE|SUPPORT_VERTEX;
		flag = SUPPORT_FACE | SUPPORT_EDGE ;
		if (m_DLPISourceInited == false)
		{
			if ((flag & SUPPORT_VERTEX) == SUPPORT_VERTEX)
			{
				autoDlpVertexSource(m_SupportVertexSources, autoParam);
			}
			if ((flag & SUPPORT_FACE) == SUPPORT_FACE)
			{
				std::vector<std::vector<DLPISource>> sectionSources;
				//m_SupportFaceSources
				autoDlpFaceSource(sectionSources, autoParam);
				m_SupportFaceSources.clear();
				for (int sectIndex = 0; sectIndex < sectionSources.size(); sectIndex++)
				{
					std::vector<DLPISource>& sectionSrcs = sectionSources[sectIndex];

					if (sectionSrcs.size())
						m_SupportFaceSources.insert(m_SupportFaceSources.end(), sectionSrcs.begin(), sectionSrcs.end());
				}
			}
			if ((flag & SUPPORT_EDGE) == SUPPORT_EDGE)
			{
				autoDlpEdgeSource(m_SupportEdgeSources, autoParam);
				if (0)
				{
					//去除相同点、邻近点
					auto cmp_elements_sort = [](const DLPISource& e1, const DLPISource& e2) -> bool {
						trimesh::vec3 zeropos(0.0, 0.0, 0.0);
						float len1 = trimesh::dist2(e1.position, zeropos);
						float len2 = trimesh::dist2(e2.position, zeropos);
						return len1 < len2;

					};
					auto cmp_elements_unique = [this](DLPISource& e1, DLPISource& e2)->bool {

						//float faceCosValue = cosf(m_autoParam.autoAngle * M_PIf / 180.0f);
						float filterValue = m_autoParam.space;
						float distanceValue = trimesh::dist(e1.position, e2.position);
						bool ret = distanceValue < filterValue;
						//std::cout << "cmp_elements_unique=== " << trimesh::dist(e1.position, e2.position) << std::endl;
						return ret;
					};
					std::vector<DLPISource>::iterator iterator;
					std::sort(m_SupportEdgeSources.begin(), m_SupportEdgeSources.end(), cmp_elements_sort);
					iterator = std::unique(m_SupportEdgeSources.begin(), m_SupportEdgeSources.end(), cmp_elements_unique);
					m_SupportEdgeSources.resize(std::distance(m_SupportEdgeSources.begin(), iterator));
				}
			}

			m_DLPISourceInited = true;
		}
		std::cout << "m_SupportVertexSources size=="<< m_SupportVertexSources.size() <<std::endl;
		std::cout << "m_SupportEdgeSources size=="<< m_SupportEdgeSources.size() <<std::endl;
		std::cout << "m_SupportFaceSources size=="<< m_SupportFaceSources.size() <<std::endl;
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
		//if (1)
		int iterations = 0;// sources.size();
		while (iterations)
		{
			//去除相同点、邻近点
			auto cmp_elements_sort = [](const DLPISource& e1, const DLPISource& e2) -> bool {
				trimesh::vec3 zeropos(0.0, 0.0, 0.0);
				float len1 = trimesh::dist2(e1.position, zeropos);
				float len2 = trimesh::dist2(e2.position, zeropos);
				return len1 < len2;

			};
			auto cmp_elements_unique = [this](DLPISource& e1, DLPISource& e2)->bool {

				//float faceCosValue = cosf(m_autoParam.autoAngle * M_PIf / 180.0f);
				 //float filterValue = m_triangleChunk->m_gridSize-EPSILON ;
				 //float filterValue = m_autoParam.space;
				float filterValue = EPSILON;
				float distanceValue = trimesh::dist(e1.position, e2.position);
				bool ret = distanceValue < filterValue;
				//std::cout << "cmp_elements_unique=== " << trimesh::dist(e1.position, e2.position) << std::endl;
				if (ret)
				{
					if (e1.typeflg & SUPPORT_VERTEX)
					{
						ret = false;
						if (e2.typeflg & SUPPORT_VERTEX)
							ret = distanceValue < EPSILON ? true : false;

					}
					else if (e1.typeflg & SUPPORT_EDGE)
					{
						//if (e2.typeflg & SUPPORT_VERTEX )
						//{
						//	e1 = e2;//由于是引用变量，可以动态更改
						//	ret = false;//标识不可移除
						//}
						//else 
						//	ret = distanceValue < EPSILON ? true : false;

					}

				}
				return ret;
			};
			std::vector<DLPISource>::iterator iterator;
			std::sort(sources.begin(), sources.end(), cmp_elements_sort);
			iterator = std::unique(sources.begin(), sources.end(), cmp_elements_unique);
			sources.resize(std::distance(sources.begin(), iterator));
			if (iterations == sources.size())
			{
				iterations = 0;
			}
			else
				iterations = sources.size();

		}
		if (m_throwFunc != NULL)
		{
			m_cbParamsPtr->percentage = 1.0;
			m_throwFunc(m_cbParamsPtr);

		}
	}

#else
	void DLPQuickData::autoDlpSources(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam, int flag, std::function<void(CallBackParams*)> callback, CallBackParams* cbParams)
	{
		m_throwFunc = callback;
		m_cbParamsPtr = cbParams;
		if (m_autoParam.space != autoParam->space)
		{
			m_dirty = true;
			m_DLPISourceInited = false;
		}
		m_autoParam = *autoParam;
		build();
		sources.clear();
		if (m_throwFunc != NULL)
		{
			m_cbParamsPtr->percentage = 0.1;
			m_throwFunc(m_cbParamsPtr);

		}
		//flag = SUPPORT_FACE;
		if (m_DLPISourceInited == false)
		{
			if ((flag & SUPPORT_VERTEX) == SUPPORT_VERTEX)
			{
				autoDlpVertexSource(m_SupportVertexSources, autoParam);
			}
			if ((flag & SUPPORT_EDGE) == SUPPORT_EDGE)
			{
				autoDlpEdgeSource(m_SupportEdgeSources, autoParam);
				if (1)
				{
					//去除相同点、邻近点
					auto cmp_elements_sort = [](const DLPISource& e1, const DLPISource& e2) -> bool {
						trimesh::vec3 zeropos(0.0, 0.0, 0.0);
						float len1 = trimesh::dist2(e1.position, zeropos);
						float len2 = trimesh::dist2(e2.position, zeropos);
						return len1 < len2;

					};
					auto cmp_elements_unique = [this](DLPISource& e1, DLPISource& e2)->bool {

						//float faceCosValue = cosf(m_autoParam.autoAngle * M_PIf / 180.0f);
						float filterValue = m_autoParam.space;
						float distanceValue = trimesh::dist(e1.position, e2.position);
						bool ret = distanceValue < filterValue;
						//std::cout << "cmp_elements_unique=== " << trimesh::dist(e1.position, e2.position) << std::endl;
						return ret;
					};
					std::vector<DLPISource>::iterator iterator;
					std::sort(m_SupportEdgeSources.begin(), m_SupportEdgeSources.end(), cmp_elements_sort);
					iterator = std::unique(m_SupportEdgeSources.begin(), m_SupportEdgeSources.end(), cmp_elements_unique);
					m_SupportEdgeSources.resize(std::distance(m_SupportEdgeSources.begin(), iterator));
				}
			}
			if ((flag & SUPPORT_FACE) == SUPPORT_FACE)
			{
				std::vector<std::vector<DLPISource>> sectionSources;
				//m_SupportFaceSources
				autoDlpFaceSource(sectionSources, autoParam);
				m_SupportFaceSources.clear();
				for (int sectIndex=0 ; sectIndex<sectionSources.size(); sectIndex++)
				{
					std::vector<DLPISource> &sectionSrcs = sectionSources[sectIndex];
					int testnumbers=sectionSources.size();
					//if (sectionSrcs.size() > 8)
					//	continue;
#if 0
					#ifdef USE_CGAL
					std::vector<trimesh::vec3> points;

					for (DLPISource dlpsrc: sectionSrcs)
					{
						points.emplace_back(dlpsrc.position);
					}
					std::vector<std::vector<trimesh::vec3>> sectionClusters= ClusterPoint::ClusterAllPoints(points);
					for (std::vector<trimesh::vec3> sectionCluster : sectionClusters)
					{
						if (sectionCluster.size())
						{
							DLPISource dlpsrctem;
							dlpsrctem.position = sectionCluster[0];
							m_SupportFaceSources.emplace_back(dlpsrctem);
						}
					}
					#endif
#else

					int iterations = sectionSrcs.size();
					//int iterations = 0;
					while (iterations )
					{
						//去除相同点、邻近点
						auto cmp_elements_sort = [](const DLPISource& e1, const DLPISource& e2) -> bool {
							trimesh::vec3 zeropos(0.0, 0.0, 0.0);
							float len1 = trimesh::dist2(e1.position, zeropos);
							float len2 = trimesh::dist2(e2.position, zeropos);
							return len1 < len2;

						};
						auto cmp_elements_unique = [this](DLPISource& e1, DLPISource& e2)->bool {

							//float faceCosValue = cosf(m_autoParam.autoAngle * M_PIf / 180.0f);
							float filterValue = m_autoParam.space;
							float distanceValue = trimesh::dist(e1.position, e2.position);
							bool ret = distanceValue < filterValue;
							//std::cout << "cmp_elements_unique=== " << trimesh::dist(e1.position, e2.position) << std::endl;
							return ret;
						};
						std::vector<DLPISource>::iterator iterator;
						std::sort(sectionSrcs.begin(), sectionSrcs.end(), cmp_elements_sort);
						iterator = std::unique(sectionSrcs.begin(), sectionSrcs.end(), cmp_elements_unique);
						sectionSrcs.resize(std::distance(sectionSrcs.begin(), iterator));
						if (iterations == sectionSrcs.size())
						{
							iterations = 0;
						}
						else
							iterations = sectionSrcs.size();
						std::cout << "iterations===" << iterations << std::endl;
					}
					if (sectionSrcs.size() == 1)
					{
						trimesh::vec3& centerPoint = m_ConnectSectionInfor.centerPoint[sectIndex];
						std::vector<int>& faceChunk = m_ConnectSectionInfor.facesIndexes[sectIndex];
						vec3 xypoint(centerPoint.x, centerPoint.y, 0.0);
						vec3 dir = UP_NORMAL;
						float t = 0.0, u = 0.0, v = 0.0;
						for (int faceIDindex = 0; faceIDindex < faceChunk.size(); faceIDindex++)
						{
							int& faceID = faceChunk[faceIDindex];
							TriMesh::Face& tFace = m_mesh->faces.at(faceID);
							vec3& vertex1 = m_vertexes.at(tFace[0]);
							vec3& vertex2 = m_vertexes.at(tFace[1]);
							vec3& vertex3 = m_vertexes.at(tFace[2]);
							vec3& normal = m_faceNormals.at(faceID);

							if (rayIntersectTriangle(xypoint, dir, vertex1, vertex2, vertex3, &t, &u, &v))
							{
								vec3 pointCross = xypoint + t * dir;
								DLPISource dlpSource = generateSource(pointCross, normal);
								dlpSource.typeflg = SUPPORT_FACE;
								sectionSrcs.clear();
								sectionSrcs.push_back(dlpSource);
							}
						}
					}
					if (sectionSrcs.size())
						m_SupportFaceSources.insert(m_SupportFaceSources.end(), sectionSrcs.begin(), sectionSrcs.end());
#endif
					/////////////
				}
			}

			m_DLPISourceInited = true;
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
		//if (1)
		int iterations = 0;
		while(iterations<2)
		{
			iterations++;
			//去除相同点、邻近点
			auto cmp_elements_sort = [](const DLPISource& e1, const DLPISource& e2) -> bool {
				trimesh::vec3 zeropos(0.0,0.0,0.0);
				float len1=trimesh::dist2(e1.position, zeropos);
				float len2=trimesh::dist2(e2.position, zeropos);
				return len1 < len2;

			};
			auto cmp_elements_unique = [this](DLPISource& e1, DLPISource& e2)->bool {

				//float faceCosValue = cosf(m_autoParam.autoAngle * M_PIf / 180.0f);
				 //float filterValue = m_triangleChunk->m_gridSize-EPSILON ;
				float filterValue = EPSILON;
				float distanceValue = trimesh::dist(e1.position, e2.position);
				bool ret = distanceValue < filterValue;
				//std::cout << "cmp_elements_unique=== " << trimesh::dist(e1.position, e2.position) << std::endl;
				if (ret)
				{
					if (e1.typeflg & SUPPORT_VERTEX)
					{
						ret = false;
						if(e2.typeflg & SUPPORT_VERTEX)
							ret = distanceValue < EPSILON ? true : false;

					}
					else if (e1.typeflg & SUPPORT_EDGE)
					{
						//if (e2.typeflg & SUPPORT_VERTEX )
						//{
						//	e1 = e2;//由于是引用变量，可以动态更改
						//	ret = false;//标识不可移除
						//}
						//else 
						//	ret = distanceValue < EPSILON ? true : false;

					}
				}
				return ret;
			};
			std::vector<DLPISource>::iterator iterator;
			std::sort(sources.begin(), sources.end(), cmp_elements_sort);
			iterator = std::unique(sources.begin(), sources.end(), cmp_elements_unique);
			sources.resize(std::distance(sources.begin(), iterator));
		}
		/////将支撑点进行网格映射
#ifdef CX_BOOST_CLUSTER
		int whileTimes = 0;
		while (whileTimes < 0)
		{
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
			whileTimes++;
		}
#endif
		if (m_throwFunc != NULL)
		{
			m_cbParamsPtr->percentage = 1.0;
			m_throwFunc(m_cbParamsPtr);

		}
	}
#endif
	void DLPQuickData::autoDlpVertexSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam)
	{
		std::vector<int> supportVertexes;
		float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);
		m_meshTopo->lowestVertex(m_vertexes, supportVertexes);
		int  percentage_Count = 0;
		sources.clear();
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
				for (int i = 0; i < faceVindexSize; i++)
				{
					//if (indexFace == faceVindex.at(i))
					//	continue;
					TriMesh::Face& tFace = m_mesh->faces.at(faceVindex.at(i));
					vec3& vertex1 = m_vertexes.at(tFace[0]);
					vec3& vertex2 = m_vertexes.at(tFace[1]);
					vec3& vertex3 = m_vertexes.at(tFace[2]);

					vec3 dir = DOWN_NORMAL;
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
					DLPISource dlpSource = generateSource(vertex, DOWN_NORMAL);
					dlpSource.typeflg = SUPPORT_VERTEX;
					sources.push_back(dlpSource);

				}

				percentage_Count++;

				if ((m_throwFunc != NULL) && (percentage_Count % 100 == 0))
				{
					m_cbParamsPtr->percentage = 0.1 + (float)percentage_Count / (float)supportVertexes.size() * 2;
					m_throwFunc(m_cbParamsPtr);
				}
			}
		}
		if (m_throwFunc != NULL)
		{
			m_cbParamsPtr->percentage = 0.3;
			m_throwFunc(m_cbParamsPtr);
		}
	}

	void DLPQuickData::autoDlpEdgeSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam)
	{
		std::vector<ivec2> supportEdges;
		float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);

		m_meshTopo->hangEdge(m_vertexes, m_faceNormals, m_dotValues, faceCosValue, supportEdges);

		float minDelta = autoParam->space;

		int percentage_Count = 0;
		sources.clear();
		for (ivec2 vertexesID : supportEdges)
		{
			int vertexID1 = vertexesID.x;
			int vertexID2 = vertexesID.y;
			vec3& vertex1 = m_vertexes.at(vertexID1);
			vec3& vertex2 = m_vertexes.at(vertexID2);
			vec3 vertex21 = vertex2 - vertex1;
			////////////////////////////////////////
			float edgeLenXY = trimesh::len(vertex21);
			if (edgeLenXY < minDelta)//长度小于自支撑长度
				continue;

#if 1
			int sampleEdgeNum = ceilf(edgeLenXY / minDelta) + 1;
			if (sampleEdgeNum > 0)
			{
				float dt = 1.0f / (float)sampleEdgeNum;
				for (int sampleIndex = 0; sampleIndex <= sampleEdgeNum; sampleIndex++)
				{
					float t = sampleIndex * dt;
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
				m_cbParamsPtr->percentage = 0.3 + (float)percentage_Count / (float)supportEdges.size() * 0.2;
				m_throwFunc(m_cbParamsPtr);
			}
		}
		if (m_throwFunc != NULL)
		{
			m_cbParamsPtr->percentage = 0.5;
			m_throwFunc(m_cbParamsPtr);
		}
	}
#ifdef USE_VCG_POISSON_SAMPLE
	void DLPQuickData::autoDlpFaceSource(std::vector<std::vector<DLPISource>>& sectionSources, AutoDLPSupportParam* autoParam)
	{
		float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);
		int percentage_Count = 0;
		std::vector<std::vector<int>> supportFaces;
		sectionSources.clear();
		supportFaces.clear();
		m_ConnectSectionInfor.facesIndexes.clear();

		m_meshTopo->chunkFace(m_dotValues, supportFaces, faceCosValue);
		vcg::CX_PoissonAlg::PoissonAlgCfg poissonAlgcfg;
		poissonAlgcfg.baseSampleRad = autoParam->baseSpace;//m_triangleChunk->m_width;
		poissonAlgcfg.userSampleRad = autoParam->space> poissonAlgcfg.baseSampleRad ? autoParam->space : poissonAlgcfg.baseSampleRad;
		poissonAlgcfg.ratio = autoParam->density;//m_triangleChunk->m_width;
		//poissonAlgcfg.baseSampleRad = poissonAlgcfg.userSampleRad;//m_triangleChunk->m_width;
		m_supportFace.clear();
		for (std::vector<int>& faceChunk : supportFaces)
		{

			trimesh::box3 connectSectBox;
			float faceChunkArea = 0.0;
			bool supportEnable = true;
			std::vector< trimesh::vec3> inVertexs;
			std::vector< trimesh::vec3> inNors;
			std::vector< trimesh::vec3> outVertexs;
			std::vector<DLPISource> sources;
			std::vector<trimesh::TriMesh::Face> sectFacesIndex;
			std::vector<int> edgeFaces;
			std::vector<trimesh::vec3> edgeSectVertexsNearUp;
			trimesh::vec3 centerPoint(-1.0, -1.0, -1.0);
			trimesh::vec3 farPoint(-1.0, -1.0, -1.0);
			for (int faceID : faceChunk)
			{
				TriMesh::Face& tFace = m_mesh->faces.at(faceID);
				sectFacesIndex.emplace_back(tFace);

			}
			bool supportFlag = sectionFaceNeedSupport(faceChunk, edgeFaces, farPoint, centerPoint);
			//bool supportFlag = sectionFaceNeedSupport(faceChunk, edgeFaces, farPoint, centerPoint, edgeSectVertexsNearUp);
			if(supportFlag)
			{
				std::vector< trimesh::vec3> outVertexs;
				std::vector< trimesh::vec3> outfirstVertexs;
				std::vector< trimesh::vec3> outSecondVertexs;
				std::vector< trimesh::vec3> outBorderVertexs;
				std::vector<DLPISource> sources;
				vcg::CX_PoissonAlg::PoissonFunc PoissonFuncObj;
				PoissonFuncObj.setPoissonCfg(&poissonAlgcfg);

				PoissonFuncObj.main(m_vertexes, sectFacesIndex, outfirstVertexs,true);
				if (outfirstVertexs.size() > 0)
				{
					PoissonFuncObj.borderSamper(outBorderVertexs);
					PoissonFuncObj.mainSecond(outBorderVertexs, outVertexs);
				}

				//outVertexs.clear();
				//outVertexs.insert(outVertexs.end(),outfirstVertexs.begin(), outfirstVertexs.end());
				//outVertexs.insert(outVertexs.end(), outSecondVertexs.begin(), outSecondVertexs.end());
				//outVertexs.insert(outVertexs.end(), outBorderVertexs.begin(), outBorderVertexs.end());
				#ifdef DEBUG
				std::cout << "extract_poisson_point===" << outVertexs.size() << std::endl;
				#endif
				m_supportFace.insert(m_supportFace.end(), faceChunk.begin(), faceChunk.end());
				for (trimesh::vec3 pt : outVertexs)
				{
					vec3 dir = DOWN_NORMAL;

					{
						ivec2 pointIndex = m_triangleChunk->index(pt);
						int iindex = pointIndex.x + pointIndex.y * m_triangleChunk->m_width;
						std::vector<int>& faceVindex = m_triangleChunk->m_cells.at(iindex);
						int faceVindexSize = faceVindex.size();
						for (int faceID : faceChunk)
						{
							TriMesh::Face& tFace = m_mesh->faces.at(faceID);
							vec3& vertex1 = m_vertexes.at(tFace[0]);
							vec3& vertex2 = m_vertexes.at(tFace[1]);
							vec3& vertex3 = m_vertexes.at(tFace[2]);
							if (PointinTriangle(vertex1, vertex2, vertex3, pt))
							{
								//std::cout << "hit" << std::endl;
								dir = m_faceNormals[faceID];
								break;
							}

						}

					}


					DLPISource dlpSource = generateSource(pt, dir);
					dlpSource.typeflg = SUPPORT_FACE;
					sources.push_back(dlpSource);
				}
				//if (sources.size() <= 1)
				if (0)
				{
					//if ((sources.size() == 1) || (supportEnable == true))
					{
						vec3 xypoint(centerPoint.x, centerPoint.y, 0.0);
						vec3 dir = UP_NORMAL;
						float t = 0.0, u = 0.0, v = 0.0;
						for (int faceIDindex = 0; faceIDindex < faceChunk.size(); faceIDindex++)
						{
							int& faceID = faceChunk[faceIDindex];
							TriMesh::Face& tFace = m_mesh->faces.at(faceID);
							vec3& vertex1 = m_vertexes.at(tFace[0]);
							vec3& vertex2 = m_vertexes.at(tFace[1]);
							vec3& vertex3 = m_vertexes.at(tFace[2]);
							vec3& normal = m_faceNormals.at(faceID);
#if 1
							if (rayIntersectTriangle(xypoint, dir, vertex1, vertex2, vertex3, &t, &u, &v))
							{
								vec3 pointCross = xypoint + t * dir;
								#ifdef DEBUG
								std::cout << "pointCross===" << pointCross << std::endl;
								#endif

								DLPISource dlpSource = generateSource(pointCross, normal);
								dlpSource.typeflg = SUPPORT_FACE;
								if (sources.size() == 1)
								{
									sources.clear();
								}
								sources.push_back(dlpSource);
							}
#else
							{
								vec3 pointCross = xypoint + t * dir;
								DLPISource dlpSource = generateSource(pointCross, normal);
								dlpSource.typeflg = SUPPORT_FACE;
								if (sources.size() == 1)
								{
									sources.clear();
								}
								sources.push_back(dlpSource);
							}
#endif
						}
					}
					if (sources.size() == 0)
					{
						std::cout << "no rayIntersectTriangle xypoint===" << centerPoint << std::endl;

					}

					//sources.clear();
					if ((sources.size() == 0) && (supportEnable == true))
					{
						int& faceID = faceChunk[0];//默认选中第一个
						TriMesh::Face& tFace = m_mesh->faces.at(faceID);
						vec3& vertex1 = m_vertexes.at(tFace[0]);
						vec3& vertex2 = m_vertexes.at(tFace[1]);
						vec3& vertex3 = m_vertexes.at(tFace[2]);
						vec3 normal = m_faceNormals.at(faceID);
						vec3 pointCross = (vertex1 + vertex2+ vertex3) / 3;
						vec3 xypoint0(pointCross.x, pointCross.y, 0.0);
						vec3 dir = UP_NORMAL;
						float t, u, v;

						if (farPoint.z>0.0)
						{
							vec3 pointCross = farPoint;
							DLPISource dlpSource = generateSource(pointCross, normal);
							dlpSource.typeflg = SUPPORT_FACE;
							sources.push_back(dlpSource);
						}
						else if (rayIntersectTriangle(xypoint0, dir, vertex1, vertex2, vertex3, &t, &u, &v))
						{
							vec3 pointCross = xypoint0 + t * dir;
							DLPISource dlpSource = generateSource(pointCross, normal);
							dlpSource.typeflg = SUPPORT_FACE;
							sources.push_back(dlpSource);
						}
					}

				}

				if (sources.size())
				{
					m_ConnectSectionInfor.facesIndexes.emplace_back();
					m_ConnectSectionInfor.facesIndexes[m_ConnectSectionInfor.facesIndexes.size() - 1] = faceChunk;

					sectionSources.emplace_back(sources);
				}
			}
			percentage_Count++;

			if ((m_throwFunc != NULL) && (percentage_Count % 100 == 0))
			{
				m_cbParamsPtr->percentage = 0.5 + (float)percentage_Count / (float)supportFaces.size() * 0.4;
				m_throwFunc(m_cbParamsPtr);
			}
		}
		if (m_throwFunc != NULL)
		{
			m_cbParamsPtr->percentage = 0.9;
			m_throwFunc(m_cbParamsPtr);
		}
	}

#else
	void DLPQuickData::autoDlpFaceSource(std::vector<std::vector<DLPISource>>& sectionSources, AutoDLPSupportParam* autoParam)
	{
		float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);
		m_ConnectSectionInfor.edgeFaces.clear();
		m_ConnectSectionInfor.facesIndexes.clear();
		m_ConnectSectionInfor.centerPoint.clear();
		m_ConnectSectionInfor.bBoxes.clear();
		std::vector<std::vector<int>> supportFaces;
		m_meshTopo->chunkFace(m_dotValues, supportFaces, faceCosValue);
		int percentage_Count = 0;
		sectionSources.clear();
		for (std::vector<int>& faceChunk : supportFaces)
		{
			//std::sort(faceChunk.begin(), faceChunk.end(), [this](int r1, int r2)->bool {
			//	return m_dotValues.at(r1) < m_dotValues.at(r2);
			//	});
				//if (testindex != 1)
				//	continue;
				//testindex++;
			trimesh::box3 connectSectBox;
			float faceChunkArea = 0.0;
			bool supportEnable = true;
			for (int faceID : faceChunk)
			{
				TriMesh::Face& tFace = m_mesh->faces.at(faceID);
				vec3& vertex1 = m_vertexes.at(tFace[0]);
				vec3& vertex2 = m_vertexes.at(tFace[1]);
				vec3& vertex3 = m_vertexes.at(tFace[2]);
				vec3 e0 = vertex2 - vertex1;
				vec3 e1 = vertex3 - vertex1;
				faceChunkArea += 0.5f * len(e0 TRICROSS e1);
				connectSectBox += vertex1;
				connectSectBox += vertex2;
				connectSectBox += vertex3;
			}
			vec3 SectBoxSize = connectSectBox.size();
			float gridSize = m_triangleChunk->m_gridSize;
			int sampleXn = ceilf((connectSectBox.max.x - connectSectBox.min.x) / gridSize);
			int sampleYn = ceilf((connectSectBox.max.y - connectSectBox.min.y) / gridSize);
			float deltx = (connectSectBox.max.x - connectSectBox.min.x) / sampleXn;
			float delty = (connectSectBox.max.y - connectSectBox.min.y) / sampleYn;
			vec2 xypointMin;
			xypointMin.x = connectSectBox.min.x;
			xypointMin.y = connectSectBox.min.y;
			/////
			m_ConnectSectionInfor.bBoxes.emplace_back(connectSectBox);
			//if ((abs(SectBoxSize.x- SectBoxSize.y)< gridSize*2)&&(sampleXn<3|| sampleYn <3))
			//	continue;
			std::vector<DLPISource> sources;
			trimesh::vec3 farPoint(0.0,0.0,0.0);
			trimesh::vec3 centerPoint(0.0,0.0,0.0);
			std::vector<int> edgeFaces;
			bool supportFlag = sectionFaceNeedSupport(faceChunk, edgeFaces, farPoint, centerPoint);
			//if ((faceChunkArea < M_PIf * std::powf(m_triangleChunk->m_gridSize / 2.0, 2.0)) && (supportFlag == false))
			//if ((faceChunkArea < M_PIf * std::powf(2.0, 2.0)) && (supportFlag == false))
			if  (supportFlag == false)
				supportEnable = false;

			for (int faceIDindex = 0; (faceIDindex < faceChunk.size()) && (supportEnable == true); faceIDindex++)
			{
				int& faceID = faceChunk[faceIDindex];
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

					//float tx = xypointMin.x + sampleXindex * deltx;
					float tx = xypointMin.x + sampleXindex * deltx + deltx / 2.0;
					if (sampleXn == 1)
						tx = xypointMin.x + SectBoxSize.x / 2.0;//取中间值
					//if ((connectSectBox.max.x - tx > deltx / 2.0)&& (connectSectBox.max.x - tx < deltx))//确保最大值处取点
					if (connectSectBox.max.x - tx < 0.0)//确保最大值处取点
						tx = connectSectBox.max.x;
					for (int sampleYindex = 0; sampleYindex < sampleYn; sampleYindex++)
					{
						//float ty = xypointMin.y + sampleYindex * delty;
						float ty = xypointMin.y + sampleYindex * delty + delty / 2.0;

						if (sampleYn == 1)
							ty = xypointMin.y + SectBoxSize.y / 2.0;
						//if ((connectSectBox.max.y - ty > delty / 2.0) && (connectSectBox.max.y - ty < delty))
						if (connectSectBox.max.y - ty < 0.)
							ty = connectSectBox.max.y;
						vec3 xypoint0(tx, ty, 0.0);
						vec3 dir = UP_NORMAL;
						float t, u, v;
						if (rayIntersectTriangle(xypoint0, dir, vertex1, vertex2, vertex3, &t, &u, &v))
						{
							vec3 pointCross = xypoint0 + t * dir;
							DLPISource dlpSource = generateSource(pointCross, normal);
							dlpSource.typeflg = SUPPORT_FACE;
							sources.push_back(dlpSource);
						}
					}
				}

			}
			//if ((sources.size() == 0) && (supportEnable == true))
			if ((sources.size() <= 1) && (supportEnable == true))
			{
				vec3 xypoint(centerPoint.x, centerPoint.y,0.0);
				vec3 dir = UP_NORMAL;
				float t = 0.0, u = 0.0, v = 0.0;
				for (int faceIDindex = 0; faceIDindex < faceChunk.size(); faceIDindex++)
				{
					int& faceID = faceChunk[faceIDindex];
					TriMesh::Face& tFace = m_mesh->faces.at(faceID);
					vec3& vertex1 = m_vertexes.at(tFace[0]);
					vec3& vertex2 = m_vertexes.at(tFace[1]);
					vec3& vertex3 = m_vertexes.at(tFace[2]);
					vec3 &normal = m_faceNormals.at(faceID);

					if (rayIntersectTriangle(xypoint, dir, vertex1, vertex2, vertex3, &t, &u, &v))
					{
						vec3 pointCross = xypoint + t * dir;
						DLPISource dlpSource = generateSource(pointCross, normal);
						dlpSource.typeflg = SUPPORT_FACE;
						if (sources.size() == 1)
						{
							sources.clear();
						}
						sources.push_back(dlpSource);
					}
				} 
			}
			if ((sources.size() == 0) && (supportEnable == true))
			{
				int& faceID = faceChunk[0];//默认选中第一个
				TriMesh::Face& tFace = m_mesh->faces.at(faceID);
				vec3& vertex1 = m_vertexes.at(tFace[0]);
				vec3& vertex2 = m_vertexes.at(tFace[1]);
				vec3& vertex3 = m_vertexes.at(tFace[2]);
				vec3 normal = m_faceNormals.at(faceID);
				vec3 pointCross = (vertex1 + vertex2 ) / 3;
				vec3 xypoint0(pointCross.x, pointCross.y, 0.0);
				vec3 dir = UP_NORMAL;
				float t, u, v;
				if (rayIntersectTriangle(xypoint0, dir, vertex1, vertex2, vertex3, &t, &u, &v))
				{
					vec3 pointCross = xypoint0 + t * dir;
					DLPISource dlpSource = generateSource(pointCross, normal);
					dlpSource.typeflg = SUPPORT_FACE;
					sources.push_back(dlpSource);
				}
			}

			if (sources.size())
			{
				m_ConnectSectionInfor.edgeFaces.emplace_back(edgeFaces);
				m_ConnectSectionInfor.centerPoint.emplace_back(centerPoint);
				m_ConnectSectionInfor.facesIndexes.emplace_back();
				m_ConnectSectionInfor.facesIndexes[m_ConnectSectionInfor.facesIndexes.size() -1]= faceChunk;
				sectionSources.emplace_back(sources);

			}

			percentage_Count++;

			if ((m_throwFunc != NULL) && (percentage_Count % 100 == 0))
			{
				m_cbParamsPtr->percentage = m_cbParamsPtr->percentage + (float)percentage_Count / (float)supportFaces.size() * 0.5;
				m_throwFunc(m_cbParamsPtr);
			}
		}
	}
#endif
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


	void DLPQuickData::extractFaceSectEdge(std::vector<std::vector<int>> SupportFaces, std::vector<std::vector<trimesh::vec3>>& edgeSectVertexs)
	{
		edgeSectVertexs.resize(SupportFaces.size());
		for (int faceChunkIndex = 0; faceChunkIndex < SupportFaces.size(); faceChunkIndex++)
		{
			std::vector<int> edgeFaces;
			std::vector<int>& faceChunk = SupportFaces.at(faceChunkIndex);
			float sectionFaceArea = 0.0;
			searchSectionFaceEdgeFace(faceChunk, edgeFaces, sectionFaceArea);
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
	void DLPQuickData::dlpSourceCheck(std::vector<DLPISource>& SupportSources, DLPISources& clusteredSources)
	{
		std::vector< trimesh::vec3> srcPts;
		for (int index = 0; index < SupportSources.size(); index++)
		{
			DLPISource& source = SupportSources[index];
			vec3& vertex = source.position;
			check(source.posHit, vertex);
			srcPts.emplace_back(vertex);
			clusteredSources.sources.emplace_back(source);
		}
#ifdef CX_BOOST_CLUSTER
		clusteredSources.clusteredSrcID = cluster(srcPts, m_triangleChunk->m_width, 2);
#endif
	}
	static std::pair <float, int> far_NearUp_index(std::vector<trimesh::vec3> VertexsDown, std::vector<trimesh::vec3> VertexsUp)
	{
		int idx = -1;
		std::pair <float,int> ret(-1.0,-1);
		std::vector<std::pair <float, int>> distanceIndex;
		auto cmp_elements_sort = [](const std::pair <float, int>& e1, const std::pair <float, int>& e2) -> bool {
			return e1.first < e2.first;

		};

		for (int upIndex=0; upIndex< VertexsUp.size(); upIndex++)
		{
			trimesh::vec3 VertexUp = VertexsUp[upIndex];
			std::vector<std::pair <float, int>> distanceIndexIndex;

			for (int downIndex = 0; downIndex < VertexsDown.size(); downIndex++)
			{
				trimesh::vec3 VertexDown = VertexsDown[downIndex];
				float distance=trimesh::dist(VertexUp, VertexDown);
				distanceIndexIndex.emplace_back(std::make_pair(distance, upIndex));
			}
			if (distanceIndexIndex.size())
			{
				std::sort(distanceIndexIndex.begin(), distanceIndexIndex.end(), cmp_elements_sort);
				std::pair <float, int> retInner = *(distanceIndexIndex.begin());
				distanceIndex.emplace_back(retInner);
			}

		}
		if (distanceIndex.size())
		{
			std::sort(distanceIndex.begin(), distanceIndex.end(), cmp_elements_sort);
			ret = *(distanceIndex.end()-1);
		}
		return ret;
	}
	//this function is just used for small section face
	bool DLPQuickData::sectionFaceNeedSupport(std::vector<int> SupportFaces, std::vector<int> &edgeFaces, trimesh::vec3 &farPoint, trimesh::vec3& centerPoint)
	{
		bool supportflag = false;
		int nearFaceUp = 0;
		int nearFaceDown = 0;
		
		std::vector<trimesh::vec3> edgeSectVertexsNearDown;
		std::vector<trimesh::vec3> edgeSectVertexsNearUp;
		float sectionFaceArea = 0.0;
		searchSectionFaceEdgeFace(SupportFaces, edgeFaces, sectionFaceArea);
		trimesh::vec3 centerPointtemp(0.0, 0.0, 0.0);
		for (int edgeFacesIndex = 0; edgeFacesIndex < edgeFaces.size(); edgeFacesIndex++)
		{
			int edgefaceID = 0, edgeVertex = 0;
			int oppofaceID = 0, oppoVertex = 0;
			m_meshTopo->halfdecode(edgeFaces[edgeFacesIndex], edgefaceID, edgeVertex);
			int vertexIndexStart=m_meshTopo->startvertexid(edgeFaces[edgeFacesIndex]);
			int vertexIndexEnd=m_meshTopo->endvertexid(edgeFaces[edgeFacesIndex]);

			TriMesh::Face& edgeFaces = m_mesh->faces.at(edgefaceID);
			centerPointtemp += m_vertexes[vertexIndexStart];
			ivec3& oppoHalfs = m_meshTopo->m_oppositeHalfEdges.at(edgefaceID);
			{
				vec3& vertex1 = m_vertexes.at(edgeFaces[0]);
				vec3& vertex2 = m_vertexes.at(edgeFaces[1]);
				vec3& vertex3 = m_vertexes.at(edgeFaces[2]);
				vec3 e0 = vertex2 - vertex1;
				vec3 e1 = vertex3 - vertex1;
				float faceArea = 0.5f * len(e0 TRICROSS e1);
				float faceAreaThreshold = M_PIf * std::pow(m_autoParam.baseSpace / 10.0, 2.0) * 0.7;
				if (faceArea < faceAreaThreshold)
					continue;

			}
			for (int oppoHalfsIndex = 0; (oppoHalfsIndex < 3)&&((oppoHalfs.x !=-1)&& (oppoHalfs.y != -1) && (oppoHalfs.z != -1)); oppoHalfsIndex++)
			{
				int oppovertexIndexStart=m_meshTopo->startvertexid(oppoHalfs[oppoHalfsIndex]);
				int oppovertexIndexEnd=m_meshTopo->endvertexid(oppoHalfs[oppoHalfsIndex]);

				m_meshTopo->halfdecode(oppoHalfs[oppoHalfsIndex], oppofaceID, oppoVertex);

				if ((vertexIndexStart == oppovertexIndexEnd)&&(vertexIndexEnd == oppovertexIndexStart))//检测到共边相邻三角形
				{
					TriMesh::Face& oppoFaces = m_mesh->faces.at(oppofaceID);
					trimesh::vec3& edgeVertexFirst = m_vertexes[edgeFaces[(edgeVertex ) % 3]];
					trimesh::vec3& edgeVertexSecond = m_vertexes[edgeFaces[(edgeVertex + 1) % 3]];

					trimesh::vec3 oppoVertexThird = m_vertexes[oppoFaces[(oppoVertex + 2) % 3]];
					//if (oppoVertexThird.z < edgeVertexThird.z)//支撑区域是悬空区
					//	supportflag = true;
					//if ((edgeVertexFirst.z - oppoVertexThird.z > EPSILON) && (edgeVertexSecond.z - oppoVertexThird.z > EPSILON))//支撑区域是悬空区
					if ((edgeVertexFirst.z - oppoVertexThird.z > EPSILON) && (edgeVertexSecond.z - oppoVertexThird.z > EPSILON))//支撑区域是悬空区
					{
						nearFaceDown += 1;
					}
					else if ((edgeVertexFirst.z - oppoVertexThird.z < EPSILON) && (edgeVertexSecond.z - oppoVertexThird.z < EPSILON))
					{
						nearFaceUp += 1;

					}
					/////////////////////////////////////////////////
					if ((oppoVertexThird.z - edgeVertexFirst.z > 0.0) || (oppoVertexThird.z - edgeVertexSecond.z > 0.0))
					{
						trimesh::vec3 temp = edgeVertexFirst.z - edgeVertexSecond.z > 0.0 ? edgeVertexSecond : edgeVertexFirst;
						edgeSectVertexsNearUp.emplace_back(temp);

					}
					else
					{
						trimesh::vec3 temp = edgeVertexFirst.z - edgeVertexSecond.z > 0.0 ? edgeVertexSecond : edgeVertexFirst;
						edgeSectVertexsNearDown.emplace_back(temp);
					}

				}

			}
		}
		std::pair <float, int> far_pointpair= far_NearUp_index( edgeSectVertexsNearDown, edgeSectVertexsNearUp);
		//std::cout << "far_point.distance==" << far_pointpair.first << std::endl;
		//std::cout << "nearFaceDown==" << nearFaceDown << std::endl;
		//std::cout << "nearFaceUp==" << nearFaceUp << std::endl;
		//std::cout << std::endl;
		if (far_pointpair.second > 0)
			farPoint = edgeSectVertexsNearUp[far_pointpair.second];
		centerPoint = centerPointtemp / edgeFaces.size();

		{
			bool smallsectEable = far_pointpair.first > m_autoParam.baseSpace;
			float areaThreshold = M_PIf * std::pow(m_autoParam.space / 2.0, 2.0) * 0.7;
			//use pow replace powf, for linux compile
			{
				//if ((sectionFaceArea > areaThreshold)&&(smallsectEable==true))
				if (sectionFaceArea > areaThreshold)
				{
					supportflag = true;
				}
			}
			if (supportflag == false)
			{
				//if (nearFaceDown == 0 || smallsectEable)
				//if (nearFaceDown==0&& (sectionFaceArea> M_PIf * std::pow(m_autoParam.baseSpace / 2.0, 2.0)))
				if (nearFaceDown < 1)//悬重面
				{
					supportflag = true;
				}
				//if(smallsectEable)
				//	supportflag = true;
			}
		}
		return supportflag;//supportflag;
	}
	void DLPQuickData::searchSectionFaceEdgeFace(std::vector<int> SupportFaces, std::vector<int>& edgeFaces, float &sectionFaceArea)
	{
		for (int faceID : SupportFaces)
		{
			ivec3& oppoHalfs = m_meshTopo->m_oppositeHalfEdges.at(faceID);
			TriMesh::Face& tFace = m_mesh->faces.at(faceID);
			vec3& vertex1 = m_vertexes.at(tFace[0]);
			vec3& vertex2 = m_vertexes.at(tFace[1]);
			vec3& vertex3 = m_vertexes.at(tFace[2]);
			vec3 e0 = vertex2 - vertex1;
			vec3 e1 = vertex3 - vertex1;
			sectionFaceArea += 0.5f * len(e0 TRICROSS e1);
			for (int halfID = 0; halfID < 3; ++halfID)
			{
				int oppoHalf = oppoHalfs.at(halfID);
				if (oppoHalf >= 0)
				{
					int edgeface = 0, edgeVertex = 0;
					m_meshTopo->halfdecode(oppoHalf, edgeface, edgeVertex);
					if (std::find(SupportFaces.begin(), SupportFaces.end(), edgeface) == SupportFaces.end())
					{
						int edgeFacestemp = m_meshTopo->halfcode(faceID, halfID);
						edgeFaces.emplace_back(edgeFacestemp);
					}
				}
				else
				{
					std::cout << "have not neighbor face!!!!" << std::endl;
				}

			}
		}
	}
}