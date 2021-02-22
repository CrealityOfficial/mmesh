#include "dlpquickdata.h"

#include "mmesh/trimesh/trianglechunk.h"
#include "mmesh/trimesh/meshtopo.h"
#include "mmesh/trimesh/algrithm3d.h"

#include "dlpcreatepolicy.h"
using namespace trimesh;
namespace mmesh
{
	DLPQuickData::DLPQuickData()
		: m_dirty(true)
		, m_mesh(nullptr)
		, m_pixel(1.0f)
	{
		m_triangleChunk = new TriangleChunk();
		m_meshTopo = new MeshTopo();
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
			m_SupportFacesFlg.resize(faceNum,false);
			m_SupportVertexFlg.resize(vertexNum,false);
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
			m_FaceSampleInCell.resize(m_triangleChunk->m_width * m_triangleChunk->m_height);
			m_FaceNormalsInCell.resize(m_triangleChunk->m_width * m_triangleChunk->m_height);

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

		ivec2 pointIndex = m_triangleChunk->index(position);
		if (m_triangleChunk->contain(pointIndex))
		{
			vec3 n = vec3(0.0f, 0.0f, -1.0f);
			int index = -1;
			std::vector<int>& cells = m_triangleChunk->cell(pointIndex);
			for (int i : cells)
			{
				TriMesh::Face& face = m_mesh->faces.at(i);
				vec3& vertex0 = m_vertexes.at(face[0]);
				vec3& vertex1 = m_vertexes.at(face[1]);
				vec3& vertex2 = m_vertexes.at(face[2]);

				float t, u, v;
				if (rayIntersectTriangle(position, n, vertex0, vertex1, vertex2, &t, &u, &v) && t > 0.0f)
				{
					vec3 c = position + t * n;
					if ((fabs(c.z - position.z) > 0.0001f) && (point.position.z < c.z))
					{
						point.position = c;
						index = i;
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

	void DLPQuickData::autoDlpSources(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam, DLPSupportParam* supportParam, int flag)
	{
		build();

		if ((flag & SUPPORT_VERTEX)== SUPPORT_VERTEX)
		{
			autoDlpVertexSource(m_SupportVertexSources, autoParam);
			for (DLPISource src : m_SupportVertexSources)
			{
				sources.emplace_back(src);
			}
		}
		if ((flag & SUPPORT_FACE)== SUPPORT_FACE)
		{
			autoDlpFaceSource(m_SupportFaceSources, autoParam);
			for (DLPISource src : m_SupportFaceSources)
			{
				sources.emplace_back(src);
			}
		}
		if ((flag & SUPPORT_EDGE) == SUPPORT_EDGE)
		{
			autoDlpEdgeSource(m_SupportEdgeSources, autoParam);
			for (DLPISource src : m_SupportEdgeSources)
			{
				sources.emplace_back(src);
			}
		}

		m_flags.clear();
	}

	void DLPQuickData::autoDlpVertexSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam)
	{
		std::vector<int> supportVertexes;
		float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);
		m_meshTopo->lowestVertex(m_vertexes, supportVertexes);

		for (int vertexID : supportVertexes)
		{
			std::vector<int>& vertexHalfs = m_meshTopo->m_outHalfEdges.at(vertexID);
			int halfSize = (int)vertexHalfs.size();
			bool lowest = true;
			for (int halfIndex = 0; halfIndex < halfSize; ++halfIndex)
			{
				int indexFace = m_meshTopo->faceid(vertexHalfs.at(halfIndex));
				//TriMesh::Face& f = m_mesh->faces.at(indexFace);
				bool faceSupport = m_dotValues.at(indexFace) < (-faceCosValue);

				//if ((m_SupportFacesFlg[indexFace] == false)&&(m_SupportEdgeFlg[vertexID]==false))//包含该点的面不是支撑面
				{

					vec3 vertex = m_vertexes.at(vertexID);
					ivec2 pointIndex = m_triangleChunk->index(vertex);
					int iindex = pointIndex.x + pointIndex.y * m_triangleChunk->m_width;
					std::vector<int>& faceVindex = m_triangleChunk->m_cells.at(iindex);
					int faceVindexSize = faceVindex.size();
					int crossn = 0;
					for(int i=0;i< faceVindexSize;i++)
					{
						if (indexFace == faceVindex.at(i))
							continue;
						TriMesh::Face& tFace = m_mesh->faces.at(faceVindex.at(i));
						vec3& vertex1 = m_vertexes.at(tFace[0]);
						vec3& vertex2 = m_vertexes.at(tFace[1]);
						vec3& vertex3 = m_vertexes.at(tFace[2]);

						vec3 dir(0.0, 0.0, -1.0);
						float t, u, v;
						if (vertex.z > vertex1.z || vertex.z > vertex2.z || vertex.z > vertex3.z)
						{
							if (rayIntersectTriangle(vertex, dir, vertex1, vertex2, vertex3, &t, &u, &v))
							{
								crossn += 1;
							}
						}
					}
					if(crossn %2==0)//向下应与模型有偶数个效点
					{
					DLPISource dlpSource = generateSource(vertex, vec3(0.0f, 0.0f, -1.0f));
					sources.push_back(dlpSource);
					m_SupportVertexFlg[vertexID] = true;
					}


				}
			}


		}
	}

	void DLPQuickData::autoDlpEdgeSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam)
	{
		std::vector<ivec2> supportEdges;
		float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);

		m_meshTopo->hangEdge(m_vertexes, m_faceNormals, m_dotValues, faceCosValue, supportEdges);

		float minDelta = m_triangleChunk->m_gridSize;

		size_t count = sources.size();
		for (ivec2 vertexesID : supportEdges)
		{
			int vertexID1 = vertexesID.x;
			int vertexID2 = vertexesID.y;
			vec3& vertex1 = m_vertexes.at(vertexID1);
			vec3& vertex2 = m_vertexes.at(vertexID2);
			vec3 vertex21 = vertex2- vertex1;
			
			//////////
			float edgeLenXY = trimesh::len(vertex21);
			if (edgeLenXY < minDelta)//长度小于自支撑长度
				continue;

			std::vector<vec3> samplePoints;

			#if 1
			int sampleEdgeNum = ceilf(edgeLenXY / minDelta) +1;
			if (sampleEdgeNum > 0)
			{
				float dt = 1.0f / (float)sampleEdgeNum ;
				for (int sampleIndex = 0; sampleIndex <= sampleEdgeNum; sampleIndex++)
				{
					float t = sampleIndex  * dt;
					vec3 p = vertex1 * (1.0f - t) + vertex2 * t;
					samplePoints.push_back(p);
				}
			}
			#endif
			for (vec3& samplePoint : samplePoints)
			{
				DLPISource dlpSource = generateSource(samplePoint, vec3(0.0f, 0.0f, -1.0f));
				//if (autoTest(dlpSource.position))
				{
					sources.push_back(dlpSource);
					//takeAutoTest(dlpSource.position);
				}
			}
		}
	}
#if 0
	void DLPQuickData::autoDlpFaceSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam)
	{
		float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);
		size_t count = sources.size();

		std::vector<std::vector<int>> supportFaces;
		m_meshTopo->chunkFace(m_dotValues, supportFaces, faceCosValue);

		float minDelta = 3.0f;
		float insertRadius = 2.0f;
		float density = autoParam->density / 100.0f;
		if (density <= 0.0f) density = 0.1f;
		minDelta /= density;
		insertRadius /= density;

		auto testInsert = [](std::vector<trimesh::vec2>& samples, trimesh::vec2& xy, float radius)->bool
		{
			bool canInsert = true;
			float radius2 = radius * radius;
			for (vec2& _xy : samples)
			{
				vec2 delta = _xy - xy;
				if (trimesh::len2(delta) < radius2)
				{
					canInsert = false;
					break;
				}
			}

			return canInsert;
		};

		int pos = 0;
		for (std::vector<int>& faceChunk : supportFaces)
		{
			pos++;
			std::vector<vec3> samplePoints;
			std::vector<vec3> sampleNormals;
			std::vector<vec2> supportsSamplePosition;

			std::sort(faceChunk.begin(), faceChunk.end(), [this](int r1, int r2)->bool {
				return m_dotValues.at(r1) < m_dotValues.at(r2);
				});

			for (int faceID : faceChunk)
			{
				TriMesh::Face& tFace = m_mesh->faces.at(faceID);
				vec3& vertex1 = m_vertexes.at(tFace[0]);
				vec3& vertex2 = m_vertexes.at(tFace[1]);
				vec3& vertex3 = m_vertexes.at(tFace[2]);
				vec3 normal = m_faceNormals.at(faceID);

				vec2 xypoint = vec2(vertex1.x, vertex1.y);
				if (testInsert(supportsSamplePosition, xypoint, insertRadius))
				{
					samplePoints.push_back(vertex1);
					sampleNormals.push_back(normal);
					supportsSamplePosition.push_back(xypoint);
				}

				vec3 e12 = vertex2 - vertex1;
				vec3 e13 = vertex3 - vertex1;
				float len12 = trimesh::len(e12);
				float len13 = trimesh::len(e13);

				int sampleXNum = floorf(len12 / minDelta) - 1;
				int sampleYNum = floorf(len13 / minDelta) - 1;
				if (sampleXNum > 0 || sampleYNum > 0)
				{
					float dxt = 1.0f / (float)(sampleXNum + 1);
					float dyt = 1.0f / (float)(sampleYNum + 1);
					for (int sampleX = 0; sampleX < sampleXNum; ++sampleX)
					{
						float tx = (float)(sampleX + 1) * dxt;
						for (int sampleY = 0; sampleY < sampleYNum; ++sampleY)
						{
							float ty = (float)(sampleY + 1) * dyt;

							if (tx + ty <= 1.0f)
							{
								vec3 offset = tx * e12 + ty * e13;
								vec3 point = offset + vertex1;
								vec2 xypoint0 = vec2(point.x, point.y);
								if (testInsert(supportsSamplePosition, xypoint0, insertRadius))
								{
									samplePoints.push_back(point);
									sampleNormals.push_back(normal);
									supportsSamplePosition.push_back(xypoint0);
								}
							}
						}
					}
				}
			}

			size_t sampleSize = samplePoints.size();
			for (size_t i = 0; i < sampleSize; ++i)
			{
				DLPISource dlpSource = generateSource(samplePoints.at(i), sampleNormals.at(i));
				if (autoTest(dlpSource.position))
				{
					sources.push_back(dlpSource);
					takeAutoTest(dlpSource.position);
				}
			}
		}
	}
#else
void DLPQuickData::autoDlpFaceSource(std::vector<DLPISource>& sources, AutoDLPSupportParam* autoParam)
{
	float faceCosValue = cosf(autoParam->autoAngle * M_PIf / 180.0f);
	size_t count = sources.size();

	m_meshTopo->chunkFace(m_dotValues, m_SupportFaces, faceCosValue);
	std::vector<vec3> samplePoints;
	std::vector<vec3> sampleNormals;
	int testindex = 0;
	for (std::vector<int>& faceChunk : m_SupportFaces)
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
		if ((abs(SectBoxSize.x- SectBoxSize.y)< gridSize*2)&&(sampleXn<3|| sampleYn <3))
			continue;
		for (int faceID : faceChunk)
		{
		TriMesh::Face& tFace = m_mesh->faces.at(faceID);
		vec3& vertex1 = m_vertexes.at(tFace[0]);
		vec3& vertex2 = m_vertexes.at(tFace[1]);
		vec3& vertex3 = m_vertexes.at(tFace[2]);
		vec3 normal = m_faceNormals.at(faceID);
		m_SupportFacesFlg[faceID] = true;
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
					vec3 dir(0.0, 0.0,1.0);
					float t, u, v;
					if (rayIntersectTriangle(xypoint0, dir,  vertex1, vertex2,vertex3, &t, &u, &v))
					{
						vec3 pointCross = xypoint0 + t * dir;
						ivec2 pointIndex = m_triangleChunk->index(pointCross);
						#if 1
						{
							ivec2 pointIndextemp1 = m_triangleChunk->index(vertex1);
							ivec2 pointIndextemp2 = m_triangleChunk->index(vertex2);
							ivec2 pointIndextemp3 = m_triangleChunk->index(vertex3);
							if ((pointIndex == pointIndextemp1)&&(m_SupportVertexFlg[tFace[0]]=true))
							{
								pointCross = vertex1;
							}
							else if ((pointIndex == pointIndextemp2) && (m_SupportVertexFlg[tFace[1]] = true))
							{
								pointCross = vertex2;
							}
							else if ((pointIndex == pointIndextemp3) && (m_SupportVertexFlg[tFace[2]] = true))
							{
								pointCross = vertex3;
							}

						}
						#endif


						int iindex = pointIndex.x + pointIndex.y * m_triangleChunk->m_width;
						m_FaceSampleInCell.at(iindex).push_back(pointCross);
						m_FaceNormalsInCell.at(iindex).push_back(normal);
					}
				}
			}

		}
	}
	///////////////////////////////////

	//for (size_t vvIndex=0; vvIndex < 0; vvIndex++)
	for (size_t vvIndex=0; vvIndex < m_FaceSampleInCell.size(); vvIndex++)
	{
		std::vector<trimesh::vec3>& vertexes = m_FaceSampleInCell.at(vvIndex);
		std::vector<trimesh::vec3>& normals = m_FaceNormalsInCell.at(vvIndex);
		trimesh::vec3 vertexCenter(0.0, 0.0, 0.0);
		for (size_t vIndex = 0; vIndex < vertexes.size(); vIndex++)
		{
			trimesh::vec3 vertex = vertexes.at(vIndex);
			trimesh::vec3 normal = normals.at(vIndex);
			samplePoints.push_back(vertex);
			sampleNormals.push_back(normal);
			//break;
		}

	}
		for (size_t i = 0; i < samplePoints.size(); ++i)
		{
			DLPISource dlpSource = generateSource(samplePoints.at(i), sampleNormals.at(i));
			//if (autoTest(dlpSource.position))
			{
				sources.push_back(dlpSource);
			}
		}

}
#endif
	bool DLPQuickData::autoTest(const trimesh::vec3& point)
	{
		
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
		int nx = (int)floorf((point.x - m_xyBox.min.x) / m_pixel);
		int ny = (int)floorf((point.y - m_xyBox.min.y) / m_pixel);

		if (nx >= 0 && nx < m_width && ny >= 0 && ny < m_height)
		{
			int iindex = nx + ny * m_width;
			m_flags.at(iindex) = false;
		}
	}

}