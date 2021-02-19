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

		float prevWorkLen = 0.4f;
		float progLens[3] = { 1.0f - prevWorkLen, 1.0f - prevWorkLen, 1.0f - prevWorkLen };
		float progStarts[3] = { prevWorkLen, prevWorkLen, prevWorkLen };
		float totallen = 0;
		if (flag & 1)
		{
			totallen += 1;
		}
		if (flag & 2)
		{
			totallen += 2;
		}
		if (flag & 4)
		{
			totallen += 5;
		}
		if (progLens > 0)
		{
			progLens[0] *= 1.0 / totallen;
			progLens[1] *= 2.0 / totallen;
			progLens[2] *= 5.0 / totallen;

			progStarts[1] = progStarts[0] + progLens[0];
			progStarts[2] = progStarts[1] + progLens[1];
		}

		trimesh::vec3 size = m_xyBox.size();
		float maxLen = std::max(size.x, size.y);
		float p = (maxLen / 50.0f < supportParam->middleRadius) ? supportParam->middleRadius : maxLen / 50.0f;
		m_pixel = 100.0f * p / autoParam->density;

		m_width = (int)(ceilf(size.x / m_pixel) + 1);
		m_height = (int)(ceilf(size.y / m_pixel) + 1);

		if (m_width > 0 && m_height > 0)
			m_flags.resize(m_width * m_height, true);

		if (flag & 1)
		{
			autoDlpVertexSource(sources);
		}

		if (flag & 2)
		{
			autoDlpEdgeSource(sources);
		}

		if (flag & 4)
		{
			autoDlpFaceSource(sources, autoParam);
		}

		m_flags.clear();
	}

	void DLPQuickData::autoDlpVertexSource(std::vector<DLPISource>& sources)
	{
		std::vector<int> supportVertexes;
		m_meshTopo->lowestVertex(m_vertexes, supportVertexes);
		int pos = 0;

		size_t count = sources.size();
		for (int vertexID : supportVertexes)
		{
			vec3 vertex = m_vertexes.at(vertexID);
			DLPISource dlpSource = generateSource(vertex, vec3(0.0f, 0.0f, -1.0f));
			if (autoTest(dlpSource.position))
			{
				sources.push_back(dlpSource);
				takeAutoTest(dlpSource.position);
			}
			pos++;
		}
	}

	void DLPQuickData::autoDlpEdgeSource(std::vector<DLPISource>& sources)
	{
		std::vector<ivec2> supportEdges;
		m_meshTopo->hangEdge(m_vertexes, m_faceNormals, m_dotValues, supportEdges);

		float minDelta = 3.0f;
		int pos = 0;

		size_t count = sources.size();
		for (ivec2 vertexesID : supportEdges)
		{
			pos++;
			int vertexID1 = vertexesID.x;
			int vertexID2 = vertexesID.y;
			vec3& vertex1 = m_vertexes.at(vertexID1);
			vec3& vertex2 = m_vertexes.at(vertexID2);

			box2 box2D;
			box2D += vec2(vertex1.x, vertex1.y);
			box2D += vec2(vertex2.x, vertex2.y);

			vec2 boxSize = box2D.size();
			float maxLen = boxSize.x > boxSize.y ? boxSize.x : boxSize.y;

			std::vector<vec3> samplePoints;

			samplePoints.push_back(vertex1);
			samplePoints.push_back(vertex2);

			int sampleEdgeNum = floorf(maxLen / minDelta) - 1;
			if (sampleEdgeNum > 0)
			{
				float dt = 1.0f / (float)(sampleEdgeNum + 1);
				for (int sampleIndex = 0; sampleIndex < sampleEdgeNum; ++sampleIndex)
				{
					float t = (float)(sampleIndex + 1) * dt;
					vec3 p = vertex1 * (1.0f - t) + vertex2 * t;
					samplePoints.push_back(p);
				}
			}

			for (vec3& samplePoint : samplePoints)
			{
				DLPISource dlpSource = generateSource(samplePoint, vec3(0.0f, 0.0f, -1.0f));
				if (autoTest(dlpSource.position))
				{
					sources.push_back(dlpSource);
					takeAutoTest(dlpSource.position);
				}
			}
		}
	}

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