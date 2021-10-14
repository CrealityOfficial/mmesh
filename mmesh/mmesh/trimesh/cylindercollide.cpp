#include "cylindercollide.h"
#include "mmesh/trimesh/polygonstack.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "trimesh2/Vec3Utils.h"
#include "mmesh/common/print.h"
#include "mmesh/trimesh/uniformpoints.h"
#include "mmesh/trimesh/quaternion.h"
#include "mmesh/util/mnode.h"
#include "mmesh/create/createcylinder.h"
#include <mmesh/trimesh/algrithm3d.h>

#include <map>
#include <list>
#include <assert.h>
#include <cmath>
#include <algorithm>
#include <functional>


namespace mmesh
{
	using namespace trimesh;

	void testCollide(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinderMesh,
		std::vector<trimesh::vec3>& meshNormals, std::vector<trimesh::vec3>& cylinderNormals,
		std::vector<TriMesh::Face>& meshFocusFaces, std::vector<trimesh::TriMesh::Face>& cylinderFocusFaces,
		std::vector<FaceCollide>& meshTris)
	{
		int meshNum = (int)meshFocusFaces.size();
		int cylinderNum = (int)cylinderFocusFaces.size();
		for (int i = 0; i < meshNum; ++i)
			meshTris.at(i).flag = 1;

		//test inner
		for (int i = 0; i < meshNum; ++i)
		{
			TriMesh::Face& focusFace = meshFocusFaces.at(i);

			vec3& focusV1 = mesh->vertices.at(focusFace[0]);
			vec3& focusV2 = mesh->vertices.at(focusFace[1]);
			vec3& focusV3 = mesh->vertices.at(focusFace[2]);

			bool testIn = true;
			for (int j = 0; j < cylinderNum; ++j)
			{
				TriMesh::Face& cylinderFace = cylinderFocusFaces[j];
				vec3& cylinderV1 = cylinderMesh->vertices.at(cylinderFace[0]);
				vec3& n = cylinderNormals.at(j);
				float d1 = n.dot(focusV1 - cylinderV1);
				float d2 = n.dot(focusV2 - cylinderV1);
				float d3 = n.dot(focusV3 - cylinderV1);
				if (d1 > 0.0f || d2 > 0.0f || d3 > 0.0f)
				{
					testIn = false;
					break;
				}
			}

			if (testIn)
			{
				meshTris.at(i).flag = -1;
			}
		}

		struct CTemp
		{
			int index;
			int e1;
			int e2;
			vec3 c1;
			vec3 c2;
		};

		auto fcollid = [](const vec3& tv, const vec3& v1, const vec3& v2, float dv, float d1, float d2, int index, CTemp& temp) {
			vec3 c1 = (dv / (dv - d1)) * v1 - (d1 / (dv - d1)) * tv;
			vec3 c2 = (dv / (dv - d2)) * v2 - (d2 / (dv - d2)) * tv;

			temp.index = index;

			temp.c1 = c1;
			temp.c2 = c2;
			temp.e1 = index;
			temp.e2 = (index + 2) % 3;
		};

		for (int i = 0; i < meshNum; ++i)
		{
			FaceCollide& fc = meshTris.at(i);
			if (fc.flag != -1)
			{
				std::vector<TriTri>& meshTri = fc.tris;
				TriMesh::Face& focusFace = meshFocusFaces.at(i);

				const vec3& focusV1 = mesh->vertices.at(focusFace[0]);
				const vec3& focusV2 = mesh->vertices.at(focusFace[1]);
				const vec3& focusV3 = mesh->vertices.at(focusFace[2]);

				vec3& meshNormal = meshNormals.at(i);
				for (int j = 0; j < cylinderNum; ++j)
				{
					vec3& cylinderNormal = cylinderNormals.at(j);

					if (cylinderNormal == meshNormal)
						continue;

					TriMesh::Face& cylinderFace = cylinderFocusFaces[j];

					const vec3& cylinderV1 = cylinderMesh->vertices.at(cylinderFace[0]);
					const vec3& cylinderV2 = cylinderMesh->vertices.at(cylinderFace[1]);
					const vec3& cylinderV3 = cylinderMesh->vertices.at(cylinderFace[2]);

					float mesh2Cylinder1 = cylinderNormal.dot(focusV1 - cylinderV1);
					float mesh2Cylinder2 = cylinderNormal.dot(focusV2 - cylinderV1);
					float mesh2Cylinder3 = cylinderNormal.dot(focusV3 - cylinderV1);
					float Cylinder2mesh1 = meshNormal.dot(cylinderV1 - focusV1);
					float Cylinder2mesh2 = meshNormal.dot(cylinderV2 - focusV1);
					float Cylinder2mesh3 = meshNormal.dot(cylinderV3 - focusV1);

					std::vector<CTemp> temps;
					float l0 = mesh2Cylinder1 * mesh2Cylinder2;
					float l1 = mesh2Cylinder2 * mesh2Cylinder3;
					float l2 = mesh2Cylinder1 * mesh2Cylinder3;
					if (l0 == 0.0f && l1 == 0.0f && l2 == 0.0f)
						continue;

					CTemp temp;
					if (l0 >= 0.0f && (l1 <= 0.0f || l2 <= 0.0f))
					{
						fcollid(focusV3, focusV1, focusV2, mesh2Cylinder3, mesh2Cylinder1, mesh2Cylinder2, 2, temp);
						temps.push_back(temp);
					}
					else if (l0 < 0.0f)
					{
						if (l1 <= 0.0f)
						{
							fcollid(focusV2, focusV3, focusV1, mesh2Cylinder2, mesh2Cylinder3, mesh2Cylinder1, 1, temp);
							temps.push_back(temp);
						}
						else if (l2 <= 0.0f)
						{
							fcollid(focusV1, focusV2, focusV3, mesh2Cylinder1, mesh2Cylinder2, mesh2Cylinder3, 0, temp);
							temps.push_back(temp);
						}
					}

					l0 = Cylinder2mesh1 * Cylinder2mesh2;
					l1 = Cylinder2mesh2 * Cylinder2mesh3;
					l2 = Cylinder2mesh1 * Cylinder2mesh3;
					if (l0 == 0.0f && l1 == 0.0f && l2 == 0.0f)
						continue;

					if (l0 >= 0.0f && (l1 <= 0.0f || l2 <= 0.0f))
					{
						fcollid(cylinderV3, cylinderV1, cylinderV2, Cylinder2mesh3, Cylinder2mesh1, Cylinder2mesh2, 2, temp);
						temps.push_back(temp);
					}
					else if (l0 < 0.0f)
					{
						if (l1 <= 0.0f)
						{
							fcollid(cylinderV2, cylinderV3, cylinderV1, Cylinder2mesh2, Cylinder2mesh3, Cylinder2mesh1, 1, temp);
							temps.push_back(temp);
						}
						else if (l2 <= 0.0f)
						{
							fcollid(cylinderV1, cylinderV2, cylinderV3, Cylinder2mesh1, Cylinder2mesh2, Cylinder2mesh3, 0, temp);
							temps.push_back(temp);
						}
					}

					if (temps.size() == 2)
					{
						std::vector<vec3> points;
						points.push_back(temps.at(0).c1);
						points.push_back(temps.at(0).c2);
						points.push_back(temps.at(1).c1);
						points.push_back(temps.at(1).c2);
						vec3 axis = points.at(1) - points.at(0);
						normalize(axis);

						std::vector<float> diss(4);
						for (int k = 0; k < 4; ++k)
						{
							diss.at(k) = dot(points.at(k) - points.at(0), axis);
						}

						bool swapped = false;
						if (diss[3] < diss[2])
						{
							std::swap(points.at(3), points.at(2));
							std::swap(diss[3], diss[2]);
							swapped = true;
						}

						float d[3] = {};
						d[0] = mesh2Cylinder1;
						d[1] = mesh2Cylinder2;
						d[2] = mesh2Cylinder3;
						float dd[3] = {};
						dd[0] = Cylinder2mesh1;
						dd[1] = Cylinder2mesh2;
						dd[2] = Cylinder2mesh3;
						TriTri mt;
						mt.topPositive = d[temps[0].index] >= 0.0f;
						mt.cylinderTopIndex = temps[1].index;
						mt.cylinderTopPositive = d[temps[1].index] >= 0.0f;

						vec3 v1;
						vec3 v2;

						if (diss[1] >= diss[2] && diss[3] >= diss[0])
						{
							ivec2 mtIndex(-1, -1);
							ivec2 mcIndex(-1, -1);
							if (diss[2] <= diss[0])
							{
								if (diss[3] < diss[1])
								{
									v1 = points.at(0);
									v2 = points.at(3);
									mtIndex[0] = temps[0].e1;
									mcIndex[1] = temps[1].e2;
								}
								else
								{
									v1 = points.at(0);
									v2 = points.at(1);
									mtIndex[0] = temps[0].e1;
									mtIndex[1] = temps[0].e2;
								}
							}
							else if (diss[2] <= diss[1])
							{
								if (diss[3] < diss[1])
								{
									v1 = points.at(2);
									v2 = points.at(3);
									mcIndex[0] = temps[1].e1;
									mcIndex[1] = temps[1].e2;
								}
								else
								{
									v1 = points.at(2);
									v2 = points.at(1);
									mtIndex[1] = temps[0].e2;
									mcIndex[0] = temps[1].e1;
								}
							}

							mt.v1 = v1;
							mt.v2 = v2;

							mt.index = mtIndex;
							mt.cindex = mcIndex;
							meshTri.push_back(mt);
							fc.flag = 0;
						}
					}
				}
			}
		}
	}

	void pointInner(std::vector<int>& pointPosition, std::vector<trimesh::TriMesh::Face>& focusFaces,int n)
	{
		TriMesh::Face& cylinderFace = focusFaces.at(n);
		if (n >= pointPosition.size())
		{
			return;
		}

		if (pointPosition[cylinderFace[0]] != -1
			&& pointPosition[cylinderFace[1]] != -1
			&& pointPosition[cylinderFace[2]] != -1)
		{
			return;
		}

		if (pointPosition[cylinderFace[0]] != -1
			|| pointPosition[cylinderFace[1]] != -1
			|| pointPosition[cylinderFace[2]] != -1)
		{
			int index = std::max(pointPosition[cylinderFace[0]], pointPosition[cylinderFace[1]]);
			index = std::max(index, pointPosition[cylinderFace[2]]);
			pointPosition[cylinderFace[0]] = index;
			pointPosition[cylinderFace[1]] = index;
			pointPosition[cylinderFace[2]] = index;
		}
		pointInner(pointPosition,focusFaces,++n);
		return;
	}

	void generateNewTriangles(std::vector<trimesh::TriMesh::Face>& focusFaces, trimesh::TriMesh* mesh, std::vector<trimesh::vec3>& normals,
		std::vector<FaceCollide>& tris, std::vector<trimesh::vec3>& newTriangles, bool positive, std::vector<int>& cylinderFlag, ccglobal::Tracer* tracer, DrillDebugger* debugger)
	{
		int meshNum = (int)focusFaces.size();
		std::vector<std::vector<bool>> vctInner;
		vctInner.resize(meshNum, std::vector<bool>(3,true));
		for (int i = 0; i < meshNum; ++i)
		{
			trimesh::TriMesh::Face& face = focusFaces.at(i);
			FaceCollide& fcollide = tris.at(i);
			std::vector<TriTri>& tritri = fcollide.tris;
			size_t size = tritri.size();
			if (size > 0)
			{
				std::vector<TriSegment> trisegments(size);
				for (int j = 0; j < size; ++j)
				{
					TriSegment& seg = trisegments.at(j);
					TriTri& tri = tritri.at(j);
					seg.index = tri.index;
					seg.topPositive = tri.topPositive;
					seg.v1 = tri.v1;
					seg.v2 = tri.v2;
				}
				std::vector<trimesh::vec3> triangles;
				if (splitTriangle(mesh->vertices[face[0]], mesh->vertices[face[1]], mesh->vertices[face[2]],
					trisegments, positive, triangles, vctInner[i],tracer))
				{
					newTriangles.insert(newTriangles.end(), triangles.begin(), triangles.end());
				}
				else
				{
					if (tracer)
						tracer->failed("error split Triangle");
					assert("error split Triangle");
				}

				if (debugger)
				{
					SplitTriangleCache cache;
					cache.v0 = mesh->vertices[face[0]];
					cache.v1 = mesh->vertices[face[1]];
					cache.v2 = mesh->vertices[face[2]];
					cache.segments = trisegments;
					debugger->onTriangleSplitCache(cache);
				}
			}
		}

		if (!positive)
		{
			std::vector<int> pointPosition;
			pointPosition.resize(mesh->vertices.size(), -1);
			//init
			for (size_t i = 0; i < tris.size(); i++)
			{
				if (tris[i].flag != 0)
					continue;
				TriMesh::Face& cylinderFace = mesh->faces.at(i);
				pointPosition[cylinderFace[0]]= vctInner[i][0];
				pointPosition[cylinderFace[1]] = vctInner[i][1];
				pointPosition[cylinderFace[2]] = vctInner[i][2];
			}
			//recursion
			for (size_t i = 0; i < tris.size(); i++)
			{
				if (tris[i].flag != 0)
					continue;
				pointInner(pointPosition, focusFaces, i);
			}

			for (size_t i = 0; i < tris.size(); i++)
			{
				if (tris[i].flag != 1)
					continue;
				TriMesh::Face& cylinderFace = mesh->faces.at(i);
				if (pointPosition[cylinderFace[0]] ==0
					|| pointPosition[cylinderFace[1]] == 0
					|| pointPosition[cylinderFace[2]] == 0)
				{
					cylinderFlag[i] = -1;
				}		
			}
		}
	}

	void generateCylinderTriangles(trimesh::TriMesh* cylinder, std::vector<trimesh::vec3>& normals,
		std::vector<FaceCollide>& tris, std::vector<trimesh::vec3>& newTriangles, bool positive)
	{

	}

	trimesh::TriMesh* generatePatchMesh(trimesh::TriMesh* oldMesh, FlagPatch& flagFaces, int flag)
	{
		auto fillmesh = [&oldMesh](trimesh::TriMesh* m) {
			for (trimesh::TriMesh::Face& f : m->faces)
			{
				m->vertices.push_back(oldMesh->vertices.at(f.x));
				m->vertices.push_back(oldMesh->vertices.at(f.y));
				m->vertices.push_back(oldMesh->vertices.at(f.z));
			}

			int index = 0;
			//remap
			for (trimesh::TriMesh::Face& f : m->faces)
			{
				f.x = index++;
				f.y = index++;
				f.z = index++;
			}
		};

		trimesh::TriMesh* newMesh = new trimesh::TriMesh();
		int size = (int)flagFaces.size();
		if (size > 0)
		{
			for (int i = 0; i < size; ++i)
			{
				if (flagFaces.at(i) == flag)
					newMesh->faces.push_back(oldMesh->faces.at(i));
			}
		}
		fillmesh(newMesh);
		return newMesh;
	}

	trimesh::TriMesh* generateAppendMesh(trimesh::TriMesh* oldMesh, FlagPatch& flagFaces,
		TriPatch& newTriangles, int flag, ccglobal::Tracer* tracer)
	{
		if (!oldMesh)
		{
			if (tracer)
				tracer->failed("Mesh is empty");
			return nullptr;
		}

		trimesh::TriMesh* newMesh = generatePatchMesh(oldMesh, flagFaces, flag);
		
		auto addmesh = [](trimesh::TriMesh* mesh, const trimesh::vec3& v1,
			const trimesh::vec3& v2, const trimesh::vec3& v3) {
				int index = (int)mesh->vertices.size();
				mesh->vertices.push_back(v1);
				mesh->vertices.push_back(v2);
				mesh->vertices.push_back(v3);
		
				mesh->faces.push_back(trimesh::TriMesh::Face(index, index + 1, index + 2));
		};
		
		int newVertexSize = (int)(newTriangles.size() / 3);
		for (int i = 0; i < newVertexSize; ++i)
			addmesh(newMesh, newTriangles.at(3 * i), newTriangles.at(3 * i + 1), newTriangles.at(3 * i + 2));

		return newMesh;
	}

	OptimizeCylinderCollide::OptimizeCylinderCollide(trimesh::TriMesh* mesh, trimesh::TriMesh* cylinder,
		ccglobal::Tracer* tracer, DrillDebugger* debugger)
		:m_mesh(mesh)
		, m_cylinder(cylinder)
		, focusTriangle(0)
		, m_cylinderResolution(0)
		, m_tracer(tracer)
		, m_debugger(debugger)
		, cylinderTriangles(0)
	{
		if (m_mesh && m_cylinder && m_mesh->faces.size() > 0)
			calculate();
	}
	
	OptimizeCylinderCollide::OptimizeCylinderCollide(trimesh::TriMesh* mesh,
		int resolution, double radius, double depth, trimesh::point pointStart, trimesh::point dir,
		ccglobal::Tracer* tracer, DrillDebugger* debugger)
		:m_mesh(mesh)
		, m_cylinder(nullptr)
		, focusTriangle(0)
		, m_tracer(tracer)
		, m_debugger(debugger)
		, cylinderTriangles(0)
		, m_cylinderResolution(resolution)
		, m_cylinderRadius(radius)
		, m_cylinderDepth(depth)
		, m_cylinderPointStart(pointStart)
		, m_cylinderDir(dir)
	{
		if (m_mesh && m_mesh->faces.size() > 0)
			mycalculate();
	}

	OptimizeCylinderCollide::~OptimizeCylinderCollide()
	{
		//delete
		if (m_cylinder && m_cylinderResolution > 0)
		{
			delete m_cylinder;
			m_cylinder = nullptr;
		}
	}

	void OptimizeCylinderCollide::calculate()
	{
		m_cylinder->need_bbox();
		box3 cylinderBox = m_cylinder->bbox;

		int faces = (int)m_mesh->faces.size();
		totalMeshFlag.resize(faces, 0);

		for (int i = 0; i < faces; ++i)
		{
			TriMesh::Face& face = m_mesh->faces.at(i);

			box3 b;
			for (int j = 0; j < 3; ++j)
			{
				b += m_mesh->vertices.at(face[j]);
			}

			if (cylinderBox.intersects(b))
			{
				meshFocusFacesMapper.push_back(i);
				meshFocusFaces.push_back(face);
			}
			else
			{
				totalMeshFlag.at(i) = 1;
			}
		}

		focusTriangle = (int)meshFocusFaces.size();
		cylinderTriangles = (int)m_cylinder->faces.size();
		focusNormals.resize(focusTriangle);
		cylinderNormals.resize(cylinderTriangles);
		cylinderFlag.resize(cylinderTriangles);
		for (int j = 0; j < cylinderTriangles; ++j)
		{
			TriMesh::Face& cylinderFace = m_cylinder->faces.at(j);

			vec3& cylinderV1 = m_cylinder->vertices.at(cylinderFace[0]);
			vec3& cylinderV2 = m_cylinder->vertices.at(cylinderFace[1]);
			vec3& cylinderV3 = m_cylinder->vertices.at(cylinderFace[2]);

			vec3 n = (cylinderV2 - cylinderV1) TRICROSS(cylinderV3 - cylinderV1);
			cylinderNormals.at(j) = normalized(n);
		}

		for (int j = 0; j < focusTriangle; ++j)
		{
			TriMesh::Face& focusFace = meshFocusFaces.at(j);

			vec3& meshV1 = m_mesh->vertices.at(focusFace[0]);
			vec3& meshV2 = m_mesh->vertices.at(focusFace[1]);
			vec3& meshV3 = m_mesh->vertices.at(focusFace[2]);

			vec3 n = (meshV2 - meshV1) TRICROSS(meshV3 - meshV1);
			focusNormals.at(j) = normalized(n);
		}

		tracerFormartPrint(m_tracer, "OptimizeCylinderCollide meshFocus [%d], cylinder [%d]",
			focusTriangle, cylinderTriangles);
		if (m_debugger)
			m_debugger->onCylinderBoxFocus(meshFocusFaces);

		meshTris.resize(focusTriangle);
		cylinderTris.resize(cylinderTriangles);

		testCollide(m_mesh, m_cylinder, focusNormals, cylinderNormals,
			meshFocusFaces, m_cylinder->faces, meshTris);

		testCollide(m_cylinder, m_mesh, cylinderNormals, focusNormals,
			m_cylinder->faces, meshFocusFaces, cylinderTris);

		for (int i = 0; i < focusTriangle; ++i)
		{
			int index = meshFocusFacesMapper.at(i);
			totalMeshFlag.at(index) = meshTris.at(i).flag;
		}
		for (int i = 0; i < cylinderTriangles; ++i)
		{
			cylinderFlag.at(i) = cylinderTris.at(i).flag;
		}

		if (m_debugger)
		{
			//m_debugger->onMeshOuter(generatePatchMesh(m_mesh, totalMeshFlag, CylinderCollideOuter));
			//m_debugger->onMeshCollide(generatePatchMesh(m_mesh, totalMeshFlag, CylinderCollideCollide));
			//m_debugger->onMeshInner(generatePatchMesh(m_mesh, totalMeshFlag, CylinderCollideInner));

			m_debugger->onMeshOuter(generatePatchMesh(m_cylinder, cylinderFlag, CylinderCollideOuter));
			m_debugger->onMeshCollide(generatePatchMesh(m_cylinder, cylinderFlag, CylinderCollideCollide));
			m_debugger->onMeshInner(generatePatchMesh(m_cylinder, cylinderFlag, CylinderCollideInner));
		}
	}

	void OptimizeCylinderCollide::mycalculate()
	{
		int faces = (int)m_mesh->faces.size();
		totalMeshFlag.resize(faces, 0);
		//m_cylinder->need_bbox();
		//box3 cylinderBox_old = m_cylinder->bbox;

		mmesh::point ZAXIS(0.0f,0.0f,1.0f);
		trimesh::vec3 ax = m_cylinderDir;
		trimesh::quaternion quat= trimesh::quaternion::fromDirection(ax, ZAXIS);

		trimesh::fxform xf = mmesh::fromQuaterian(quat);

		m_mesh->need_bbox();
		m_cylinderPointStart = m_cylinderPointStart + 2 * (-m_cylinderDir);
		double cylinderInitDepth = 2 * m_mesh->bbox.radius() + 2;

		//box3 cylinderBox_new;
		//int cylinderVertexNum = (int)m_cylinder->vertices.size();
		//for (int i = 0; i < cylinderVertexNum; ++i)
		//{
		//	trimesh::vec3 v = xf * m_cylinder->vertices.at(i);
		//	cylinderBox_new += v;
		//}
		
		trimesh::vec3 newStartPos = xf * m_cylinderPointStart;
		trimesh::vec3 boxMin = trimesh::vec3(newStartPos.x - m_cylinderRadius, newStartPos.y - m_cylinderRadius, newStartPos.z);
		trimesh::vec3 boxMax = trimesh::vec3(newStartPos.x + m_cylinderRadius, newStartPos.y + m_cylinderRadius, newStartPos.z + cylinderInitDepth);
		box3 cylinderBox_new(boxMin, boxMax);

		for (int i = 0; i < faces; ++i)
		{
			TriMesh::Face& face = m_mesh->faces.at(i);
			box3 b;
			for (int j = 0; j < 3; ++j)
			{
				b += xf * m_mesh->vertices.at(face[j]);
			}

			if (cylinderBox_new.intersects(b))
			{
				meshFocusFacesMapper.push_back(i);
				meshFocusFaces.push_back(face);
			}
			else
			{
				totalMeshFlag.at(i) = 1;
			}
		}

		// 确定打孔实际深度，生成圆柱体网格
		m_cylinderDepth = getDrillDepth();
		if (m_cylinderDepth <= 0)
			return;

		m_cylinderDepth += 2.0;

		m_cylinder = createCylinderMesh(newStartPos + m_cylinderDepth * ZAXIS, newStartPos, m_cylinderRadius, m_cylinderResolution, 0.0);
		trimesh::fxform invXF = trimesh::inv(xf);
		for (int i = 0; i < m_cylinder->vertices.size(); ++i)
		{
			m_cylinder->vertices[i] = invXF * m_cylinder->vertices[i];
		}

		focusTriangle = (int)meshFocusFaces.size();
		cylinderTriangles = (int)m_cylinder->faces.size();
		focusNormals.resize(focusTriangle);
		cylinderNormals.resize(cylinderTriangles);
		cylinderFlag.resize(cylinderTriangles);
		for (int j = 0; j < cylinderTriangles; ++j)
		{
			TriMesh::Face& cylinderFace = m_cylinder->faces.at(j);

			vec3& cylinderV1 = m_cylinder->vertices.at(cylinderFace[0]);
			vec3& cylinderV2 = m_cylinder->vertices.at(cylinderFace[1]);
			vec3& cylinderV3 = m_cylinder->vertices.at(cylinderFace[2]);

			vec3 n = (cylinderV2 - cylinderV1) TRICROSS(cylinderV3 - cylinderV1);
			cylinderNormals.at(j) = normalized(n);
		}

		for (int j = 0; j < focusTriangle; ++j)
		{
			TriMesh::Face& focusFace = meshFocusFaces.at(j);

			vec3& meshV1 = m_mesh->vertices.at(focusFace[0]);
			vec3& meshV2 = m_mesh->vertices.at(focusFace[1]);
			vec3& meshV3 = m_mesh->vertices.at(focusFace[2]);

			vec3 n = (meshV2 - meshV1) TRICROSS(meshV3 - meshV1);
			focusNormals.at(j) = normalized(n);
		}

		tracerFormartPrint(m_tracer, "OptimizeCylinderCollide meshFocus [%d], cylinder [%d]",
			focusTriangle, cylinderTriangles);
		if (m_debugger)
			m_debugger->onCylinderBoxFocus(meshFocusFaces);


		meshTris.resize(focusTriangle);
		cylinderTris.resize(cylinderTriangles);

		testCollide(m_mesh, m_cylinder, focusNormals, cylinderNormals,
			meshFocusFaces, m_cylinder->faces, meshTris);

		testCollide(m_cylinder, m_mesh, cylinderNormals, focusNormals,
			m_cylinder->faces, meshFocusFaces, cylinderTris);

		for (int i = 0; i < focusTriangle; ++i)
		{
			int index = meshFocusFacesMapper.at(i);
			totalMeshFlag.at(index) = meshTris.at(i).flag;
		}
		for (int i = 0; i < cylinderTriangles; ++i)
		{
			cylinderFlag.at(i) = cylinderTris.at(i).flag;
		}

		if (m_debugger)
		{
			//m_debugger->onMeshOuter(generatePatchMesh(m_mesh, totalMeshFlag, CylinderCollideOuter));
			//m_debugger->onMeshCollide(generatePatchMesh(m_mesh, totalMeshFlag, CylinderCollideCollide));
			//m_debugger->onMeshInner(generatePatchMesh(m_mesh, totalMeshFlag, CylinderCollideInner));

			m_debugger->onMeshOuter(generatePatchMesh(m_cylinder, cylinderFlag, CylinderCollideOuter));
			m_debugger->onMeshCollide(generatePatchMesh(m_cylinder, cylinderFlag, CylinderCollideCollide));
			m_debugger->onMeshInner(generatePatchMesh(m_cylinder, cylinderFlag, CylinderCollideInner));
		}
	}

	double OptimizeCylinderCollide::getDrillDepth()
	{
		// focus 集合按照离打孔起点距离由近及远排序
		std::map<int, double> faceDisMap;
		for (int i = 0; i < meshFocusFacesMapper.size(); i++)
		{
			trimesh::TriMesh::Face& face = m_mesh->faces[meshFocusFacesMapper[i]];
			double dis20 = trimesh::dist2(m_cylinderPointStart, m_mesh->vertices[face[0]]);
			double dis21 = trimesh::dist2(m_cylinderPointStart, m_mesh->vertices[face[1]]);
			double dis22 = trimesh::dist2(m_cylinderPointStart, m_mesh->vertices[face[2]]);
			faceDisMap[meshFocusFacesMapper[i]] = std::min(dis20, std::min(dis21, dis22));
		}

		trimesh::TriMesh* modelMesh = m_mesh;
		trimesh::vec3 cylinderStartPos = m_cylinderPointStart;
		std::function<bool(int, int)> compareFunc = [&modelMesh, &cylinderStartPos, &faceDisMap](int A, int B)->bool
		{
			double dis2A = faceDisMap[A];
			double dis2B = faceDisMap[B];

			return dis2A < dis2B;
		};
		std::sort(meshFocusFacesMapper.begin(), meshFocusFacesMapper.end(), compareFunc);

		int i;
		int layerFlag = 0;
		int layerCount = 0;
		float t, u, v;
		double drillDepth = 0.0;
		double presetDepth2 = m_cylinderDepth * m_cylinderDepth;
		trimesh::dvec3 intersectedPos(NAN, NAN, NAN);
		for (i = 0; i < meshFocusFacesMapper.size(); i++)
		{
			trimesh::TriMesh::Face& face = m_mesh->faces[meshFocusFacesMapper[i]];
			bool isIntersected = rayIntersectTriangle(m_cylinderPointStart, m_cylinderDir, m_mesh->vertices[face[0]], m_mesh->vertices[face[1]], m_mesh->vertices[face[2]],
				&t, &u, &v);
			if (isIntersected)
			{
				trimesh::vec3 faceNormal = trimesh::normalized((m_mesh->vertices[face[1]] - m_mesh->vertices[face[0]]) TRICROSS(m_mesh->vertices[face[2]] - m_mesh->vertices[face[0]]));
				if (faceNormal.dot(m_cylinderDir) < 0)
					layerFlag = 1;
				else if (layerFlag > 0)
					layerFlag = -1;

				intersectedPos = m_cylinderPointStart + t * m_cylinderDir;

				double depth2 = trimesh::dist2(intersectedPos, trimesh::dvec3(m_cylinderPointStart));

				if (m_cylinderDepth > 0)
				{
					if (depth2 < presetDepth2 && layerFlag == -1)
					{
						drillDepth = depth2;
						layerFlag = 0;
					}
					else if (depth2 >= presetDepth2)
						break;
				}
				else if(layerFlag == -1)
				{
					drillDepth = depth2;
					break;
				}
			}
		}

		 // 剔除距离比打孔深度长的三角面（暂不启用）
		if (drillDepth > 0)
		{
			drillDepth = sqrt(drillDepth);

			//double offset = 100;
			//double farthestFaceDis = drillDepth;
			//do
			//{
			//	i++;
			//} while (i < meshFocusFacesMapper.size() && fabs(faceDisMap[meshFocusFacesMapper[i]] - farthestFaceDis) < offset);

			//if (i < meshFocusFacesMapper.size())
			//{
			//	for (int j = i; j < meshFocusFacesMapper.size(); j++)
			//	{
			//		totalMeshFlag[meshFocusFacesMapper[j]] = 1;
			//	}

			//	meshFocusFacesMapper.erase(meshFocusFacesMapper.begin() + i, meshFocusFacesMapper.end());
			//}

			meshFocusFaces.clear();
			for (i = 0; i < meshFocusFacesMapper.size(); i++)
			{
				meshFocusFaces.push_back(m_mesh->faces[meshFocusFacesMapper[i]]);
			}
		}

		return drillDepth;
	}

	bool OptimizeCylinderCollide::lineCollideTriangle(trimesh::dvec3 linePos, trimesh::dvec3 lineDir, trimesh::dvec3 A, trimesh::dvec3 B, trimesh::dvec3 C, trimesh::dvec3& intersectedPos)
	{
		trimesh::dvec3 faceNormal = trimesh::normalized((B - A) TRICROSS(C - A));

		double temp = lineDir.x * faceNormal.x + lineDir.y * faceNormal.y + lineDir.z * faceNormal.z;
		if (temp == 0)
			return false;

		double t = ((A.x - linePos.x) * faceNormal.x + (A.y - linePos.y) * faceNormal.y + (A.z - linePos.z) * faceNormal.z) / temp;

		trimesh::dvec3 intersecedPosTemp(lineDir.x * t + linePos.x, lineDir.y * t + linePos.y, lineDir.z * t + linePos.z);

		trimesh::dvec3 v0 = C - A;
		trimesh::dvec3 v1 = B - A;
		trimesh::dvec3 v2 = intersecedPosTemp - A;
		double dot00 = v0 DOT v0;
		double dot01 = v0 DOT v1;
		double dot02 = v0 DOT v2;
		double dot11 = v1 DOT v1;
		double dot12 = v1 DOT v2;

		double bottom = 1.0 / (dot00 * dot11 - dot01 * dot01);
		double u = (dot11 * dot02 - dot01 * dot12) * bottom;
		double v = (dot00 * dot12 - dot01 * dot02) * bottom;
		if (u >= 0 && v >= 0 && u + v <= 1)
		{
			intersectedPos = intersecedPosTemp;
			return true;
		}

		return false;
	}

	trimesh::TriMesh* OptimizeCylinderCollide::createCylinderMesh(trimesh::vec3 top, trimesh::vec3 bottom, float radius, int num, float theta)
	{
		trimesh::TriMesh* mesh = new trimesh::TriMesh();

		int hPart = num;

		trimesh::vec3 start = bottom;
		trimesh::vec3 end = top;

		trimesh::vec3 dir = trimesh::normalized(end - start);
		trimesh::quaternion q = trimesh::quaternion::fromDirection(dir, trimesh::vec3(0.0f, 0.0f, 1.0f));

		theta *= M_PIf / 180.0f;
		float deltaTheta = M_PIf * 2.0f / (float)(hPart);
		std::vector<float> cosValue;
		std::vector<float> sinValue;
		for (int i = 0; i < hPart; ++i)
		{
			cosValue.push_back(cos(deltaTheta * (float)i + theta));
			sinValue.push_back(sin(deltaTheta * (float)i + theta));
		}

		std::vector<trimesh::vec3> baseNormals;
		for (int i = 0; i < hPart; ++i)
		{
			baseNormals.push_back(trimesh::vec3(cosValue[i], sinValue[i], 0.0f));
		}

		int vertexNum = 2 * hPart;
		std::vector<trimesh::vec3> points(vertexNum);
		int faceNum = 4 * hPart - 4;
		mesh->faces.resize(faceNum);

		int vertexIndex = 0;
		for (int i = 0; i < hPart; ++i)
		{
			trimesh::vec3 n = q * baseNormals[i];
			trimesh::vec3 s = start + n * radius;
			points.at(vertexIndex) = trimesh::vec3(s.x, s.y, s.z);
			++vertexIndex;
			trimesh::vec3 e = end + n * radius;
			points.at(vertexIndex) = trimesh::vec3(e.x, e.y, e.z);
			++vertexIndex;
		}
		mesh->vertices.swap(points);

		auto fvindex = [&hPart](int layer, int index)->int {
			return layer + 2 * index;
		};

		int faceIndex = 0;
		for (int i = 0; i < hPart; ++i)
		{
			int v1 = fvindex(0, i);
			int v2 = fvindex(1, i);
			int v3 = fvindex(0, (i + 1) % hPart);
			int v4 = fvindex(1, (i + 1) % hPart);

			trimesh::TriMesh::Face& f1 = mesh->faces.at(faceIndex);
			f1[0] = v1;
			f1[1] = v3;
			f1[2] = v2;
			++faceIndex;
			trimesh::TriMesh::Face& f2 = mesh->faces.at(faceIndex);
			f2[0] = v2;
			f2[1] = v3;
			f2[2] = v4;
			++faceIndex;
		}

		for (int i = 1; i < hPart - 1; ++i)
		{
			trimesh::TriMesh::Face& f1 = mesh->faces.at(faceIndex);
			f1[0] = 0;
			f1[1] = fvindex(0, i + 1);
			f1[2] = fvindex(0, i);
			++faceIndex;
		}

		for (int i = 1; i < hPart - 1; ++i)
		{
			trimesh::TriMesh::Face& f1 = mesh->faces.at(faceIndex);
			f1[0] = 1;
			f1[1] = fvindex(1, i);
			f1[2] = fvindex(1, i + 1);
			++faceIndex;
		}

		return mesh;
	}

	bool OptimizeCylinderCollide::valid()
	{
		return focusTriangle > 0;
	}

	trimesh::TriMesh* OptimizeCylinderCollide::drill(ccglobal::Tracer* tracer)
	{
		TriPatch newMeshTriangles;
		generateNewTriangles(meshFocusFaces, m_mesh, focusNormals, meshTris, newMeshTriangles, true, totalMeshFlag, tracer, nullptr);
		if (tracer)
		{
			tracer->progress(0.4f);
		}
		TriPatch newCylinderTriangles;
		generateNewTriangles(m_cylinder->faces, m_cylinder, cylinderNormals, cylinderTris, newCylinderTriangles, false, cylinderFlag, tracer, m_debugger);
		if (tracer)
		{
			tracer->progress(0.6f);
		}
		trimesh::TriMesh* Mout = generateAppendMesh(m_mesh, totalMeshFlag, newMeshTriangles, CylinderCollideOuter, tracer);
		if (tracer)
		{
			tracer->progress(0.8f);
		}
		trimesh::TriMesh* Cyin = generateAppendMesh(m_cylinder, cylinderFlag, newCylinderTriangles, CylinderCollideInner, tracer);
		if (tracer)
		{
			tracer->progress(0.9f);
		}
		return postProcess(Mout, Cyin);
	}

	trimesh::TriMesh* OptimizeCylinderCollide::postProcess(trimesh::TriMesh* Mout, trimesh::TriMesh* Cin)
	{
		if (Cin)
		{
			tracerFormartPrint(m_tracer, "drill reverseTrimesh.");
			mmesh::reverseTriMesh(Cin);
		}

		tracerFormartPrint(m_tracer, "drill mergeTriMesh.");
		std::vector<trimesh::TriMesh*> meshes;
		if(Mout)
			meshes.push_back(Mout);
		if(Cin)
			meshes.push_back(Cin);
		trimesh::TriMesh* drillMesh = new trimesh::TriMesh();
		mergeTriMesh(drillMesh, meshes);

		for (trimesh::TriMesh* mesh : meshes)
			delete mesh;
		meshes.clear();

		tracerFormartPrint(m_tracer, "drill dumplicateMesh.");
		dumplicateMesh(drillMesh, m_tracer);

		if (m_tracer)
			m_tracer->success();
		return drillMesh;
	}
}