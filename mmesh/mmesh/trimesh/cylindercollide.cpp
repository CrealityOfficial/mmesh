#include "cylindercollide.h"
#include "mmesh/trimesh/polygonstack.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "trimesh2/Vec3Utils.h"
#include "mmesh/common/print.h"
#include "mmesh/trimesh/uniformpoints.h"

#include <map>
#include <list>
#include <assert.h>

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

	void generateNewTriangles(std::vector<trimesh::TriMesh::Face>& focusFaces, trimesh::TriMesh* mesh, std::vector<trimesh::vec3>& normals,
		std::vector<FaceCollide>& tris, std::vector<trimesh::vec3>& newTriangles, bool positive, DrillDebugger* debugger)
	{
		int meshNum = (int)focusFaces.size();
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
					trisegments, positive, triangles))
				{
					newTriangles.insert(newTriangles.end(), triangles.begin(), triangles.end());
				}
				else
				{
					assert("error cylinder is watertight");
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
		TriPatch& newTriangles, int flag)
	{
		if (!oldMesh)
			return nullptr;

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
		, m_tracer(tracer)
		, m_debugger(debugger)
		, cylinderTriangles(0)
	{
		if (m_mesh && m_cylinder && m_mesh->faces.size() > 0)
			calculate();
	}

	OptimizeCylinderCollide::~OptimizeCylinderCollide()
	{

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
			m_debugger->onMeshOuter(generatePatchMesh(m_mesh, totalMeshFlag, CylinderCollideOuter));
			m_debugger->onMeshCollide(generatePatchMesh(m_mesh, totalMeshFlag, CylinderCollideCollide));
			m_debugger->onMeshInner(generatePatchMesh(m_mesh, totalMeshFlag, CylinderCollideInner));
		}
		//std::vector<std::vector<TriTri>> cylinderTris(cylinderTriangles);
		//testCollide(cylinderFlag, m_cylinder, m_mesh, cylinderNormals, focusNormals, m_cylinder->faces, meshFocusFaces, cylinderTris);
		//generateNewTriangles(m_cylinder->faces, m_cylinder, cylinderNormals, cylinderTris, newCylinderTriangles, false);
		//
		//for (int i = 0; i < cylinderTriangles; ++i)
		//{
		//	TriMesh::Face& f = m_cylinder->faces.at(i);
		//	std::vector<TriTri>& tri = cylinderTris.at(i);
		//	if (cylinderFlag.at(i) == -1)
		//		cylinderInnerFaces.push_back(f);
		//	if (cylinderFlag.at(i) == 1 && (tri.size() == 0))
		//		cylinderOuterFaces.push_back(f);
		//}
	}

	bool OptimizeCylinderCollide::valid()
	{
		return focusTriangle > 0;
	}

	trimesh::TriMesh* OptimizeCylinderCollide::drill()
	{
		TriPatch newMeshTriangles;
		generateNewTriangles(meshFocusFaces, m_mesh, focusNormals, meshTris, newMeshTriangles, true, nullptr);
		TriPatch newCylinderTriangles;
		generateNewTriangles(m_cylinder->faces, m_cylinder, cylinderNormals, cylinderTris, newCylinderTriangles, false, m_debugger);
		trimesh::TriMesh* Mout = generateAppendMesh(m_mesh, totalMeshFlag, newMeshTriangles, CylinderCollideOuter);
		trimesh::TriMesh* Cyin = generateAppendMesh(m_cylinder, cylinderFlag, newCylinderTriangles, CylinderCollideInner);
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