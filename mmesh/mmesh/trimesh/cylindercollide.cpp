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

						TriTri mt;
						mt.topIndex = temps[0].index;
						mt.d[0] = mesh2Cylinder1;
						mt.d[1] = mesh2Cylinder2;
						mt.d[2] = mesh2Cylinder3;

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
								}
								else
								{
									v1 = points.at(2);
									v2 = points.at(1);
									mtIndex[1] = temps[0].e2;
								}
							}

							mt.v1 = v1;
							mt.v2 = v2;

							mt.index = mtIndex;
							meshTri.push_back(mt);
							fc.flag = 0;
						}
					}
				}
			}
		}
	}

	void generatePolygon(TriMesh::Face& face, trimesh::TriMesh* mesh, std::vector<TriTri>& tri, bool positive,
		std::vector<std::vector<int>>& polygons, std::vector<trimesh::vec3>& d3points)
	{
		UniformPoints uniformPoints;

		struct segment
		{
			int start;
			int end;

			int startEdge;
			int endEdge;
		};

		int triSize = tri.size();
		std::vector<segment> segments(triSize);

		for (int j = 0; j < tri.size(); ++j)
		{
			TriTri& tt = tri.at(j);
			segment& seg = segments.at(j);

			if (positive)
			{
				if (tt.d[tt.topIndex] >= 0.0f)
				{
					seg.start = uniformPoints.add(tt.v1);
					seg.end = uniformPoints.add(tt.v2);

					seg.startEdge = tt.index[0];
					seg.endEdge = tt.index[1];
				}
				else
				{
					seg.start = uniformPoints.add(tt.v2);
					seg.end = uniformPoints.add(tt.v1);

					seg.startEdge = tt.index[1];
					seg.endEdge = tt.index[0];
				}
			}
			else
			{
				if (tt.d[tt.topIndex] >= 0.0f)
				{
					seg.start = uniformPoints.add(tt.v2);
					seg.end = uniformPoints.add(tt.v1);

					seg.startEdge = tt.index[1];
					seg.endEdge = tt.index[0];
				}
				else
				{
					seg.start = uniformPoints.add(tt.v1);
					seg.end = uniformPoints.add(tt.v2);

					seg.startEdge = tt.index[0];
					seg.endEdge = tt.index[1];
				}
			}
		}

		int unionSize = (int)uniformPoints.uniformSize();
		d3points.reserve(unionSize + 3);
		uniformPoints.toVector(d3points);

		for (int i = 0; i < 3; ++i)
		{
			d3points.push_back(mesh->vertices.at(face[i]));
		}

		std::vector<int> types(unionSize + 3, -2); // -2 outer, -1 inner, >= 0 edge
		std::vector<bool> edgeIsFirst(unionSize, true);
		for (int i = 0; i < triSize; ++i)
		{
			segment& s = segments.at(i);

			if (s.startEdge >= 0)
			{
				assert(types.at(s.start) != -1);
				if (types.at(s.start) == -2)
				{
					types.at(s.start) = s.startEdge;
				}
			}
			else
			{
				assert(types.at(s.start) < 0);
				if (types.at(s.start) == -2)
				{
					types.at(s.start) = -1;
				}
			}

			if (s.endEdge >= 0)
			{
				assert(types.at(s.end) != -1);
				if (types.at(s.end) == -2)
				{
					types.at(s.end) = s.endEdge;
					edgeIsFirst.at(s.end) = false;
				}
			}
			else
			{
				assert(types.at(s.end) < 0);
				if (types.at(s.end) == -2)
				{
					types.at(s.end) = -1;
				}
			}
		}

		struct IndexPolygon
		{
			std::list<int> polygon;
			int start;
			int end;

			bool closed()
			{
				return (polygon.size() >= 2) && (polygon.front() == polygon.back());
			}
		};

		std::vector<IndexPolygon> indexPolygons;
		for (int i = 0; i < triSize; ++i)
		{
			IndexPolygon ipolygon;
			segment& seg = segments.at(i);
			ipolygon.start = seg.start;
			ipolygon.end = seg.end;
			ipolygon.polygon.push_back(ipolygon.start);
			ipolygon.polygon.push_back(ipolygon.end);

			indexPolygons.emplace_back(ipolygon);
		}

		size_t indexPolygonSize = indexPolygons.size();
		std::map<int, IndexPolygon*> IndexPolygonMap;
		for (size_t i = 0; i < indexPolygonSize; ++i)
		{
			IndexPolygon& p1 = indexPolygons.at(i);
			if (!p1.closed())
				IndexPolygonMap.insert(std::pair<int, IndexPolygon*>(p1.start, &p1));
		}

		//combime
		for (size_t i = 0; i < indexPolygonSize; ++i)
		{
			IndexPolygon& p1 = indexPolygons.at(i);

			if (p1.polygon.size() == 0 || p1.closed())
				continue;

			auto it = IndexPolygonMap.find(p1.end);
			while (it != IndexPolygonMap.end())
			{

				IndexPolygon& p2 = *(*it).second;
				if (p2.polygon.size() == 0)
					break;

				bool merged = false;
				if (p1.start == p2.end)
				{
					p1.start = p2.start;
					for (auto iter = p2.polygon.rbegin(); iter != p2.polygon.rend(); ++iter)
					{
						if ((*iter) != p1.polygon.front()) p1.polygon.push_front((*iter));
					}
					merged = true;
				}
				else if (p1.end == p2.start)
				{
					p1.end = p2.end;
					for (auto iter = p2.polygon.begin(); iter != p2.polygon.end(); ++iter)
					{
						if ((*iter) != p1.polygon.back()) p1.polygon.push_back((*iter));
					}
					merged = true;
				}

				if (merged)
				{
					p2.polygon.clear();
				}
				else
					break;

				it = IndexPolygonMap.find(p1.end);
			}
		}

		std::vector<IndexPolygon> validIndexPolygons;
		for (size_t i = 0; i < indexPolygons.size(); ++i)
		{
			if (indexPolygons.at(i).polygon.size() > 0)
			{
				validIndexPolygons.push_back(indexPolygons.at(i));
			}
		}

		int polygonSize = (int)validIndexPolygons.size();
		std::vector<int> edgePolygon;
		std::vector<int> innerPolygon;
		for (int i = 0; i < polygonSize; ++i)
		{
			IndexPolygon& ip = validIndexPolygons.at(i);
			if (types.at(ip.start) >= 0 && types.at(ip.end) >= 0)
			{
				edgePolygon.push_back(i);
			}
			else
			{
				innerPolygon.push_back(i);
			}
		}

		int edgePolygonSize = edgePolygon.size();
		int innerPolygonSize = innerPolygon.size();
		for (int i = 0; i < innerPolygonSize; ++i)
		{
			int index = innerPolygon.at(i);
			IndexPolygon& ipolygon = validIndexPolygons.at(index);
			if (ipolygon.closed())
			{
				std::vector<int> polygon;
				polygon.insert(polygon.end(), ipolygon.polygon.begin(), ipolygon.polygon.end());
				polygons.push_back(polygon);
			}
		}

		if (edgePolygonSize == 0 && positive)
		{
			std::vector<int> polygon;
			for (int i = 0; i < 3; ++i)
			{
				polygon.push_back(unionSize + i);
			}
			polygon.push_back(unionSize);

			polygons.push_back(polygon);
		}
		else
		{
			std::vector<std::vector<int>> edgesPoints(3);
			for (int i = 0; i < unionSize; ++i)
			{
				if (types.at(i) >= 0)
					edgesPoints.at(types.at(i)).push_back(i);
			}

			std::vector<IndexPolygon> indexedgePolygons;
			for (int i = 0; i < edgePolygonSize; ++i)
			{
				indexedgePolygons.push_back(validIndexPolygons.at(edgePolygon.at(i)));
			}

			//add triangle edge
			for (int i = 0; i < 3; ++i)
			{
				std::vector<int> edgePoints = edgesPoints.at(i);
				int startIndex = unionSize + i;
				int endIndex = unionSize + (i + 1) % 3;

				std::sort(edgePoints.begin(), edgePoints.end(), [&startIndex, &d3points](int i1, int i2)->bool {
					vec3 v1 = d3points.at(i1);
					vec3 v2 = d3points.at(i2);
					vec3 o = d3points.at(startIndex);
					return len(v1 - o) < len(v2 - o);
					});

				struct PP
				{
					int start;
					int end;
				};

				std::vector<PP> pairs;
				if (edgePoints.size() == 0)
				{
					PP pp;
					pp.start = startIndex;
					pp.end = endIndex;
					pairs.push_back(pp);
				}
				else
				{
					if (edgeIsFirst.at(edgePoints.at(0)))
					{
						PP pp;
						pp.start = startIndex;
						pp.end = edgePoints.at(0);
						pairs.push_back(pp);
					}
					if (!edgeIsFirst.at(edgePoints.back()))
					{
						PP pp;
						pp.start = edgePoints.back();
						pp.end = endIndex;
						pairs.push_back(pp);
					}

					if (positive)
					{
						for (int ii = 1; ii < (int)edgePoints.size(); ii += 2)
						{
							if (ii + 1 < edgePoints.size())
							{
								int s = edgePoints.at(ii);
								int e = edgePoints.at(ii + 1);
								if (edgeIsFirst.at(s) && !edgeIsFirst.at(e))
								{
									PP pp;
									pp.start = s;
									pp.end = e;
									pairs.push_back(pp);
								}
							}
						}
					}
					else
					{
						for (int ii = 0; ii < (int)edgePoints.size() - 1; ii += 2)
						{
							int s = edgePoints.at(ii);
							int e = edgePoints.at(ii + 1);
							if (!edgeIsFirst.at(s) && edgeIsFirst.at(e))
							{
								PP pp;
								pp.start = s;
								pp.end = e;
								pairs.push_back(pp);
							}
						}
					}
				}


				for (PP& pp : pairs)
				{
					IndexPolygon ip;
					ip.start = pp.start;
					ip.end = pp.end;
					ip.polygon.push_back(ip.start);
					ip.polygon.push_back(ip.end);
					indexedgePolygons.push_back(ip);
				}
			}

			size_t indexedgePolygonSize = indexedgePolygons.size();
			std::map<int, IndexPolygon*> IndexEdgePolygonMap;
			for (size_t i = 0; i < indexedgePolygonSize; ++i)
			{
				IndexPolygon& p1 = indexedgePolygons.at(i);
				if (!p1.closed())
					IndexEdgePolygonMap.insert(std::pair<int, IndexPolygon*>(p1.start, &p1));
			}

			//combime
			for (size_t i = 0; i < indexedgePolygonSize; ++i)
			{
				IndexPolygon& p1 = indexedgePolygons.at(i);

				if (p1.polygon.size() == 0 || p1.closed())
					continue;

				auto it = IndexEdgePolygonMap.find(p1.end);
				while (it != IndexEdgePolygonMap.end())
				{

					IndexPolygon& p2 = *(*it).second;
					if (p2.polygon.size() == 0)
						break;

					bool merged = false;
					if (p1.start == p2.end)
					{
						p1.start = p2.start;
						for (auto iter = p2.polygon.rbegin(); iter != p2.polygon.rend(); ++iter)
						{
							if ((*iter) != p1.polygon.front()) p1.polygon.push_front((*iter));
						}
						merged = true;
					}
					else if (p1.end == p2.start)
					{
						p1.end = p2.end;
						for (auto iter = p2.polygon.begin(); iter != p2.polygon.end(); ++iter)
						{
							if ((*iter) != p1.polygon.back()) p1.polygon.push_back((*iter));
						}
						merged = true;
					}

					if (merged)
					{
						p2.polygon.clear();
					}
					else
						break;

					it = IndexEdgePolygonMap.find(p1.end);
				}
			}

			for (int i = 0; i < (int)indexedgePolygons.size(); ++i)
			{
				if (indexedgePolygons.at(i).polygon.size() > 0)
				{
					std::list<int>& polygon = indexedgePolygons.at(i).polygon;

					std::vector<int> inpolygon;
					inpolygon.insert(inpolygon.end(), polygon.begin(), polygon.end());
					polygons.push_back(inpolygon);
				}
			}
		}
	}

	void generateNewTriangles(std::vector<trimesh::TriMesh::Face>& focusFaces, trimesh::TriMesh* mesh, std::vector<trimesh::vec3>& normals,
		std::vector<FaceCollide>& tris, std::vector<trimesh::vec3>& newTriangles, bool positive)
	{
		int meshNum = (int)focusFaces.size();
		for (int i = 0; i < meshNum; ++i)
		{
			FaceCollide& fcollide = tris.at(i);
			std::vector<TriTri>& tri = fcollide.tris;
			if (tri.size() > 0)
			{
				std::vector<std::vector<int>> polygons;
				std::vector<vec3> points;
				generatePolygon(focusFaces.at(i), mesh, tri, positive, polygons, points);

				generateTriangleSoup(points, normals.at(i), polygons, newTriangles);
			}
		}
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

	trimesh::TriMesh* generateAppendMesh(trimesh::TriMesh* oldMesh, FlagPatch& flagFaces, TriPatch& newTriangles)
	{
		if (!oldMesh)
			return nullptr;

		trimesh::TriMesh* newMesh = generatePatchMesh(oldMesh, flagFaces, CylinderCollideOuter);
		
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

		for (int i = 0; i < focusTriangle; ++i)
		{
			int index = meshFocusFacesMapper.at(i);
			totalMeshFlag.at(index) = meshTris.at(i).flag;
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
		generateNewTriangles(meshFocusFaces, m_mesh, focusNormals, meshTris, newMeshTriangles, true);

		trimesh::TriMesh* Mout = generateAppendMesh(m_mesh, totalMeshFlag, newMeshTriangles);
		trimesh::TriMesh* Cyin = nullptr; // generateNewMesh(m_cylinder, cylinderInnerFaces, newCylinderTriangles);
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