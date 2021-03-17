#include "mmesh/trimesh/polygonstack.h"
#include <stack>
#include <functional>

#include "mmesh/trimesh/polygon.h"
#include "mmesh/trimesh/polygon2util.h"
#include "mmesh/trimesh/savepolygonstack.h"

namespace mmesh
{
	PolygonStack::PolygonStack()
		: m_currentPolygon(0)
	{
	}
	
	PolygonStack::~PolygonStack()
	{
		clear();
	}

	void PolygonStack::clear()
	{
		for (Polygon2* poly : m_polygon2s)
			delete poly;
		m_polygon2s.clear();
		m_currentPolygon = 0;
	}

	void PolygonStack::generates(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points, std::vector<trimesh::TriMesh::Face>& triangles)
	{
#if 0
		static int i = 0;
		char buffer[128];
		sprintf(buffer, "%d.poly", i++);
		stackSave(buffer, polygons, points);
#endif
		prepare(polygons, points);
		generate(triangles);
	}

	void PolygonStack::generatesWithoutTree(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points, std::vector<trimesh::TriMesh::Face>& triangles)
	{
		prepareWithoutTree(polygons, points);
		generate(triangles);
	}

	void PolygonStack::prepareWithoutTree(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points)
	{
		size_t size = polygons.size();
		if (size > 0)
		{
			for (size_t i = 0; i < size; ++i)
			{
				std::vector<int>& polygon = polygons.at(i);
				if (polygon.size() > 0 && (polygon.at(0) != polygon.back()))
				{// loop
					polygon.push_back(polygon.at(0));
				}

				if (polygon.size() > 0)
				{
					Polygon2* poly = new Polygon2();
					poly->setup(polygon, points);
					m_polygon2s.push_back(poly);
				}
			}
		}
	}

	void PolygonStack::prepare(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points)
	{
		size_t size = polygons.size();
		if (size > 0)
		{
			// build simple polygon
			struct SimpleInfo
			{
				double area;
				trimesh::dbox2 box;
			};
			std::list<int> candidates;
			std::vector<SimpleInfo> infos(size);
			for (size_t i = 0; i < size; ++i)
			{
				std::vector<int>& polygon = polygons.at(i);
				if (polygon.size() > 0 && (polygon.at(0) != polygon.back()))
				{// loop
					polygon.push_back(polygon.at(0));
				}

				size_t polygonSize = polygon.size();
				SimpleInfo& info = infos.at(i);
				candidates.push_back((int)i);
				info.area = 0.0;
				if (polygonSize > 1)
				{
					info.box += points.at(polygon.at(0));
					for (size_t j = 1; j < polygonSize; ++j)
					{
						trimesh::dvec2& v1 = points.at(polygon.at(j));
						trimesh::dvec2& v2 = points.at(polygon.at(j - 1));

						info.area += 0.5 * crossValue(v2, v1);
						info.box += v1;
					}
				}
			}

			candidates.sort([&infos](int i1, int i2)->bool {
					return abs(infos.at(i1).area) > abs(infos.at(i2).area);
					});

			struct TreeNode
			{
				int index;
				bool outer;
				std::list<int> temp;
				std::vector<TreeNode> children;
			};
			TreeNode rootNode;
			rootNode.outer = false;
			rootNode.index = -1;
			rootNode.temp.swap(candidates);
			std::vector<std::vector<int>> simplePolygons(size);

			std::function<void(TreeNode& node)> split;
			std::function<void(TreeNode& node)> merge;
			std::function<bool(int c, int t)> testin;
			testin = [&infos, &polygons, &points](int c, int t) ->bool{
				trimesh::dbox2 cb = infos.at(c).box;
				trimesh::dbox2 tb = infos.at(t).box;
				if(!cb.contains(tb))
					return false;

				int tindex = polygons.at(t).at(0);
				trimesh::dvec2& tvertex = points.at(tindex);
				std::vector<int>& cpolygon = polygons.at(c);
				int nvert = (int)cpolygon.size();
				int i, j, count = 0;
				for (i = 0, j = nvert - 1; i < nvert; j = i++)
				{
					trimesh::dvec2& verti = points.at(cpolygon.at(i));
					trimesh::dvec2& vertj = points.at(cpolygon.at(j));
					if (((verti.y > tvertex.y) != (vertj.y > tvertex.y)) &&
						(tvertex.x < (vertj.x - verti.x) * (tvertex.y - verti.y) / (vertj.y - verti.y) + verti.x))
						count = !count;
				}
				return count != 0;
			};
			merge = [&merge, &polygons, &simplePolygons, &infos, &points](TreeNode& node) {
				int index = node.index;
				if (index >= 0 && infos.at(index).area > 0.0)
				{
					simplePolygons.at(index) = polygons.at(index);
					std::vector<int>& outerPolygon = simplePolygons.at(index);
					if(node.children.size() > 0)
					{
						std::vector<int> indices;
						for (TreeNode& n : node.children)
							indices.push_back(n.index);

						std::sort(indices.begin(), indices.end(), [&infos](int i, int j)->bool {
							return infos.at(i).box.max.x < infos.at(j).box.max.x;
							});

						const double EPSON = 0.00000001;

#if 0
						size_t innserPolygonSize = indices.size();
						while (indices.size() > innserPolygonSize - 12)
#else
						while (indices.size() > 0)
#endif
						{
							int polygonIndex = indices.back();
							indices.pop_back();

							//// merge polygonIndex and index
							std::vector<int>& innerPolygon = polygons.at(polygonIndex);
							int innerSize = (int)innerPolygon.size();
							float mx = -10000000.0f;
							int vertexIndex = -1;
							for (int i = 0; i < innerSize; ++i)
							{
								trimesh::dvec2& v = points.at(innerPolygon.at(i));
								if (v.x > mx)
								{
									mx = v.x;
									vertexIndex = i;
								}
							}

							if (vertexIndex >= 0)
							{
								trimesh::dvec2& tvertex = points.at(innerPolygon.at(vertexIndex));
								int nvert = (int)outerPolygon.size();
								int i, j = 0;
								double cmx = 1000000.0;
								int cOuterIndex = -1;
								int cOuterIndex0 = -1;
								for (i = 0, j = nvert - 1; i < nvert; j = i++)
								{
									trimesh::dvec2& verti = points.at(outerPolygon.at(i));
									trimesh::dvec2& vertj = points.at(outerPolygon.at(j));
									if (verti.y == tvertex.y && vertj.y == tvertex.y)
									{
										double mmx = verti.x > vertj.x ? vertj.x : verti.x;
										if (mmx > tvertex.x && mmx < cmx)
										{
											cOuterIndex = i;
											cOuterIndex0 = j;
											cmx = mmx;
										}
									}
									else if ((verti.y > tvertex.y) != (vertj.y > tvertex.y))
									{
										double cx = (vertj.x - verti.x)* (tvertex.y - verti.y) / (vertj.y - verti.y) + verti.x;
										if (cx > tvertex.x)  // must 
										{
											if (std::abs(cx - cmx) < EPSON)
											{  // collide two opposite edge
												trimesh::dvec2 xxn(1.0, 0.0);
												trimesh::dvec2 nji = verti - vertj;
												if (crossValue(xxn, nji) >= 0.0)
												{
													cOuterIndex = i;
													cOuterIndex0 = j;
													cmx = cx;
												}
											}else if(cx < cmx)
											{
												cOuterIndex = i;
												cOuterIndex0 = j;
												cmx = cx;
											}
										}
									}
								}

								int mutaulIndex = -1;
								if (cOuterIndex >= 0)
								{
									if (cmx == points.at(outerPolygon.at(cOuterIndex)).x
										&& tvertex.y == points.at(outerPolygon.at(cOuterIndex)).y)
										mutaulIndex = cOuterIndex;
									else if (cmx == points.at(outerPolygon.at(cOuterIndex0)).x
										&& tvertex.y == points.at(outerPolygon.at(cOuterIndex0)).y)
									{
										mutaulIndex = cOuterIndex0;
									}
									else
									{
										trimesh::dvec2 M = tvertex;
										if (points.at(outerPolygon.at(cOuterIndex)).x < points.at(outerPolygon.at(cOuterIndex0)).x)
										{
											cOuterIndex = cOuterIndex0;
										}
										trimesh::dvec2 P = points.at(outerPolygon.at(cOuterIndex));
										trimesh::dvec2 I = trimesh::dvec2(cmx, M.y);
										if (P.y > I.y)
										{
											trimesh::dvec2 T = P;
											P = I;
											I = T;
										}

										std::vector<int> reflexVertex;
										for (i = 0; i < nvert; ++i)
										{
											trimesh::dvec2& tv = points.at(outerPolygon.at(i));
											if ((i != cOuterIndex) && (outerPolygon.at(i) != outerPolygon.at(cOuterIndex)) && insideTriangle(M, P, I, tv))
											{
												reflexVertex.push_back(i);
											}
										}

										if (reflexVertex.size() == 0)
										{
											mutaulIndex = cOuterIndex;
										}
										else
										{
											int reflexSize = reflexVertex.size();
											double minLen = 1000000.0;
											double maxDot = -10000.0;
											int minReflexIndex = 0;
											for (i = 0; i < reflexSize; ++i)
											{
												trimesh::dvec2 R = points.at(outerPolygon.at(reflexVertex.at(i)));
												trimesh::dvec2 MR = R - M;
												double len = trimesh::len(MR);
												trimesh::normalize(MR);
												double dot = abs(dotValue(MR, trimesh::dvec2(1.0, 0.0)));
												if (dot > maxDot && len < minLen)
												{
													minReflexIndex = i;
													minLen = len;
													maxDot = dot;
												}
											}

											mutaulIndex = reflexVertex.at(minReflexIndex);
										}
									}
								}

								if (mutaulIndex >= 0)
								{// merge mutaulIndex in outer and vertexIndex in inner
									std::vector<int> mergedPolygon;
									for (i = 0; i < nvert; ++i) 
									{
										mergedPolygon.push_back(outerPolygon.at(i));
										if (i == mutaulIndex)
										{// insert inner
											for(j = vertexIndex; j < innerSize - 1; ++j)
												mergedPolygon.push_back(innerPolygon.at(j));
											for(j = 0; j <= vertexIndex; ++j)
												mergedPolygon.push_back(innerPolygon.at(j));
											mergedPolygon.push_back(outerPolygon.at(i));
										}
									}
									outerPolygon.swap(mergedPolygon);
								}
							}
						}
					}
				}

				for (TreeNode& cNode : node.children)
				{
					merge(cNode);
				}
			};
			split = [&split, &testin, &infos](TreeNode& node) {
				std::list<int>& candidates = node.temp;
				while (candidates.size() > 0)
				{
					int index = candidates.front();
					candidates.pop_front();

					if ((!node.outer && infos.at(index).area > 0.0) || (node.outer && infos.at(index).area < 0.0))
					{
						TreeNode rootNode;
						rootNode.outer = !node.outer;
						rootNode.index = index;

						std::vector<int> added;
						for (std::list<int>::iterator it = candidates.begin(); it != candidates.end();)
						{
							std::list<int>::iterator c = it;
							++it;
							if (testin(index, *c))
							{
								bool inAdded = false;
								for (int addedIndex : added)
								{
									if (testin(addedIndex, *c))
									{
										inAdded = true;
										break;
									}
								}
								if (!inAdded)  //test *c collide in index
								{
									added.push_back(*c);
								}

								rootNode.temp.push_back(*c);

								candidates.erase(c);
							}
						}
						node.children.push_back(rootNode);
					}
				}

				for (TreeNode& cNode : node.children)
				{
					split(cNode);
				}
			};

			split(rootNode);
			merge(rootNode);

			for (size_t i = 0; i < size; ++i)
			{
				if (simplePolygons.at(i).size() > 0)
				{
					Polygon2* poly = new Polygon2();
					poly->setup(simplePolygons.at(i), points);
					m_polygon2s.push_back(poly);
				}
			}
		}
	}

	void PolygonStack::generate(std::vector<trimesh::TriMesh::Face>& triangles)
	{
		for (Polygon2* poly : m_polygon2s)
		{
			poly->earClipping(triangles);
		}
	}

	bool PolygonStack::earClipping(trimesh::TriMesh::Face& face, std::vector<int>* earIndices)
	{
		if (m_currentPolygon >= (int)m_polygon2s.size() || m_currentPolygon < 0)
			return false;

		Polygon2* poly = m_polygon2s.at(m_currentPolygon);
		if (!poly->earClipping(face, earIndices))
		{
			++m_currentPolygon;
		}

		return true;
	}

	void PolygonStack::getEars(std::vector<int>* earIndices)
	{
		Polygon2* poly = nullptr;
		if (m_currentPolygon >= 0 && m_currentPolygon < (int)m_polygon2s.size())
		{
			poly = m_polygon2s.at(m_currentPolygon);
		}
		if (poly) poly->getEars(earIndices);
	}

	Polygon2* PolygonStack::validIndexPolygon(int index)
	{
		if (index >= 0 && index < (int)m_polygon2s.size())
		{
			return m_polygon2s.at(index);
		}
		return nullptr;
	}

	int PolygonStack::validPolygon()
	{
		return (int)m_polygon2s.size();
	}
}