#include "trianglesplit.h"
#include "mmesh/trimesh/polygonstack.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "trimesh2/Vec3Utils.h"
#include "mmesh/common/print.h"
#include "mmesh/trimesh/uniformpoints.h"

#include <assert.h>
namespace mmesh
{
	int PointInPolygon(trimesh::dvec2 pt,const std::vector<int>& polygon, std::vector<trimesh::dvec2>& d2points)
	{
		//returns 0 if false, +1 if true, -1 if pt ON polygon boundary
		int result = 0;
		size_t cnt = polygon.size();
		if (cnt < 3) return 0;
		trimesh::dvec2 ip = d2points[polygon[0]];
		for (size_t i = 1; i <= cnt; ++i)
		{
			trimesh::dvec2 ipNext = (i == cnt ? d2points[polygon[0]] : d2points[polygon[i]]);
			if (ipNext.y == pt.y)
			{
				if ((ipNext.x == pt.x) || (ip.y == pt.y &&
					((ipNext.x > pt.x) == (ip.x < pt.x)))) return -1;
			}
			if ((ip.y < pt.y) != (ipNext.y < pt.y))
			{
				if (ip.x >= pt.x)
				{
					if (ipNext.x > pt.x) result = 1 - result;
					else
					{
						double d = (double)(ip.x - pt.x) * (ipNext.y - pt.y) -
							(double)(ipNext.x - pt.x) * (ip.y - pt.y);
						if (!d) return -1;
						if ((d > 0) == (ipNext.y > ip.y)) result = 1 - result;
					}
				}
				else
				{
					if (ipNext.x > pt.x)
					{
						double d = (double)(ip.x - pt.x) * (ipNext.y - pt.y) -
							(double)(ipNext.x - pt.x) * (ip.y - pt.y);
						if (!d) return -1;
						if ((d > 0) == (ipNext.y > ip.y)) result = 1 - result;
					}
				}
			}
			ip = ipNext;
		}
		return result;
	}

	bool splitTriangle(const trimesh::vec3& v0, const trimesh::vec3& v1, const trimesh::vec3& v2,
		const std::vector<TriSegment>& tri, bool positive, std::vector<trimesh::vec3>& newTriangles,std::vector<bool>& isInner)
	{
		trimesh::vec3 normal = (v1 - v0) TRICROSS (v2 - v0);
		trimesh::normalize(normal);

		UniformPoints uniformPoints;

		int triSize = tri.size();
		std::vector<IndexPolygon> validIndexPolygons;
		std::vector<IndexSegment> segments(triSize);
		std::vector<int> vertexEdges;
		std::vector<bool> edgeIsFirst;

		auto vbe = [&vertexEdges](int pindex, int eindex) {
			assert(pindex <= vertexEdges.size());
			if (pindex == vertexEdges.size())
				vertexEdges.push_back(eindex);
			//assert(vertexEdges[pindex] == eindex);
		};
		auto fbe = [&edgeIsFirst](int pindex, bool first) {
			assert(pindex <= edgeIsFirst.size());
			if (pindex == edgeIsFirst.size())
				edgeIsFirst.push_back(first);
			edgeIsFirst[pindex] = first;
		};
		for (int j = 0; j < tri.size(); ++j)
		{
			const TriSegment& tt = tri.at(j);

			int start = uniformPoints.add(tt.v1);
			vbe(start, tt.index[0]);
			int end = uniformPoints.add(tt.v2);
			vbe(end, tt.index[1]);

			IndexPolygon ipolygon;
			ipolygon.start = start;
			ipolygon.end = end;

			if (tt.topPositive == false)
			{
				std::swap(ipolygon.start, ipolygon.end);
				fbe(ipolygon.end, false);
				fbe(ipolygon.start, true);
			}
			else
			{
				fbe(ipolygon.start, true);
				fbe(ipolygon.end, false);
			}
			ipolygon.polygon.push_back(ipolygon.start);
			ipolygon.polygon.push_back(ipolygon.end);
			validIndexPolygons.emplace_back(ipolygon);
		}

		mergeIndexPolygon(validIndexPolygons);
		int polygonSize = (int)validIndexPolygons.size();
		int unionSize = (int)uniformPoints.uniformSize();
		assert(unionSize == vertexEdges.size());

		std::vector<std::vector<int>> polygons;
		std::vector<trimesh::vec3> d3points;
		std::vector<trimesh::dvec2> d2points;
		std::vector<trimesh::dvec2> d2pointsVertices;
		uniformPoints.toVector(d3points);
		d3points.push_back(v0);
		d3points.push_back(v1);
		d3points.push_back(v2);
		transform3to2(d3points, normal, d2points);

		std::vector<trimesh::vec3> vpoints;
		vpoints.push_back(v0);
		vpoints.push_back(v1);
		vpoints.push_back(v2);
		transform3to2(vpoints, normal, d2pointsVertices);

		std::vector<int> orderEdgesPoints;

		for (int e = 0; e < 3; ++e)
		{
			std::vector<int> edgePoints;
			for (int i = 0; i < unionSize; ++i)
			{
				if (vertexEdges.at(i) == e)
					edgePoints.push_back(i);
			}

			std::sort(edgePoints.begin(), edgePoints.end(), [&unionSize, &e, &d3points](int i1, int i2)->bool {
				trimesh::vec3 v1 = d3points.at(i1);
				trimesh::vec3 v2 = d3points.at(i2);
				trimesh::vec3 o = d3points.at(unionSize + e);
				return trimesh::len(v1 - o) < trimesh::len(v2 - o);
				});

			orderEdgesPoints.push_back(unionSize + e);
			orderEdgesPoints.insert(orderEdgesPoints.end(), edgePoints.begin(), edgePoints.end());
		}

		std::vector<int> edgePolygon;
		std::vector<int> innerPolygon;
		for (int i = 0; i < polygonSize; ++i)
		{
			IndexPolygon& ip = validIndexPolygons.at(i);
			if ((vertexEdges.at(ip.start) >= 0) && (vertexEdges.at(ip.end) >= 0))
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

		if (positive)
		{
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
				else
				{
					//assert(false);
				}
			}

			if (edgePolygonSize == 0)
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
					if (vertexEdges.at(i) >= 0)
						edgesPoints.at(vertexEdges.at(i)).push_back(i);
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
						trimesh::vec3 v1 = d3points.at(i1);
						trimesh::vec3 v2 = d3points.at(i2);
						trimesh::vec3 o = d3points.at(startIndex);
						return trimesh::len(v1 - o) < trimesh::len(v2 - o);
						});

					std::vector<IndexSegment> pairs;
					if (edgePoints.size() == 0)
					{
						IndexSegment pp;
						pp.start = startIndex;
						pp.end = endIndex;
						pairs.push_back(pp);
					}
					else
					{
						if (edgeIsFirst.at(edgePoints.at(0)))
						{
							IndexSegment pp;
							pp.start = startIndex;
							pp.end = edgePoints.at(0);
							pairs.push_back(pp);
						}
						if (!edgeIsFirst.at(edgePoints.back()))
						{
							IndexSegment pp;
							pp.start = edgePoints.back();
							pp.end = endIndex;
							pairs.push_back(pp);
						}

						for (int ii = 0; ii < (int)edgePoints.size(); ii += 1)
						{
							int nextIndex = ii + 1;
							if (nextIndex < edgePoints.size())
							{
								int s = edgePoints.at(ii);
								int e = edgePoints.at(nextIndex);
								if (!edgeIsFirst.at(s) && edgeIsFirst.at(e))
								{
									IndexSegment pp;
									pp.start = s;
									pp.end = e;
									pairs.push_back(pp);
								}
							}
						}
					}


					for (IndexSegment& pp : pairs)
					{
						IndexPolygon ip;
						ip.start = pp.start;
						ip.end = pp.end;
						ip.polygon.push_back(ip.start);
						ip.polygon.push_back(ip.end);
						indexedgePolygons.push_back(ip);
					}
				}

				mergeIndexPolygon(indexedgePolygons);
				int validEdgeSize = (int)indexedgePolygons.size();
				for (int i = 0; i < validEdgeSize; ++i)
				{
					std::list<int>& polygon = indexedgePolygons.at(i).polygon;
					if((polygon.size() <= 2) ||(polygon.front() != polygon.back()))
						continue;
					std::vector<int> inpolygon;
					inpolygon.insert(inpolygon.end(), polygon.begin(), polygon.end());
					polygons.push_back(inpolygon);
				}
			}
		}
		else
		{
			for (int i = 0; i < innerPolygonSize; ++i)
			{
				int index = innerPolygon.at(i);
				IndexPolygon& ipolygon = validIndexPolygons.at(index);
				if (ipolygon.closed())
				{
					std::vector<int> polygon;
					polygon.insert(polygon.end(), ipolygon.polygon.rend(), ipolygon.polygon.rbegin());
					polygons.push_back(polygon);
				}
				else
				{
					//assert(false);
				}
			}

			{
				std::vector<std::vector<int>> edgesPoints(3);
				for (int i = 0; i < unionSize; ++i)
				{
					if (vertexEdges.at(i) >= 0)
						edgesPoints.at(vertexEdges.at(i)).push_back(i);
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
						trimesh::vec3 v1 = d3points.at(i1);
						trimesh::vec3 v2 = d3points.at(i2);
						trimesh::vec3 o = d3points.at(startIndex);
						return trimesh::len(v1 - o) < trimesh::len(v2 - o);
						});

					std::vector<IndexSegment> pairs;
					if (edgePoints.size() == 0)
					{
						IndexSegment pp;
						pp.start = startIndex;
						pp.end = endIndex;
						pairs.push_back(pp);
					}
					else
					{
						if (!edgeIsFirst.at(edgePoints.at(0)))
						{
							IndexSegment pp;
							pp.start = startIndex;
							pp.end = edgePoints.at(0);
							pairs.push_back(pp);
						}
						if (edgeIsFirst.at(edgePoints.back()))
						{
							IndexSegment pp;
							pp.start = edgePoints.back();
							pp.end = endIndex;
							pairs.push_back(pp);
						}

						for (int ii = 0; ii < (int)edgePoints.size(); ii += 1)
						{
							int nextIndex = ii + 1;
							if (nextIndex < edgePoints.size())
							{
								int s = edgePoints.at(ii);
								int e = edgePoints.at(nextIndex);
								if (edgeIsFirst.at(s) && !edgeIsFirst.at(e))
								{
									IndexSegment pp;
									pp.start = s;
									pp.end = e;
									pairs.push_back(pp);
								}
							}
						}
					}

					for (IndexSegment& pp : pairs)
					{
						IndexPolygon ip;
						ip.start = pp.end;
						ip.end = pp.start;
						ip.polygon.push_back(ip.start);
						ip.polygon.push_back(ip.end);
						indexedgePolygons.push_back(ip);
					}
				}

				mergeIndexPolygon(indexedgePolygons);
				int validEdgeSize = (int)indexedgePolygons.size();
				for (int i = 0; i < validEdgeSize; ++i)
				{
					std::list<int>& polygon = indexedgePolygons.at(i).polygon;
					if ((polygon.size() <= 2) || (polygon.front() != polygon.back()))
						continue;
					std::vector<int> inpolygon;
					inpolygon.insert(inpolygon.end(), polygon.rbegin(), polygon.rend());
					polygons.push_back(inpolygon);
				}
			}

			for (size_t i = 0; i < d2pointsVertices.size(); i++)
			{
				for (size_t j = 0; j < polygons.size(); j++)
				{
					int re = PointInPolygon(d2pointsVertices[i], polygons[j], d2points);
					if (re != 0)
					{
						isInner[i] = false;
					}
				}
			}
		}

		generateTriangleSoup(d3points, d2points, polygons, newTriangles);
		return true;
	}
}