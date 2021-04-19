#include "zsplit.h"
#include <map>
#include <list>
#include "mmesh/trimesh/polygonstack.h"

namespace mmesh
{
	ZSplit::ZSplit()
	{

	}

	ZSplit::~ZSplit()
	{

	}

	void lines2polygon(std::vector<trimesh::vec3>& lines, std::vector<std::vector<int>>& polygons, std::vector<trimesh::vec3>& uniPoints)
	{
		size_t size = lines.size();
		size_t segsize = size / 2;

		class point_cmp
		{
		public:
			point_cmp(float e = FLT_MIN) :eps(e) {}

			bool operator()(const trimesh::vec3& v0, const trimesh::vec3& v1) const
			{
				if (fabs(v0.x - v1.x) <= eps)
				{
					if (fabs(v0.y - v1.y) <= eps)
					{
						return (v0.z < v1.z - eps);
					}
					else return (v0.y < v1.y - eps);
				}
				else return (v0.x < v1.x - eps);
			}
		private:
			float eps;
		};

		typedef std::map<trimesh::vec3, int, point_cmp> unique_point;
		typedef unique_point::iterator point_iterator;

		struct segment
		{
			int start;
			int end;
		};

		typedef std::map<trimesh::vec3, int, point_cmp> unique_point;
		typedef unique_point::iterator point_iterator;
		unique_point points;

		auto f = [&points](const trimesh::vec3& v)->int {
			int index = -1;
			point_iterator it = points.find(v);
			if (it != points.end())
			{
				index = (*it).second;
			}
			else
			{
				index = (int)points.size();
				points.insert(unique_point::value_type(v, index));
			}

			return index;
		};

		std::vector<segment> segments(segsize);
		for (size_t i = 0; i < segsize; ++i)
		{
			trimesh::vec3 v1 = lines.at(2 * i);
			trimesh::vec3 v2 = lines.at(2 * i + 1);

			segments.at(i).start = f(v1);
			segments.at(i).end = f(v2);
		}

		std::vector<trimesh::vec3> vecpoints(points.size());
		for (auto it = points.begin(); it != points.end(); ++it)
		{
			vecpoints.at((*it).second) = (*it).first;
		}

		std::vector<segment*> segmap(points.size(), nullptr);
		for (segment& s : segments)
		{
			segmap.at(s.start) = &s;
		}

		std::vector<bool> used(points.size(), false);

		auto check = [&used]() ->int {
			int index = -1;
			size_t size = used.size();
			for (size_t i = 0; i < size; ++i)
			{
				if (!used.at(i))
				{
					index = (int)i;
					break;
				}
			}
			return index;
		};

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
		int index = check();
		while (index >= 0)
		{
			used.at(index) = true;
			segment* seg = segmap.at(index);
			if (seg)
			{
				int s = seg->start;
				int e = seg->end;

				bool find = false;
				for (IndexPolygon& polygon : indexPolygons)
				{
					if (s == polygon.end)
					{
						polygon.polygon.push_back(e);
						polygon.end = e;
						find = true;
					}
					else if (e == polygon.start)
					{
						polygon.polygon.push_front(s);
						polygon.start = s;
						find = true;
					}

					if (find)
						break;
				}

				if (!find)
				{
					IndexPolygon polygon;
					polygon.polygon.push_back(s);
					polygon.polygon.push_back(e);
					polygon.start = s;
					polygon.end = e;
					indexPolygons.emplace_back(polygon);
				}
			}
			index = check();
		}
		size_t indexPolygonSize = indexPolygons.size();
		std::map<int, IndexPolygon*> IndexPolygonMap;
		for (size_t i = 0; i < indexPolygonSize; ++i)
		{
			IndexPolygon& p1 = indexPolygons.at(i);
			if (!p1.closed())
				IndexPolygonMap.insert(std::pair<int, IndexPolygon*>(p1.start, &p1));
		}

		////sort
		//for (size_t i = 0; i < indexPolygonSize; ++i)
		//{
		//	IndexPolygon& p1 = indexPolygons.at(i);
		//	for (size_t j = i + 1; j < indexPolygonSize; ++j)
		//	{
		//		IndexPolygon& p2 = indexPolygons.at(j);

		//		if (p1.end > p2.start)
		//		{
		//			std::swap(p1.polygon, p2.polygon);
		//			std::swap(p1.start, p2.start);
		//			std::swap(p1.end, p2.end);
		//		}
		//	}
		//}
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

			//for (size_t j = i + 1; j < indexPolygonSize; ++j)
			//{
			//	IndexPolygon& p2 = indexPolygons.at(j);
			//	if (p2.polygon.size() == 0)
			//		continue;

			//	bool merged = false;
			//	if (p1.start == p2.end)
			//	{
			//		p1.start = p2.start;
			//		for (auto it = p2.polygon.rbegin(); it != p2.polygon.rend(); ++it)
			//		{
			//			if ((*it) != p1.polygon.front()) p1.polygon.push_front((*it));
			//		}
			//		merged = true;
			//	}else if (p1.end == p2.start)
			//	{
			//		p1.end = p2.end;
			//		for (auto it = p2.polygon.begin(); it != p2.polygon.end(); ++it)
			//		{
			//			if ((*it) != p1.polygon.back()) p1.polygon.push_back((*it));
			//		}
			//		merged = true;
			//	}

			//	if (merged)
			//	{
			//		p2.polygon.clear();
			//	}
			//}
		}

		size_t polygonSize = indexPolygons.size();
		if (polygonSize > 0)
		{
			polygons.reserve(polygonSize);
			for (size_t i = 0; i < polygonSize; ++i)
			{
				std::vector<int> polygon;
				IndexPolygon& ipolygon = indexPolygons.at(i);
				for (int iindex : ipolygon.polygon)
				{
					polygon.push_back(iindex);
				}

				if (polygon.size() > 0)
				{
					polygons.emplace_back(polygon);
				}
			}
		}
		uniPoints.swap(vecpoints);
	}

	trimesh::TriMesh* ZSplit::splitPolygon(float z)
	{
		if (!m_mesh || 
			m_mesh->vertices.size() == 0 ||
			m_mesh->faces.size() == 0)
			return nullptr;

		std::vector<float> distances;
		size_t vertex_size = m_mesh->vertices.size();
		distances.resize(vertex_size);

#define min_value 1e-4
		bool allPositive = true;
		bool allNegtive = true;
		for (int i = 0; i < vertex_size; ++i)
		{
			distances.at(i) = m_mesh->vertices.at(i).z - z;

			if (distances.at(i) < -min_value)
				allPositive = false;
			if (distances.at(i) > min_value)
				allNegtive = false;
		}

		if (allPositive || allNegtive)
			return nullptr;

		std::vector<trimesh::TriMesh::Face> collideFaces;
		for (trimesh::TriMesh::Face& f : m_mesh->faces)
		{
			float l0 = distances.at(f.x);
			float l1 = distances.at(f.y);
			float l2 = distances.at(f.z);
			if (l0 > 0.0f && l1 > 0.0f && l2 > 0.0f)
			{
			}
			else if (l0 < 0.0f && l1 < 0.0f && l2 < 0.0f)
			{
			}
			else
				collideFaces.push_back(f);
		}

		auto addmesh = [](trimesh::TriMesh* mesh, const trimesh::vec3& v1, const trimesh::vec3& v2, const trimesh::vec3& v3) {
			int index = (int)mesh->vertices.size();
			mesh->vertices.push_back(v1);
			mesh->vertices.push_back(v2);
			mesh->vertices.push_back(v3);

			mesh->faces.push_back(trimesh::TriMesh::Face(index, index + 1, index + 2));
		};

		std::vector<trimesh::vec3> lines;
		//process collide
		auto fcollid = [&addmesh, this, &distances, &lines](int t, int i1, int i2)
		{
			trimesh::vec3& tv = m_mesh->vertices.at(t);
			trimesh::vec3& v1 = m_mesh->vertices.at(i1);
			trimesh::vec3& v2 = m_mesh->vertices.at(i2);
			float dv = distances.at(t);
			float d1 = distances.at(i1);
			float d2 = distances.at(i2);

			if (d1 == 0.0f && d2 == 0.0f)
			{
				if (dv >= 0.0f)
				{
					lines.push_back(v1);
					lines.push_back(v2);
				}
				else
				{
					lines.push_back(v2);
					lines.push_back(v1);
				}
			}
			else if (d1 == 0.0f && dv * d2 >= 0.0f)
			{
			}
			else if (d2 == 0.0f && dv * d1 >= 0.0f)
			{
			}
			else
			{
				trimesh::vec3 c1 = (dv / (dv - d1)) * v1 - (d1 / (dv - d1)) * tv;
				trimesh::vec3 c2 = (dv / (dv - d2)) * v2 - (d2 / (dv - d2)) * tv;

				if (dv > 0.0f)
				{
					lines.push_back(c1);
					lines.push_back(c2);
				}
				else if (dv < 0.0f)
				{
					lines.push_back(c2);
					lines.push_back(c1);
				}
			}
		};

		int faceNum = (int)m_mesh->faces.size();
		for (int i = 0; i < faceNum; ++i)
		{
			trimesh::TriMesh::Face& f = m_mesh->faces.at(i);
			float l0 = distances.at(f.x) * distances.at(f.y);
			float l1 = distances.at(f.y) * distances.at(f.z);
			float l2 = distances.at(f.x) * distances.at(f.z);
			if (l0 == 0.0f && l1 == 0.0f && l2 == 0.0f)
				continue;

			if (l0 >= 0.0f && (l1 <= 0.0f || l2 <= 0.0f))
			{
				fcollid(f.z, f.x, f.y);
			}
			else if (l0 < 0.0f)
			{
				if (l1 <= 0.0f)
				{
					fcollid(f.y, f.z, f.x);
				}
				else if (l2 <= 0.0f)
				{
					fcollid(f.x, f.y, f.z);
				}
			}
		}

		//fill hole
		trimesh::TriMesh* mesh = new trimesh::TriMesh();
		std::vector<std::vector<int>> m_polygons;
		lines2polygon(lines, m_polygons, mesh->vertices);

		std::vector<trimesh::dvec2> polygons2;
		polygons2.reserve(mesh->vertices.size());

		for (size_t i = 0; i < mesh->vertices.size(); ++i)
		{
			const trimesh::vec3& v = mesh->vertices.at(i);
			polygons2.push_back(trimesh::dvec2(v.x, v.y));
		}

		mmesh::PolygonStack pstack;
		pstack.generates(m_polygons, polygons2, mesh->faces);
		return mesh;
	}
}