#include "polygonstackex.h"

// CGAL
#include "CGAL/Exact_predicates_inexact_constructions_kernel.h"
#include "CGAL/Constrained_Delaunay_triangulation_2.h"
#include "CGAL/Triangulation_face_base_with_info_2.h"
#include "CGAL/Polygon_2.h"


namespace mmesh
{
	struct FaceInfo_2
	{
		FaceInfo_2() {}
		int nesting_level;
		bool in_domain() { return nesting_level % 2; }
	};

	typedef CGAL::Exact_predicates_inexact_constructions_kernel      K;
	typedef CGAL::Triangulation_vertex_base_2<K>                     Vb;
	typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo_2, K> Fbb;
	typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb>      Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb>             TDS;
	typedef CGAL::Exact_predicates_tag                               Itag;
	typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
	typedef CDT::Point                                               Point;
	typedef CGAL::Polygon_2<K>                                       Polygon_2;
	typedef CDT::Face_handle                                         Face_handle;


	static void mark_domains(CDT& ct, Face_handle start, int index, std::list<CDT::Edge>& border)
	{
		if (start->info().nesting_level != -1)
		{
			return;
		}
		std::list<Face_handle> queue;
		queue.push_back(start);
		while (!queue.empty())
		{
			Face_handle fh = queue.front();
			queue.pop_front();
			if (fh->info().nesting_level == -1)
			{
				fh->info().nesting_level = index;
				for (int i = 0; i < 3; i++)
				{
					CDT::Edge e(fh, i);
					Face_handle n = fh->neighbor(i);
					if (n->info().nesting_level == -1)
					{
						if (ct.is_constrained(e))
						{
							border.push_back(e);
						}
						else
						{
							queue.push_back(n);
						}
					}
				}
			}
		}
	}

	static void mark_domains(CDT& cdt)
	{
		for (CDT::Face_handle f : cdt.all_face_handles())
		{
			f->info().nesting_level = -1;
		}
		std::list<CDT::Edge> border;
		mark_domains(cdt, cdt.infinite_face(), 0, border);
		while (!border.empty())
		{
			CDT::Edge e = border.front();
			border.pop_front();
			Face_handle n = e.first->neighbor(e.second);
			if (n->info().nesting_level == -1)
			{
				mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
			}
		}
	}

	// based on CGAL
	// site: https://doc.cgal.org/latest/Triangulation_2/index.html#title30
	void Triangulation(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points, trimesh::TriMesh* mesh)
	{
		CDT cdt;

		for (size_t i = 0; i < polygons.size(); i++)
		{
			Polygon_2 cgal_polygon;
			for (size_t j = 0; j < polygons[i].size(); j++)
			{
				trimesh::dvec2& point = points[polygons[i][j]];
				cgal_polygon.push_back(Point(point.x, point.y));
			}
			cdt.insert_constraint(cgal_polygon.vertices_begin(), cgal_polygon.vertices_end(), true);
		}

		mark_domains(cdt);

		int vertex_id_start = -1;

		for (Face_handle face : cdt.finite_face_handles())
		{
			if (face->info().in_domain())
			{
				vertex_id_start = mesh->vertices.size();

				mesh->vertices.emplace_back(face->vertex(0)->point().x(), face->vertex(0)->point().y(), 0.0f);
				mesh->vertices.emplace_back(face->vertex(1)->point().x(), face->vertex(1)->point().y(), 0.0f);
				mesh->vertices.emplace_back(face->vertex(2)->point().x(), face->vertex(2)->point().y(), 0.0f);

				mesh->faces.emplace_back(vertex_id_start, vertex_id_start + 1, vertex_id_start + 2);
			}
		}
	}


	PolygonStackEx::PolygonStackEx()
	{

	}

	PolygonStackEx::~PolygonStackEx()
	{

	}

	void PolygonStackEx::generates(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points, trimesh::TriMesh* mesh)
	{
		Triangulation(polygons, points, mesh);
	}

}