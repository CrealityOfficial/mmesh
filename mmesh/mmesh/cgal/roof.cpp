#include "roof.h"

void seperate1423(ClipperLib::PolyTree* polyTree, std::vector<PolyPair*>& polyPairs)
{
    for (ClipperLib::PolyNode* node1 : polyTree->Childs)
    {
        std::vector<ClipperLib::PolyNode*>& node2 = node1->Childs;
        std::vector<ClipperLib::PolyNode*> node3;
        for (ClipperLib::PolyNode* n : node2)
            node3.insert(node3.end(), n->Childs.begin(), n->Childs.end());
        std::vector<ClipperLib::PolyNode*> node4;
        for (ClipperLib::PolyNode* n : node3)
            node4.insert(node4.end(), n->Childs.begin(), n->Childs.end());

        PolyPair* pair1 = new PolyPair();
        pair1->clockwise = false;
        pair1->outer = node1;
        pair1->inner.swap(node4);
        polyPairs.push_back(pair1);

        for (ClipperLib::PolyNode* n : node2)
        {
            PolyPair* pair = new PolyPair();
            pair->clockwise = true;
            pair->outer = n;
            pair->inner = n->Childs;
            polyPairs.push_back(pair);
        }
    }
}

#if defined(WIN32) && defined(USE_CGAL)
#include <boost/shared_ptr.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef K::Point_2                    Point;
typedef CGAL::Polygon_2<K>            Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;
typedef CGAL::Straight_skeleton_2<K>  Straight_skeleton;
typedef Straight_skeleton::Halfedge_const_iterator Halfedge_const_iterator;
typedef Straight_skeleton::Halfedge_const_handle   Halfedge_const_handle;
typedef Straight_skeleton::Vertex_const_iterator Vertex_const_iterator;
typedef Straight_skeleton::Vertex_const_handle Vertex_const_handle;
typedef Straight_skeleton::Face_const_iterator Face_const_iterator;
typedef Straight_skeleton::Face_const_handle Face_const_handle;

typedef boost::shared_ptr<Straight_skeleton> Straight_skeleton_ptr;

namespace mmesh
{
    void fill_polygon(Polygon_2& polygon, ClipperLib::Path* path, bool inverse)
    {
        size_t size = path->size();
        if (!inverse)
        {
            for (size_t i = 0; i < size; ++i)
            {
                ClipperLib::IntPoint& p = path->at(i);
                polygon.push_back(Point(INT2MM(p.X), INT2MM(p.Y)));
            }
        }
        else
        {
            for (size_t i = 0; i < size; ++i)
            {
                ClipperLib::IntPoint& p = path->at(size - 1 - i);
                polygon.push_back(Point(INT2MM(p.X), INT2MM(p.Y)));
            }
        }
    }

    void build_polygon_with_holes(Polygon_with_holes* input, PolyPair* pair)
    {
        bool inverse = pair->clockwise;

        Polygon_2& poly = input->outer_boundary();
        ClipperLib::Path& path = pair->outer->Contour;
        size_t size = path.size();
        if (size < 3 /*|| inverse*/)
            return;

        fill_polygon(input->outer_boundary(), &path, inverse);

        for (ClipperLib::PolyNode* n : pair->inner)
        {
            ClipperLib::Path& npath = n->Contour;
            size_t nsize = npath.size();
            if (nsize < 3)
                return;

            Polygon_2 hole;
            fill_polygon(hole, &npath, inverse);

            input->add_hole(hole);
        }
    }

    bool test_simple_polygon(Polygon_with_holes& input)
    {
        //check the validity of the input and fix orientation
        if (!input.outer_boundary().is_simple())
        {
            std::cerr << "ERROR: outer boundary is not simple.";
            return false;
        }

        bool valid = true;
        for (Polygon_with_holes::Hole_iterator it = input.holes_begin();
            it != input.holes_end(); ++it)
        {
            if (!it->is_simple())
            {
                valid = false;
                break;
            }
        }

        return valid;
    }

	void buildRoofs(ClipperLib::PolyTree* polyTree, std::vector<std::vector<trimesh::vec3>*>& patches, double roofHeight, double thickness)
	{
        std::vector<PolyPair*> pairs;
        seperate1423(polyTree, pairs);

        for (PolyPair*  pair : pairs)
        {
            Polygon_with_holes input;
            build_polygon_with_holes(&input, pair);
            if (!test_simple_polygon(input))
                continue;

            Straight_skeleton_ptr ss = CGAL::create_interior_straight_skeleton_2(input);
            if (ss)
            {

            }
            else
            {
                std::cerr << "ERROR creating interior straight skeleton" << std::endl;
            }
        }

        for (PolyPair* pair : pairs)
        {
            delete pair;
        }
        pairs.clear();
	}

    ClipperLib::IntPoint cgal_to_point(const Point& point)
    {
        ClipperLib::IntPoint p;
        p.X = (ClipperLib::cInt)(1000.0 * point.x());
        p.Y = (ClipperLib::cInt)(1000.0 * point.y());

        return p;
    }

    void traitSkeletonPoints(ClipperLib::PolyTree* roofPoint, Straight_skeleton_ptr skeleton)
    {
        for (Vertex_const_iterator vit = skeleton->vertices_begin();
            vit != skeleton->vertices_end(); ++vit)
        {
            Vertex_const_handle h = vit;
            if(h->is_contour() || h->is_skeleton())
                roofPoint->Contour.push_back(cgal_to_point(h->point()));
        }
    }

    void traitSkeletonLine(ClipperLib::PolyTree* roofLine, Straight_skeleton_ptr skeleton)
    {
        for (Halfedge_const_iterator hit = skeleton->halfedges_begin();
            hit != skeleton->halfedges_end(); ++hit)
        {
            Halfedge_const_handle h = hit;
            
            roofLine->Contour.push_back(cgal_to_point(h->vertex()->point()));
            roofLine->Contour.push_back(cgal_to_point(h->opposite()->vertex()->point()));
        }
    }

    void traitSkeletonFace(ClipperLib::Paths* roofFace, Straight_skeleton_ptr skeleton,bool clockwise=false)
    {
        size_t faceSize = skeleton->size_of_faces();
        size_t roofFase = roofFace->size();
        if (faceSize > 0)
            roofFace->resize(roofFase +faceSize);

        int index = roofFase;
        for (Face_const_iterator fit = skeleton->faces_begin();
            fit != skeleton->faces_end(); ++fit)
        {
            ClipperLib::Path& path = roofFace->at(index);
            //if (index < roofFase + faceSize)
            {
                Halfedge_const_handle he = fit->halfedge();
                Halfedge_const_handle h = he;
                do
                {
                    ClipperLib::IntPoint p= cgal_to_point(h->vertex()->point());
                    if (h->vertex()->is_skeleton())
                    {
                        p.Z = 500;
						if (clockwise)
						{
                            p.Z = 300;
						}
                    }
                    path.push_back(p);
                    h = h->next();
                } while (h != he);

            }

            ++index;
        }
    }

    void roofLine(ClipperLib::PolyTree* polyTree,
        ClipperLib::PolyTree* roof, ClipperLib::PolyTree* roofPoint, ClipperLib::Paths* roofFace, bool onePoly)
    {
        std::vector<PolyPair*> pairs;
        seperate1423(polyTree, pairs);

        for (PolyPair* pair : pairs)
        {
            if (onePoly)
            {
				if (pair->clockwise) //outer
				{
					continue;
				}
            }

            Polygon_with_holes input;
            build_polygon_with_holes(&input, pair);
            if (!test_simple_polygon(input))
                continue;

            ClipperLib::PolyTree opposite;
            Straight_skeleton_ptr aSkeleton = CGAL::create_interior_straight_skeleton_2(input);
            if (aSkeleton)
            {
                if (roofPoint)
                {
                    traitSkeletonPoints(roofPoint, aSkeleton);
                }

                if (roof)
                {
                    traitSkeletonLine(roof, aSkeleton);
                }

                if (roofFace)
                {
                    traitSkeletonFace(roofFace, aSkeleton, pair->clockwise);
                }

			 //   Halfedge_const_handle h = hit;
				//int idege = aSkeleton->size_of_halfedges();
				////                     if (/*h->is_bisector() &&*/ /*((h->id() % 2) == 0*/) 
				//// //                         && !h->has_infinite_time()
				//// //                         && !h->opposite()->has_infinite_time()
				////                         )
				//{
				//	if (h->is_border() && (h->id() % 2) != 0)
				//		continue;

				//	int i = h->face()->id();

				//	ClipperLib::IntPoint n = cgal_to_point(h->opposite()->vertex()->point());
				//	ClipperLib::IntPoint m = cgal_to_point(h->vertex()->point());

				//	if ((h->id() % 2) == 0)
				//	{
				//		if (h->is_inner_bisector())
				//		{
				//			n.Z = 5000;
				//		}
				//		if (h->is_bisector())
				//		{
				//			m.Z = 5000;
				//		}
				//		vct.push_back(i);
				//		vct.push_back(i);
				//		roof->Contour.push_back(n);
				//		roof->Contour.push_back(m);
				//	}
				//	else
				//	{
				//		if (h->is_inner_bisector())
				//		{
				//			m.Z = 5000;
				//		}
				//		if (h->is_bisector())
				//		{
				//			n.Z = 5000;
				//		}

				//		vctopposite.push_back(i);
				//		vctopposite.push_back(i);
				//		opposite.Contour.push_back(m);
				//		opposite.Contour.push_back(n);
				//	}
				//}
                //std::vector<int> vct;
                //std::vector<int> vctopposite;              

                //for (Halfedge_const_iterator hit = aSkeleton->halfedges_begin();
                //    hit != aSkeleton->halfedges_end(); ++hit)
                //{
                //    Halfedge_const_handle h = hit;
                //    int idege = aSkeleton->size_of_halfedges();
//              //       if (/*h->is_bisector() &&*/ /*((h->id() % 2) == 0*/) 
// //           //              && !h->has_infinite_time()
// //           //              && !h->opposite()->has_infinite_time()
//              //           )
                //    {
                //        if (h->is_border() && (h->id() % 2)!=0)
                //            continue;
                //
				//		int i = h->face()->id();				
                //
				//		ClipperLib::IntPoint n = cgal_to_point(h->opposite()->vertex()->point());  
				//		ClipperLib::IntPoint m = cgal_to_point(h->vertex()->point());
                //
                //        if ((h->id() % 2) == 0)
                //        {
				//			if (h->is_inner_bisector())
				//			{
				//				n.Z = 5000;
				//			}
				//			if (h->is_bisector())
				//			{
				//				m.Z = 5000;
				//			}
                //            vct.push_back(i);
                //            vct.push_back(i);
				//			roof->Contour.push_back(n);
				//			roof->Contour.push_back(m);
                //        } 
                //        else
                //        {
				//			if (h->is_inner_bisector())
				//			{
				//				m.Z = 5000;
				//			}
				//			if (h->is_bisector())
				//			{
				//				n.Z = 5000;
				//			}
                //
                //            vctopposite.push_back(i);
                //            vctopposite.push_back(i);
                //            opposite.Contour.push_back(m);
                //            opposite.Contour.push_back(n);
                //        }
                //    }
                //}
                //
                //paths->resize(aSkeleton->size_of_faces());
                //for (size_t i = 0; i < vct.size(); i++)
                //{
 				//	bool isExit = false;
  				//	for (size_t j = 0; j < paths->at(vct.at(i)).size(); j++)
  				//	{
  				//		if (paths->at(vct.at(i)).at(j).X == roof->Contour.at(i).X
  				//			&& paths->at(vct.at(i)).at(j).Y == roof->Contour.at(i).Y
  				//			&& paths->at(vct.at(i)).at(j).Z == roof->Contour.at(i).Z)
  				//		{
  				//			isExit = true;
  				//			break;
  				//		}
  				//	}
  				//	if (!isExit)
  				//	{
  				//		paths->at(vct.at(i)).push_back(roof->Contour.at(i));
  				//	}
                //}
                //
				//for (int i = vctopposite.size()- 1; i >=0; i--)
 				//{
				//	bool isExit = false;
				//	for (size_t j = 0; j < paths->at(vctopposite.at(i)).size(); j++)
				//	{
				//		if (paths->at(vctopposite.at(i)).at(j).X == opposite.Contour.at(i).X
				//			&& paths->at(vctopposite.at(i)).at(j).Y == opposite.Contour.at(i).Y
				//			&& paths->at(vctopposite.at(i)).at(j).Z == opposite.Contour.at(i).Z)
				//		{
				//			isExit = true;
				//			break;
				//		}
				//	}
				//	if (!isExit)
				//	{
				//		paths->at(vctopposite.at(i)).push_back(opposite.Contour.at(i));
				//	}
				//}                        
            }
            else
            {
                std::cerr << "ERROR creating interior straight skeleton" << std::endl;
            }
        }

        for (PolyPair* pair : pairs)
        {
            delete pair;
        }
        pairs.clear();
    }
}

#else

namespace mmesh
{
    void buildRoofs(ClipperLib::PolyTree* polyTree, std::vector<std::vector<trimesh::vec3>*>& patches, double roofHeight, double thickness)
    {

    }

    void roofLine(ClipperLib::PolyTree* polyTree,
        ClipperLib::PolyTree* roof, ClipperLib::PolyTree* roofPoint, ClipperLib::Paths* roofFace)
    {
    }
}
#endif