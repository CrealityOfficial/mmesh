#include "simplify.h"


#if WIN32
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>


#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
//////////////////////////////
// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>

#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>

#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>

//Placement wrapper
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

namespace PolyhedronSimplify
{
    static double gEdgeLengthThres = 0.05;
    static std::size_t gEdgeNumbers = 0;
    static double gEdgeRatio = 1.0;

    //typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    namespace SMS = CGAL::Surface_mesh_simplification;

    typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
    typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;

    typedef Polyhedron::Halfedge_handle    Halfedge_handle;
    typedef Polyhedron::Facet_handle       Facet_handle;
    typedef Polyhedron::Vertex_handle      Vertex_handle;

    typedef Polyhedron::Vertex_iterator Vertex_iterator;
    typedef Polyhedron::Facet_iterator Facet_iterator;
    typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;

    typedef Polyhedron::HalfedgeDS HalfedgeDS;
    typedef HalfedgeDS::Vertex   Vertex;
    typedef Vertex::Point Point;

    template <class HDS>
    class PolyhedronBuilder : public CGAL::Modifier_base<HDS>
    {
    public:
        PolyhedronBuilder(const trimesh::TriMesh& _mesh)
            :mesh(_mesh)
        {
        }

        void operator()(HDS& hds)
        {
            CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
            int pointSize = (int)mesh.vertices.size();
            int facesSize = (int)mesh.faces.size();

            B.begin_surface(pointSize, facesSize, 0);
            for (int i = 0; i < pointSize; i++)
            {
                const trimesh::vec3& v = mesh.vertices.at(i);
                B.add_vertex(Point(v.x, v.y, v.z));
            }
            for (int i = 0; i < facesSize; i++)
            {
                const trimesh::TriMesh::Face& f = mesh.faces.at(i);
                B.begin_facet();
                for (int j = 0; j < 3; j++)
                {
                    B.add_vertex_to_facet(f[j]);
                }
                B.end_facet();
            }
            B.end_surface();
        }
    private:
        const trimesh::TriMesh& mesh;
    };
    struct Border_is_constrained_edge_map
    {
        const Polyhedron* sm_ptr;
        typedef boost::graph_traits<Polyhedron>::edge_descriptor key_type;
        typedef bool                                               value_type;
        typedef value_type                                         reference;
        typedef boost::readable_property_map_tag                   category;

        Border_is_constrained_edge_map(const Polyhedron& sm) : sm_ptr(&sm) {}

        friend bool get(Border_is_constrained_edge_map m, const key_type& edge) {
            return CGAL::is_border(edge, *m.sm_ptr);
        }
    };

    // Placement class
    typedef SMS::Constrained_placement<SMS::Midpoint_placement<Polyhedron>,
        Border_is_constrained_edge_map > Placement;

    trimesh::TriMesh* cgal2trimesh(Polyhedron& polygon)
    {
        size_t vertexSize = polygon.size_of_vertices();
        size_t faceSize = polygon.size_of_facets();

        if ((vertexSize < 3) && (faceSize < 1))
            return nullptr;

        trimesh::TriMesh* mesh = new trimesh::TriMesh();
        mesh->vertices.resize(vertexSize);
        mesh->faces.resize(faceSize);

        size_t vertexIndex = 0;
        for (Vertex_iterator it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it, ++vertexIndex)
        {
            trimesh::vec3& v = mesh->vertices.at(vertexIndex);
            const Point& p = it->point();
            p.x();
            v[0] = CGAL::to_double(p.x());
            v[1] = CGAL::to_double(p.y());
            v[2] = CGAL::to_double(p.z());
        }

        size_t faceIndex = 0;
        for (Facet_iterator it = polygon.facets_begin(); it != polygon.facets_end(); ++it, ++faceIndex)
        {
            trimesh::TriMesh::Face& f = mesh->faces.at(faceIndex);

            Halfedge_facet_circulator j = it->facet_begin();
            CGAL_assertion(CGAL::circulator_size(j) == 3);

            int index = 0;
            do
            {
                f[index] = std::distance(polygon.vertices_begin(), j->vertex());
                ++index;
            } while (++j != it->facet_begin());
        }

        return mesh;
    }

    void trimesh2cgal(const trimesh::TriMesh& mesh, Polyhedron& polygon)
    {
        PolyhedronBuilder<HalfedgeDS> builder(mesh);
        polygon.delegate(builder);
    }

    bool simplitfyFunction(Polyhedron & polyhedronObj, cxSimplify_operation_type typeindex)
    {
        int r = 0;
        bool retval = false;
        if (!CGAL::is_triangle_mesh(polyhedronObj))
        {
            std::cerr << "Input geometry is not triangulated." << std::endl;
            retval = false;
            return retval;
        }

        // In this example, the simplification stops when the number of undirected edges
        // drops below 10% of the initial count
        switch (typeindex)
        {
        case cxEDGE_LENGTH:
        {
            const double threshold = gEdgeLengthThres;
            Border_is_constrained_edge_map bem(polyhedronObj);

            if (threshold > 0.0)
            {
                 r = SMS::edge_collapse(polyhedronObj,
                    CGAL::Surface_mesh_simplification::Edge_length_stop_predicate<double>(threshold),
                    CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index, polyhedronObj))
                    .halfedge_index_map(get(CGAL::halfedge_external_index, polyhedronObj))
                     .edge_is_constrained_map(bem)
                     .get_placement(Placement(bem)));
                 retval = true;

            }
        }
        break;
        case cxEDGE_NUMBERS:
        {
            const std::size_t stop_n = gEdgeNumbers;
            Border_is_constrained_edge_map bem(polyhedronObj);

            if (stop_n > 3)
            {
                SMS::Count_stop_predicate<Polyhedron> stop(stop_n);
                r = SMS::edge_collapse(polyhedronObj, stop,
                    CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index, polyhedronObj))
                    .halfedge_index_map(get(CGAL::halfedge_external_index, polyhedronObj))
                    .edge_is_constrained_map(bem)
                    .get_placement(Placement(bem))
                );
                retval = true;
            }
        }
        break;
        case cxEDGE_RATIO:
        {
            double stop_ratio = gEdgeRatio;
            SMS::Count_ratio_stop_predicate<Polyhedron> stop(stop_ratio);
            Border_is_constrained_edge_map bem(polyhedronObj);

            r = SMS::edge_collapse(polyhedronObj, stop,
                CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index, polyhedronObj))
                .halfedge_index_map(get(CGAL::halfedge_external_index, polyhedronObj))
                .edge_is_constrained_map(bem)
                .get_placement(Placement(bem))
            );
            retval = true;

        }
        break;
        default:
            printf("unknow simplity stype=%d \n", typeindex);
            retval = false;
            break;
    }

#if 0
        if (retval)
        {
            std::ofstream os("out.off");
            os.precision(17);
            os << polyhedronObj;
        }
#endif
        return retval;
    }


    trimesh::TriMesh* cxSimplifyOperateMeshObj(trimesh::TriMesh* meshObj, cxSimplify_operation_type typeindex)
    {

        trimesh::TriMesh* outMesh = NULL;
        Polyhedron polyhedronObj;
        bool validvalue = false;
        trimesh2cgal(*meshObj, polyhedronObj);
        if (polyhedronObj.is_empty())
            goto END;
        //main_test();
        validvalue= simplitfyFunction(polyhedronObj, typeindex);
        if (validvalue)
        {
            outMesh = cgal2trimesh(polyhedronObj);
        }
    END:
        return outMesh;
    }
}

trimesh::TriMesh* cxSimplifyOperateMeshObj(trimesh::TriMesh* meshObj,cxSimplify_operation_type typeindex)
{
    return PolyhedronSimplify::cxSimplifyOperateMeshObj(meshObj,typeindex);
}
void cxSimplifySetTypeVaue(void* setvalue, cxSimplify_operation_type typeindex)
{
    switch (typeindex)
    {
    case cxEDGE_LENGTH:
        PolyhedronSimplify::gEdgeLengthThres =*(double*)setvalue;
    break;
    case cxEDGE_NUMBERS:
        PolyhedronSimplify::gEdgeNumbers = *( std::size_t*)setvalue;
    break;
    case cxEDGE_RATIO:
        PolyhedronSimplify::gEdgeRatio = *(double*)setvalue;
    break;
    default:
        printf("unknow simplity stype=%d \n", typeindex);
        break;
    }
}

#else
trimesh::TriMesh* cxSimplifyOperateMeshObj(trimesh::TriMesh* meshObj, cxSimplify_operation_type typeindex)
{
    return nullptr;
}

void cxSimplifySetTypeVaue(void* setvalue, cxSimplify_operation_type typeindex)
{
}
#endif
