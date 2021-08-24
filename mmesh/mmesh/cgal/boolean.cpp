#include "boolean.h"

#if defined(WIN32) && defined(USE_CGAL)
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Surface_mesh.h>
#include <fstream>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3>             Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;
typedef K::Point_3                                      Point;

namespace PMP = CGAL::Polygon_mesh_processing;

trimesh::TriMesh* cgal2trimesh(Mesh& surfaceMesh)
{
    size_t vertexSize = surfaceMesh.number_of_vertices();
    size_t faceSize = surfaceMesh.number_of_faces();

    if ((vertexSize < 3) && (faceSize < 1))
        return nullptr;

    trimesh::TriMesh* mesh = new trimesh::TriMesh();
    mesh->vertices.resize(vertexSize);
    mesh->faces.reserve(faceSize);
    // The status of being used or removed is stored in a property map
    Mesh::Property_map<Mesh::Vertex_index, bool> removed
        = surfaceMesh.property_map<Mesh::Vertex_index, bool>("v:removed").first;

    {
        unsigned int i = 0, end = surfaceMesh.number_of_vertices() + surfaceMesh.number_of_removed_vertices();
        for (; i < end; ++i) {
            vertex_descriptor vh(i);
           if (surfaceMesh.is_removed(vh))
                continue;
         const Point& p = surfaceMesh.point(vh);
         trimesh::vec3& v = mesh->vertices.at(i);
         v[0] = CGAL::to_double(p.exact().x());
         v[1] = CGAL::to_double(p.exact().y());
         v[2] = CGAL::to_double(p.exact().z());

          //  std::cout << m.point(vh) << ((m.is_removed(vh)) ? "  R\n" : "\n");
        }
    }


    //for (faceIndex=0; faceIndex< faceSize; faceIndex++)
    {

        //Get face indices ...
        for (Mesh::Face_index face_index : surfaceMesh.faces()) 
        {
            //trimesh::TriMesh::Face& f = mesh->faces.at(faceIndex);
            trimesh::TriMesh::Face f;
#if 1
           // Mesh::Vertex_around_face_circulator fvit(surfaceMesh.halfedge(face_index), surfaceMesh);
             CGAL::Vertex_around_face_circulator<Mesh> vcirc(surfaceMesh.halfedge(face_index), surfaceMesh), done(vcirc);
            int index = 0;
            if (vcirc)
            {
                do
                {
                    //std::cout << (*vcirc).idx() << std::endl;
                    f[index] = (*vcirc).idx();
                    if (f[index] < 0)
                    {
                        break;
                    }
                    index += 1;
                } while (++vcirc != done && index < 3);
                if(index==3)
                    mesh->faces.emplace_back(f);
            }

            //if (test == 1)

#else
            {
                auto    vtc = surfaceMesh.vertices_around_face(surfaceMesh.halfedge(face_index));
                int     i = 0;
                for (auto v : vtc) f[i++] = static_cast<int>(v);
            }
#endif
        }
    }

        //for (Mesh::Face_iterator fit = surfaceMesh.faces_begin(); fit != surfaceMesh.faces_end(); ++fit)
        //{
        //    Mesh::Vertex_around_face_circulator vertices(*fit);
        //    Mesh::Vertex_around_face_circulator fvit = surfaceMesh.vertices(*fit), fvend = fvit;
        //    do
        //    {
        //        std::cout<< (*fvit).idx()<<std::endl;
        //    } while (++fvit != fvend);
        //}

    return mesh;
}

void trimesh2cgal(const trimesh::TriMesh& mesh, Mesh& surfaceMesh)
{
    int pointSize = (int)mesh.vertices.size();
    int facesSize = (int)mesh.faces.size();
    if (pointSize < 3 || facesSize < 1)
        return;
    for (int i = 0; i < pointSize; i++)
    {
        const trimesh::vec3& v = mesh.vertices.at(i);
        surfaceMesh.add_vertex(K::Point_3(v.x, v.y, v.z));

    }

    for (int i = 0; i < facesSize; i++)
    {
        const trimesh::TriMesh::Face& f = mesh.faces.at(i);
        if (f[0] < 0 || f[1] < 0 || f[2] < 0)
            continue;
        vertex_descriptor vh0(f[0]);
        vertex_descriptor vh1(f[1]);
        vertex_descriptor vh2(f[2]);

        surfaceMesh.add_face(vh0, vh1, vh2);
    }
}


int main_test()
{
    const char* filename1 = "data/blobby.off";
    const char* filename2 = "data/eight.off";
    std::ifstream input(filename1);

    Mesh mesh1, mesh2;
    if (!input || !(input >> mesh1))
    {
        std::cerr << "First mesh is not a valid off file." << std::endl;
        return 1;
    }
    input.close();
    input.open(filename2);
    if (!input || !(input >> mesh2))
    {
        std::cerr << "Second mesh is not a valid off file." << std::endl;
        return 1;
    }

    Mesh out;
    bool valid_union = PMP::corefine_and_compute_union(mesh1, mesh2, out);

    if (valid_union)
    {
        std::cout << "Union was successfully computed\n";
        std::ofstream output("union.off");
        output.precision(17);
        output << out;
        return 0;
    }
    std::cout << "Union could not be computed\n";
    return 1;
}

namespace mmesh
{
    trimesh::TriMesh* cxBooleanOperateMeshObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2, cxBoolean_operation_type typeindex)
    {
        trimesh::TriMesh* outMesh = NULL;
        Mesh surfaceMesh1;
        Mesh surfaceMesh2;
        Mesh surfaceOut;
        bool validvalue = false;
        trimesh2cgal(*Mesh1, surfaceMesh1);
        trimesh2cgal(*Mesh2, surfaceMesh2);
        if (surfaceMesh1.is_empty() || surfaceMesh2.is_empty())
            goto END;
        //main_test();

        PMP::stitch_borders(surfaceMesh1);
        PMP::stitch_borders(surfaceMesh2);

        switch (typeindex)
        {
        case cxBoolean_operation_type::CX_UNION:
            validvalue = PMP::corefine_and_compute_union(surfaceMesh1, surfaceMesh2, surfaceOut, PMP::parameters::throw_on_self_intersection(true));
            break;
        case cxBoolean_operation_type::CX_INTERSECTION:
            validvalue = PMP::corefine_and_compute_intersection(surfaceMesh1, surfaceMesh2, surfaceOut, PMP::parameters::throw_on_self_intersection(true));
            break;
        case cxBoolean_operation_type::CX_TM1_MINUS_TM2:
            validvalue = PMP::corefine_and_compute_difference(surfaceMesh1, surfaceMesh2, surfaceOut, PMP::parameters::throw_on_self_intersection(true));
            break;
        case cxBoolean_operation_type::CX_TM2_MINUS_TM1:
            validvalue = PMP::corefine_and_compute_difference(surfaceMesh2, surfaceMesh1, surfaceOut, PMP::parameters::throw_on_self_intersection(true));
            break;
        }
        if (validvalue)
        {
#if 0
            std::ofstream output("bool_result.off");
            output.precision(17);
            output << surfaceOut;
#endif
            outMesh = cgal2trimesh(surfaceOut);
        }
    END:
        return outMesh;
    }
}

#else
namespace mmesh
{
    trimesh::TriMesh* cxBooleanOperateMeshObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2, cxBoolean_operation_type typeindex)
    {
        return nullptr;
    }
}
#endif // WIN32
