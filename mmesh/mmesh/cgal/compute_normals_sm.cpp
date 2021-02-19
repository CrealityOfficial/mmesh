#include "compute_normals_sm.h"

#if defined(WIN32) && defined(USE_CGAL)
namespace ComputeNormalsSM
{
    namespace PMP = CGAL::Polygon_mesh_processing;
    int main(int argc, char* argv[])
    {
        const char* filename = (argc > 1) ? argv[1] : "data/eight.off";
        std::ifstream input(filename);

        Surface_mesh mesh;
        if (!input || !(input >> mesh) || mesh.is_empty()) {
            std::cerr << "Not a valid off file." << std::endl;
            return 1;
        }

        auto vnormals = mesh.add_property_map<vertex_descriptor, Vector>("v:normals", CGAL::NULL_VECTOR).first;
        auto fnormals = mesh.add_property_map<face_descriptor, Vector>("f:normals", CGAL::NULL_VECTOR).first;

        //PMP::compute_normals(mesh, vnormals, fnormals);
        PMP::compute_face_normals(mesh, fnormals);

        std::cout << "Vertex normals :" << std::endl;
        for (vertex_descriptor vd : vertices(mesh))
            std::cout << vnormals[vd] << std::endl;

        std::cout << "Face normals :" << std::endl;
        for (face_descriptor fd : faces(mesh))
            std::cout << fnormals[fd] << std::endl;

        return 0;
    }



    void computeFaceNormals(const Surface_mesh & mesh,fnormalsMap & fnormals)
    {
        assert(!is_empty(mesh));
        PMP::compute_face_normals(mesh, fnormals);
    }



    void getFaceNormals(Surface_mesh* meshPtr, fnormalsMap& fnormals)
    {

        std::cout << "Test with Surface_mesh, and kernel: " << typeid(K).name() << std::endl;
        assert(!is_empty(*meshPtr));
        fnormals = meshPtr->add_property_map<face_descriptor, Vector>("f:normals", CGAL::NULL_VECTOR).first;
        PMP::compute_face_normals(*meshPtr, fnormals);
        for (face_descriptor fd : faces(*meshPtr))
        {
            std::cout << "computeFaceNormals==" << fnormals[fd] << std::endl;
            break;
        }



    }


}

#endif