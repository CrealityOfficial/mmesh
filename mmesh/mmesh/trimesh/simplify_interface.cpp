#include "simplify_interface.h"
#include "trimesh2/TriMesh.h"
#include "mmesh/cgal/simplify.h"
#include "mmesh/vcg/trimesh_simplify.h"
#include "thirdparty/Simplify.h"


namespace mmesh
{
    trimesh::TriMesh* SimplifyInterface::remove_duplicated_vertex(trimesh::TriMesh* input, trimesh::TriMesh* output)
    {
        if (!output)
        {
            return nullptr;
        }

        output->clear();
        output->vertices.resize(input->vertices.size());
        output->faces.resize(input->faces.size());

        std::vector<unsigned int> map_sorted_2_origin(input->vertices.size());   // { index after sorting -> origin index }
        std::vector<unsigned int> map_origin_2_removed(input->vertices.size());  // { origin index -> index after removing duplicated vertex }
        unsigned int vertices_count = 0;

        for (size_t i = 0; i < map_sorted_2_origin.size(); i++)
        {
            map_sorted_2_origin[i] = i;
        }

        std::sort(map_sorted_2_origin.begin(), map_sorted_2_origin.end(), [input] (int a, int b) { return input->vertices[a] < input->vertices[b]; });

        for (size_t i = 0; i < map_sorted_2_origin.size(); i++)
        {
            unsigned int index_origin = map_sorted_2_origin[i];

            if (i == 0 || output->vertices[vertices_count - 1] != input->vertices[index_origin])
            {
                output->vertices[vertices_count++] = input->vertices[index_origin];
            }

            map_origin_2_removed[index_origin] = vertices_count - 1;
        }

        output->vertices.resize(vertices_count);

        for (size_t i = 0; i < input->faces.size(); i++)
        {
            trimesh::TriMesh::Face& face = input->faces[i];
            output->faces[i].set(map_origin_2_removed[face.x], map_origin_2_removed[face.y], map_origin_2_removed[face.z]);
        }

        return output;
    }

    trimesh::TriMesh* SimplifyInterface::simplify_cgal(trimesh::TriMesh* input, double percent)
    {
        percent = percent < 0.01 ? 0.01 : (percent > 1.0 ? 1.0 : percent);
        cxSimplifySetTypeVaue(&percent, cxEDGE_RATIO);
        return cxSimplifyOperateMeshObj(input, cxEDGE_RATIO);
    }

    trimesh::TriMesh* SimplifyInterface::simplify_vcg(trimesh::TriMesh* input, double percent, trimesh::TriMesh* output)
    {
        trimesh::TriMesh compact_mesh;
        return mmesh::TrimeshSimplify(remove_duplicated_vertex(input, &compact_mesh)).Perform_quadric(percent, output);
    }

    trimesh::TriMesh* SimplifyInterface::simplify_fast(trimesh::TriMesh* input, double percent, trimesh::TriMesh* output)
    {
        if (!output)
        {
            return nullptr;
        }

        trimesh::TriMesh compact_mesh;
        remove_duplicated_vertex(input, &compact_mesh);

        Simplify::vertices.clear();
        Simplify::triangles.clear();

        // import
        Simplify::vertices.resize(compact_mesh.vertices.size());

        for (size_t i = 0; i < compact_mesh.vertices.size(); i++)
        {
            Simplify::vertices[i].p.x = compact_mesh.vertices[i].x;
            Simplify::vertices[i].p.y = compact_mesh.vertices[i].y;
            Simplify::vertices[i].p.z = compact_mesh.vertices[i].z;
        }

        Simplify::triangles.resize(compact_mesh.faces.size());

        for (size_t i = 0; i < compact_mesh.faces.size(); i++)
        {
            Simplify::triangles[i].v[0] = compact_mesh.faces[i][0];
            Simplify::triangles[i].v[1] = compact_mesh.faces[i][1];
            Simplify::triangles[i].v[2] = compact_mesh.faces[i][2];
            Simplify::triangles[i].attr = 0;
            Simplify::triangles[i].material = -1;
        }

        // simplify
        if (percent >= 1.0)
        {
            Simplify::simplify_mesh_lossless(true);
        }
        else
        {
            if (percent < 0.001)
            {
                percent = 0.001;
            }
            Simplify::simplify_mesh(percent * compact_mesh.faces.size());
        }

        // export
        output->clear();
        output->vertices.resize(Simplify::vertices.size());

        for (size_t i = 0; i < Simplify::vertices.size(); i++)
        {
            output->vertices[i].x = Simplify::vertices[i].p.x;
            output->vertices[i].y = Simplify::vertices[i].p.y;
            output->vertices[i].z = Simplify::vertices[i].p.z;
        }

        output->faces.resize(Simplify::triangles.size());

        for (size_t i = 0; i < Simplify::triangles.size(); i++)
        {
            if (Simplify::triangles[i].deleted)
            {
                continue;
            }
            output->faces[i][0] = Simplify::triangles[i].v[0];
            output->faces[i][1] = Simplify::triangles[i].v[1];
            output->faces[i][2] = Simplify::triangles[i].v[2];
        }

        return output;
    }
}