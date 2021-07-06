#pragma once

namespace trimesh
{
    class TriMesh;
}

namespace mmesh
{
    class SimplifyInterface
    {
    public:
        // remove duplicated vertex and make a compact mesh, new TriMesh object allocated and returned
        static trimesh::TriMesh* remove_duplicated_vertex(trimesh::TriMesh* input, trimesh::TriMesh* output);

        // seems like very slow...
        // @param <percent>: indicates the target edge number
        static trimesh::TriMesh* simplify_cgal(trimesh::TriMesh* input, double percent);

        // @param <percent>: indicates the target face number
        static trimesh::TriMesh* simplify_vcg(trimesh::TriMesh* input, double percent, trimesh::TriMesh* output);

        // about 4x as fast as simplify_vcg
        // @param <percent>: indicates the target face number
        static trimesh::TriMesh* simplify_fast(trimesh::TriMesh* input, double percent, trimesh::TriMesh* output);
    };
}