#ifndef MMESH_SHAPECREATOR_1619603379371_H
#define MMESH_SHAPECREATOR_1619603379371_H
#include <trimesh2/Box.h>
#include <trimesh2/TriMesh.h>
#include "mmesh/trimesh/box2dgrid.h"

namespace mmesh
{
	trimesh::TriMesh* createBox(const trimesh::box3& box);

    static float static_box_position[24] = {
0.0f, 0.0f, 0.0f,
1.0f, 0.0f, 0.0f,
1.0f, 1.0f, 0.0f,
0.0f, 1.0f, 0.0f,
0.0f, 0.0f, 1.0f,
1.0f, 0.0f, 1.0f,
1.0f, 1.0f, 1.0f,
0.0f, 1.0f, 1.0f
    };

    static unsigned static_box_triangles_indices[36] = {
        2, 3, 0,
        2, 0, 1,
        0, 1, 5,
        0, 5, 4,
        1, 2, 6,
        1, 6, 5,
        2, 3, 7,
        2, 7, 6,
        3, 0, 4,
        3, 4, 7,
        4, 5, 6,
        4, 6, 7
    };


    class ShapeCreator
    {
    public:
        ShapeCreator();
        virtual ~ShapeCreator();

        static trimesh::TriMesh* createCylinderMesh(trimesh::vec3 top, trimesh::vec3 bottom, float radius, int num = 20, float theta = 0.0f);
        static trimesh::TriMesh* createCuboidMesh(trimesh::vec3 size);
        static trimesh::TriMesh* createCuboidMesh(trimesh::vec3 bottom, trimesh::vec3 top, int n, float radius, float startTheta = 0.7853981634f);

        static trimesh::TriMesh* createCuboidMesh(std::vector<mmesh::VerticalSeg>& segments, std::vector<int>& chunks, trimesh::fxform& matrix, int n, float radius, float startTheta = 0.7853981634f);

        /* create a pyramid */
        static trimesh::TriMesh* createPyramidMesh(int nbBottomVertex, float bottomRadius, float height);

    protected:
    };
}

#endif // MMESH_SHAPECREATOR_1619603379371_H