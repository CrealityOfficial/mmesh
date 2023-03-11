#ifndef TRIMESH_SELECT_H
#define TRIMESH_SELECT_H

#include "trimesh2/Vec.h"
#include <vector>

#include "ccglobal/tracer.h"

namespace trimesh
{
	class TriMesh;
}

namespace mmesh
{
    struct Cr30Param
    {
        bool belt_support_enable;
        double support_angle;
        double machine_width;
        double machine_depth;
        Cr30Param()
        {
            belt_support_enable = false;
            support_angle = 0.0f;
            machine_width = 0.0f;
            machine_depth = 0.0f;
        }
    };

    std::vector<trimesh::TriMesh*> sliceBelt(trimesh::TriMesh* mesh, const Cr30Param& cr30Param, ccglobal::Tracer* m_progress = nullptr);
}

#endif //