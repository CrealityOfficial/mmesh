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
        std::string belt_support_enable;
        std::string support_angle;
        std::string machine_width;
        std::string machine_depth;

    };

    std::vector<trimesh::TriMesh*> sliceBelt(const std::vector<trimesh::TriMesh*>& meshs, const Cr30Param& cr30Param, ccglobal::Tracer* m_progress = nullptr);
}

#endif //