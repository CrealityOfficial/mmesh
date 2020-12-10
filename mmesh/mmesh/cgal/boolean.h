#ifndef MMESH_BOOLEAN_1607481576423_H
#define MMESH_BOOLEAN_1607481576423_H
#include "trimesh2/TriMesh.h"

    enum cxBoolean_operation_type {
        CX_UNION = 0, CX_INTERSECTION = 1,
        CX_TM1_MINUS_TM2 = 2, CX_TM2_MINUS_TM1 = 3, CX_NONE
    };
    trimesh::TriMesh* cxBooleanOperateMeshObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2, cxBoolean_operation_type typeindex);

#endif // MMESH_BOOLEAN_1607481576423_H