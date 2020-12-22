#ifndef MMESH_BOOLEAN_1607481576423_H
#define MMESH_BOOLEAN_1607481576423_H
#include "trimesh2/TriMesh.h"

    enum cxSimplify_operation_type {
        cxEDGE_LENGTH = 0,cxEDGE_NUMBERS = 1,
        cxEDGE_RATIO = 2, cxSIMPLIFY_NONE
    };
    trimesh::TriMesh* cxSimplifyOperateMeshObj(trimesh::TriMesh* meshObj, cxSimplify_operation_type typeindex);
    void cxSimplifySetTypeVaue(void* setvalue, cxSimplify_operation_type typeindex);

#endif // MMESH_BOOLEAN_1607481576423_H