#ifndef __EXTRACT_SUPPORT_FACE_H__
#define __EXTRACT_SUPPORT_FACE_H__
#include "trimesh2/TriMesh.h"

namespace extractSupportInfor
{
    class extractSupportInfor
    {
    public:
        extractSupportInfor();
        ~extractSupportInfor();
    public:
        void setNeedSupportMesh(trimesh::TriMesh* meshObj);
        void getSupportFace(std::vector<trimesh::vec3>& points);
        void getSupportLine(std::vector<trimesh::vec3>& points);
        void getSupportPoint(std::vector<trimesh::vec3>& points);
        std::vector<std::vector<trimesh::vec3>> ClusterSupportPoint(std::vector<trimesh::vec3>& pointsF, std::vector<trimesh::vec3>& pointsL, std::vector<trimesh::vec3>& pointsP);
    };

}

#endif // 