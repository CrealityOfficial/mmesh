#ifndef __EXTRACT_SUPPORT_FACE_H__
#define __EXTRACT_SUPPORT_FACE_H__
#include "trimesh2/TriMesh.h"

namespace extractSupportInfor
{
    typedef struct SUPPORT_FACE_CONFIG
    {
        float ThresAngle;
        float faceArea;
    }SupportFaceConf;
    typedef struct SUPPORT_LINE_CONFIG
    {
        float ThresAngle;
        float ThresAngleBetweenFace;
        float lineLength;
    }SupportLineConf;
    typedef struct SUPPORT_SINGLE_POINT_CONFIG
    {
        float ThresAngleBetweenFace;
        float ThresAngle;
    }SupportSinglePtConf;

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
        std::vector<std::vector<trimesh::vec3>> ClusterSupportPoint(std::vector<trimesh::vec3>& points);
    public:
        SupportFaceConf m_faceCfg;
        SupportLineConf m_lineCfg;
        SupportSinglePtConf m_singlePtCfg;

    };

}

#endif // 