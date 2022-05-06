#include "geometryutil.h"
//#include <QDebug>

namespace mmesh
{

    float GetArea(trimesh::vec3& A, trimesh::vec3& B, trimesh::vec3& C)
    {
        //double ab1 = A.distanceToPoint(B); 
        //double ac1 = A.distanceToPoint(C);
        //double bc1 = B.distanceToPoint(C);

        double ab = trimesh::distance(A, B);
        double ac = trimesh::distance(A, C);
        double bc = trimesh::distance(B, C);

        double p = (ab + bc + ac) / 2;
        double t = p * (p - ab) * (p - ac) * (p - bc);
        if (t < 0)
        {
            t = 0;
        }
        return sqrt(t);
    }


    // Determine whether point P in triangle ABC
    bool PointinTriangle(trimesh::vec3& A, trimesh::vec3& B, trimesh::vec3& C, trimesh::vec3& P)
    {
        double s1 = GetArea(A, B, P);
        double s2 = GetArea(B, C, P);
        double s3 = GetArea(C, A, P);
        double s = GetArea(A, B, C);
        float dis = fabs(s1 + s2 + s3 - s);
        bool b = dis < 0.1;
        return b;
    }
}

