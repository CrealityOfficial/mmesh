#ifndef __CLUSTER_POINT_H__
#define __CLUSTER_POINT_H__

#include "trimesh2/TriMesh.h"
namespace ClusterPoint
{

    std::vector<std::vector<trimesh::vec3>> ClusterAllPoints(std::vector<trimesh::vec3>& points);
}
#endif

