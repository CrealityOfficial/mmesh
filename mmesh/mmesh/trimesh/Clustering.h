#ifndef CX_BOOST_CLUSTERING_H
#define CX_BOOST_CLUSTERING_H

#include <vector>
#include "trimesh2/TriMesh.h"
namespace mmesh { 

using ClusterEl = std::vector<unsigned>;
using ClusteredPoints = std::vector<ClusterEl>;
ClusteredPoints cluster(std::vector< trimesh::vec3>& points,
    double dist,
    unsigned max_points);
}

#endif // CX_CLUSTERING_H
