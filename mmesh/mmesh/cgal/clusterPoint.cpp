#include "clusterPoint.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/cluster_point_set.h>
#include <CGAL/compute_average_spacing.h>

#include <CGAL/Random.h>
#include <CGAL/Real_timer.h>
#include <fstream>

namespace ClusterPoint
{
    typedef CGAL::Simple_cartesian<float>	Kernel;
    typedef  Kernel::Point_3                       Point;
    typedef  CGAL::Point_set_3<Point>              Point_set;

    void ClusterAddPoints(std::vector<trimesh::vec3>& points, Point_set& pointset)
    {
        for (trimesh::vec3 pointtmp:points)
        {
            pointset.insert(Point(pointtmp.x, pointtmp.y, pointtmp.z));
        }
    }
    std::vector<trimesh::vec3>  ClusterAllPoints(std::vector<trimesh::vec3>& pointsF, std::vector<trimesh::vec3>& pointsL, std::vector<trimesh::vec3>& pointsP)
    {
        Point_set pointset;
        std::vector<trimesh::vec3> pointsout;
        ClusterAddPoints(pointsF, pointset);
        ClusterAddPoints(pointsL, pointset);
        ClusterAddPoints(pointsP, pointset);
        // Add a cluster map
        Point_set::Property_map<int> cluster_map = pointset.add_property_map<int>("cluster", -1).first;

        // Compute average spacing
        //double spacing = CGAL::compute_average_spacing<CGAL::Parallel_if_available_tag>(points, 12);
        double spacing =1.0;
       std::cerr << "Spacing = " << spacing << std::endl;

        // Adjacencies stored in vector
        std::vector<std::pair<std::size_t, std::size_t> > adjacencies;

        // Compute clusters
        CGAL::Real_timer t;
        t.start();
        std::size_t nb_clusters
            = CGAL::cluster_point_set(pointset, cluster_map,
                pointset.parameters().neighbor_radius(spacing).
                adjacencies(std::back_inserter(adjacencies)));
        t.stop();
        std::cerr << "Found " << nb_clusters << " clusters with " << adjacencies.size()
            << " adjacencies in " << t.time() << " seconds" << std::endl;

        // Output a colored PLY file
        for (Point_set::Index idx : pointset)
        {
            // One color per cluster
            CGAL::Random rand(cluster_map[idx]);
        }


        return pointsout;
    }
}