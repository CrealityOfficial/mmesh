#include "clusterPoint.h"

#if defined(WIN32) && defined(USE_CGAL)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/cluster_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/grid_simplify_point_set.h>

#include <CGAL/Random.h>
#include <CGAL/Real_timer.h>
#endif

#include <fstream>

namespace ClusterPoint
{
#if defined(WIN32) && defined(USE_CGAL)
    #define GRID_SIMPLITFY_CELL_SIZE        0.1
    #define CLUSTER_SPACE_SZIE              0.5
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
#endif
    std::vector<std::vector<trimesh::vec3>>  ClusterAllPoints(std::vector<trimesh::vec3>& points)
    {
        std::vector<std::vector<trimesh::vec3>> pointsout;
#if defined(WIN32) && defined(USE_CGAL)
        CGAL::Real_timer t;
        Point_set pointset;
        
        ClusterAddPoints(points, pointset);
        std::cout<<"before gird_simplify size=="<<pointset.size()<<std::endl;
        t.start();
        pointset.remove(CGAL::grid_simplify_point_set(pointset, GRID_SIMPLITFY_CELL_SIZE),
            pointset.end());
        pointset.collect_garbage();

         //{
         //    pointsout.resize(1);
         //    Point_set::Point_map testpmap = pointset.point_map();
         //    for (Point_set::iterator it = pointset.begin(); it != pointset.end(); ++it)
         //    {

         //        Point temp = get(testpmap, *it);
         //        /* testmaptemp.push_back(   (temp);*/
         //        pointsout[0].emplace_back(trimesh::vec3(temp.x(), temp.y(), temp.z()));

         //    }
         //}

         std::cout<<"end gird_simplify size=="<<pointset.size()<<std::endl;
         std::cerr << "grid_simplify_point_set time==" << t.time() << " seconds" << std::endl;



       // Add a cluster map
        Point_set::Property_map<int> cluster_map = pointset.add_property_map<int>("cluster", -1).first;

        // Compute average spacing
        //double spacing = CGAL::compute_average_spacing<CGAL::Parallel_if_available_tag>(points, 12);

        // Adjacencies stored in vector
        std::vector<std::pair<std::size_t, std::size_t> > adjacencies;
        CGAL::Identity_property_map<Kernel::Point_3> pmap;
        // Compute clusters
        t.reset();
        std::size_t nb_clusters
            = CGAL::cluster_point_set(pointset, cluster_map,
                pointset.parameters().neighbor_radius(CLUSTER_SPACE_SZIE).
                adjacencies(std::back_inserter(adjacencies)));
        t.stop();
        std::cerr << "Found " << nb_clusters << " clusters with " << adjacencies.size()
            << " adjacencies in " << t.time() << " seconds" << std::endl;
        pointsout.resize(nb_clusters);

#if 1
        Point_set::Point_map testpmap=pointset.point_map();
        for (Point_set::iterator it = pointset.begin(); it != pointset.end(); ++it)
        {
            
           int clusterindex = get(cluster_map, *it);
           Point temp=get(testpmap, *it);
          /* testmaptemp.push_back(   (temp);*/
           pointsout[clusterindex].emplace_back(trimesh::vec3(temp.x(), temp.y(), temp.z()));

        }
#endif
#endif
        // Output a colored PLY file
#if 0
        for (Point_set::Index idx : pointset)
        {
           // Point_set::const_iterator tesp = pointset.begin();
            // One color per cluster
            //CGAL::Random rand(cluster_map[idx]);
            std::cout << "cluster_map===" << cluster_map[idx] << std::endl;
        }
        for (std::pair<std::size_t, std::size_t> itindex : adjacencies)
        {
            std::cout << "adjacencies(first,second)===" << itindex.first <<","<< itindex.second << std::endl;
        }
#endif


        return pointsout;
    }
}