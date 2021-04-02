#include "Clustering.h"
#include "trimesh2/Vec.h"
#ifdef CX_BOOST_CLUSTER
#include "boost/geometry/index/rtree.hpp"
#include <boost/geometry.hpp>
namespace mmesh 
{
namespace bg = boost::geometry;
namespace bgm = bg::model;
namespace bgi = bg::index;
typedef bgm::point<float, 3, bg::cs::cartesian > bstPoint;
typedef std::pair<bstPoint, unsigned> PointIndexEl;
using Index3D = bgi::rtree< PointIndexEl, bgi::rstar<16,4> /* ? */ >;

// Clustering a set of points by the given distance.
ClusteredPoints cluster(const std::vector<unsigned>& indices,
    std::function< bstPoint(unsigned)> pointfn,
    double dist,
    unsigned max_points);


ClusteredPoints cluster(
    const std::vector<unsigned>& indices,
    std::function< bstPoint(unsigned)> pointfn,
    std::function<bool(const PointIndexEl&, const PointIndexEl&)> predicate,
    unsigned max_points);

// This function returns the position of the centroid in the input 'clust'
// vector of point indices.
template<class DistFn, class PointFn>
long cluster_centroid(const ClusterEl& clust, PointFn pointfn, DistFn df)
{
    switch (clust.size()) {
    case 0: /* empty cluster */ return -1;
    case 1: /* only one element */ return 0;
    case 2: /* if two elements, there is no center */ return 0;
    default:;
    }

    // The function works by calculating for each point the average distance
    // from all the other points in the cluster. We create a selector bitmask of
    // the same size as the cluster. The bitmask will have two true bits and
    // false bits for the rest of items and we will loop through all the
    // permutations of the bitmask (combinations of two points). Get the
    // distance for the two points and add the distance to the averages.
    // The point with the smallest average than wins.

    // The complexity should be O(n^2) but we will mostly apply this function
    // for small clusters only (cca 3 elements)

    std::vector<bool> sel(clust.size(), false);   // create full zero bitmask
    std::fill(sel.end() - 2, sel.end(), true);    // insert the two ones
    std::vector<double> avgs(clust.size(), 0.0);  // store the average distances

    do {
        std::array<size_t, 2> idx;
        for (size_t i = 0, j = 0; i < clust.size(); i++)
            if (sel[i]) idx[j++] = i;

        double d = df(pointfn(clust[idx[0]]),
            pointfn(clust[idx[1]]));

        // add the distance to the sums for both associated points
        for (auto i : idx) avgs[i] += d;

        // now continue with the next permutation of the bitmask with two 1s
    } while (std::next_permutation(sel.begin(), sel.end()));

    // Divide by point size in the cluster to get the average (may be redundant)
    for (auto& a : avgs) a /= clust.size();

    // get the lowest average distance and return the index
    auto minit = std::min_element(avgs.begin(), avgs.end());
    return long(minit - avgs.begin());
}
bool cmp_ptidx_elements(const PointIndexEl& e1, const PointIndexEl& e2)
{
    return e1.second < e2.second;
};

ClusteredPoints cluster(Index3D &sindex,
                        unsigned max_points,
                        std::function<std::vector<PointIndexEl>(
                            const Index3D &, const PointIndexEl &)> qfn)
{
    using Elems = std::vector<PointIndexEl>;

    // Recursive function for visiting all the points in a given distance to
    // each other
    std::function<void(Elems&, Elems&)> group =
        [&sindex, &group, max_points, qfn](Elems& pts, Elems& cluster)
    {
        for(auto& p : pts) {
            std::vector<PointIndexEl> tmp = qfn(sindex, p);

            std::sort(tmp.begin(), tmp.end(), cmp_ptidx_elements);

            Elems newpts;
            std::set_difference(tmp.begin(), tmp.end(),
                                cluster.begin(), cluster.end(),
                                std::back_inserter(newpts), cmp_ptidx_elements);

            int c = max_points && newpts.size() + cluster.size() > max_points?
                        int(max_points - cluster.size()) : int(newpts.size());

            cluster.insert(cluster.end(), newpts.begin(), newpts.begin() + c);
            std::sort(cluster.begin(), cluster.end(), cmp_ptidx_elements);

            if(!newpts.empty() && (!max_points || cluster.size() < max_points))
                group(newpts, cluster);
        }
    };

    std::vector<Elems> clusters;
    for(auto it = sindex.begin(); it != sindex.end();)
    {
        Elems cluster = {};
        Elems pts = {*it};
        group(pts, cluster);
    
        for(auto& c : cluster) sindex.remove(c);
        it = sindex.begin();
    
        clusters.emplace_back(cluster);
    }

    ClusteredPoints result;
    for(auto& cluster : clusters) {
        result.emplace_back();
        for(auto c : cluster) result.back().emplace_back(c.second);
    }

    return result;
}

std::vector<PointIndexEl> distance_queryfn(const Index3D& sindex,
                                           const PointIndexEl& p,
                                           double dist,
                                           unsigned max_points)
{
    std::vector<PointIndexEl> tmp; tmp.reserve(max_points);
    sindex.query(
        bgi::nearest(p.first, max_points),
        std::back_inserter(tmp)
        );

    for(auto it = tmp.begin(); it < tmp.end(); ++it)
        if(bg::distance(p.first ,it->first) > dist) it = tmp.erase(it);

    return tmp;
}


// Clustering a set of points by the given criteria
ClusteredPoints cluster(
    const std::vector<unsigned>& indices,
    std::function<bstPoint(unsigned)> pointfn,
    double dist,
    unsigned max_points)
{
    // A spatial index for querying the nearest points
    Index3D sindex;

    // Build the index
    for(auto idx : indices) 
        sindex.insert( std::make_pair(pointfn(idx), idx));

    return cluster(sindex, max_points,
                   [dist, max_points](const Index3D& sidx, const PointIndexEl& p)
                   {
                       return distance_queryfn(sidx, p, dist, max_points);
                   });
}

// Clustering a set of points by the given criteria
ClusteredPoints cluster(
    const std::vector<unsigned>& indices,
    std::function<bstPoint(unsigned)> pointfn,
    std::function<bool(const PointIndexEl&, const PointIndexEl&)> predicate,
    unsigned max_points)
{
    // A spatial index for querying the nearest points
    Index3D sindex;

    // Build the index
    for(auto idx : indices) sindex.insert( std::make_pair(pointfn(idx), idx));

    return cluster(sindex, max_points,
                   [max_points, predicate](const Index3D& sidx, const PointIndexEl& p)
                   {
                       std::vector<PointIndexEl> tmp; tmp.reserve(max_points);
                       sidx.query(bgi::satisfies([p, predicate](const PointIndexEl& e){
                                      return predicate(p, e);
                                  }), std::back_inserter(tmp));
                       return tmp;
                   });
}

ClusteredPoints cluster(std::vector< trimesh::vec3>& pts, double dist, unsigned max_points)
{
    // A spatial index for querying the nearest points
    Index3D sindex;
    // Build the index
    for (unsigned i = 0; i < pts.size(); i++)
    {
        bstPoint pt(pts[i].x,pts[i].y,pts[i].z);
        sindex.insert(std::make_pair(pt, i));

    }

    return cluster(sindex, max_points,
                   [dist, max_points](const Index3D& sidx, const PointIndexEl& p)
                   {
                       return distance_queryfn(sidx, p, dist, max_points);
                   });
}
}
#endif
