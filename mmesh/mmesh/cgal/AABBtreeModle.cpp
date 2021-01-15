// Author(s): Camille Wormser, Pierre Alliez
// Example of an AABB tree used with indexed triangle set

#include <iostream>
#include <list>
#include <boost/iterator/iterator_adaptor.hpp>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>

#include "AABBtreeModle.h"
namespace AABBTreeModle
{
    typedef CGAL::Simple_cartesian<float> K;
    typedef K::Triangle_3 Triangle3dFace;

    // My own point type:
    struct My_point {
        float x;
        float y;
        float z;
        My_point()
            : x(0), y(0), z(0) {};

        My_point(float _x, float _y, float _z)
            : x(_x), y(_y), z(_z) {}
    };

    // The triangles are stored in a flat array of indices
    // referring to an array of points: three consecutive
    // indices represent a triangle.
    typedef std::vector<size_t>::const_iterator Point_index_iterator;

    // Let us now define the iterator on triangles that the tree needs:
    class Triangle_iterator
        : public boost::iterator_adaptor<
        Triangle_iterator               // Derived
        , Point_index_iterator            // Base
        , boost::use_default              // Value
        , boost::forward_traversal_tag    // CategoryOrTraversal
        >
    {
    public:
        Triangle_iterator()
            : Triangle_iterator::iterator_adaptor_() {}

        explicit Triangle_iterator(Point_index_iterator p)
            : Triangle_iterator::iterator_adaptor_(p) {}

    private:
        friend class boost::iterator_core_access;
        void increment() { this->base_reference() += 3; }
    };


    // The following primitive provides the conversion facilities between
    // my own triangle and point types and the CGAL ones
    struct My_triangle_primitive {
    public:
        typedef Triangle_iterator    Id;

        // the CGAL types returned
        typedef K::Point_3    Point;
        typedef K::Triangle_3 Datum;

        // a static pointer to the vector containing the points
        // is needed to build the triangles on the fly:
        static const std::vector<My_point>* point_container;

    private:
        Id m_it; // this is what the AABB tree stores internally

    public:
        My_triangle_primitive() {} // default constructor needed

        // the following constructor is the one that receives the iterators from the
        // iterator range given as input to the AABB_tree
        My_triangle_primitive(Triangle_iterator a)
            : m_it(a) {}

        Id id() const { return m_it; }

        // on the fly conversion from the internal data to the CGAL types
        Datum datum() const
        {
            Point_index_iterator p_it = m_it.base();
            const My_point& mp = (*point_container)[*p_it];
            Point p(mp.x, mp.y, mp.z);
            ++p_it;
            const My_point& mq = (*point_container)[*p_it];
            Point q(mq.x, mq.y, mq.z);
            ++p_it;
            const My_point& mr = (*point_container)[*p_it];
            Point r(mr.x, mr.y, mr.z);

            return Datum(p, q, r); // assembles triangle from three points
        }

        // one point which must be on the primitive
        Point reference_point() const
        {
            const My_point& mp = (*point_container)[*m_it];
            return Point(mp.x, mp.y, mp.z);
        }
    };


    // types
    typedef CGAL::AABB_traits<K, My_triangle_primitive> My_AABB_traits;
    typedef CGAL::AABB_tree<My_AABB_traits> Tree;
    const std::vector<My_point>* My_triangle_primitive::point_container = 0;
    static Tree* gTreeObjPtr = NULL;

    int CreateAABBTree(const trimesh::TriMesh* meshPtr, const trimesh::fxform *xfPtr)
    {
        // generates point set
        static std::vector<My_point> points;
        static std::vector<size_t> triangles;
       static trimesh::TriMesh* meshPtrBefore = NULL;
        if (meshPtr == NULL)
            return EXIT_FAILURE;
        if (meshPtrBefore == meshPtr)
            return EXIT_SUCCESS;
        int verticeSize = (int)meshPtr->vertices.size();
        meshPtrBefore =(trimesh::TriMesh*) meshPtr;
        points.clear();
        points.resize(verticeSize);
        #ifdef _OPENMP
        #pragma omp parallel for shared(points, meshPtr) num_threads(20)
        #endif
        for (int i = 0; i < verticeSize; i++)
        {
           // printf("OpenMP Thread ID: %d\n", omp_get_thread_num());
            const trimesh::vec3& v = meshPtr->vertices.at(i);
            trimesh::vec3 vt = v;
            if(xfPtr !=NULL)
                vt = (*xfPtr) * v;

            My_point a(vt.x, vt.y, vt.z);
            //points.push_back(a);
            points.at(i)= a;

            //if (i == 0)
            //{
            //    std::cout <<"tree Point"<< vt.x <<" "<< vt.y << " " << vt.z << std::endl;
            //}

        }

        My_triangle_primitive::point_container = &points;
        //points.push_back(a);
        //points.push_back(b);
        //points.push_back(c);
        //points.push_back(d);

        // generates indexed triangle set
        //triangles.push_back(0); triangles.push_back(1); triangles.push_back(2);
        //triangles.push_back(0); triangles.push_back(1); triangles.push_back(3);
        //triangles.push_back(0); triangles.push_back(3); triangles.push_back(2);
        int facesSize = (int)meshPtr->faces.size();
        triangles.clear();
        triangles.reserve(facesSize * 3);
        for (int i = 0; i < facesSize; i++)
        {
            const trimesh::TriMesh::Face& f = meshPtr->faces.at(i);
            triangles.push_back(f[0]);
            triangles.push_back(f[1]);
            triangles.push_back(f[2]);
        }
        // constructs AABB tree
         clearAABBTree();
        //static Tree tree(Triangle_iterator(triangles.begin()),
        //    Triangle_iterator(triangles.end()));
        ////tree.build();
        gTreeObjPtr =new Tree(Triangle_iterator(triangles.begin()),
            Triangle_iterator(triangles.end()));
        gTreeObjPtr->build();


#if 0
        // counts #intersections
        K::Ray_3 ray_query(K::Point_3(0.2, 0.2, 0.2), K::Point_3(0.0, 1.0, 0.0));
        std::cout << gTreeObjPtr->number_of_intersected_primitives(ray_query)
            << " intersections(s) with ray query" << std::endl;

        // computes closest point
        K::Point_3 point_query(2.0, 2.0, 2.0);
        K::Point_3 closest_point = gTreeObjPtr->closest_point(point_query);
        std::cout << "closest point is: " << closest_point << std::endl;

        K::Point_3 a(10, 10, 10);
        K::Point_3 b(20, 20, 20);
        K::Point_3 c(10, 50, 30);

        Triangle3dFace triangle_query(a, b, c);
        gTreeObjPtr->do_intersect(triangle_query);

#endif
        //TriangleIntersect(meshPtr);
        return EXIT_SUCCESS;
    }

    bool TriangleIntersect(const trimesh::TriMesh* meshPtr)
    {
        //Triangle intersections
        int faceSize = meshPtr->faces.size();
        bool retvalue = false;
        int faceIndex = 0;
        if (gTreeObjPtr == NULL)
        {
            std::cout << "Please Create AABBTree" << std::endl;
            return false;
        }
        for(faceIndex=0;faceIndex< faceSize;faceIndex++)
        {
            const trimesh::TriMesh::Face& f = meshPtr->faces.at(faceIndex);
            const trimesh::vec3& v = meshPtr->vertices.at(f[0]);
            const trimesh::vec3& v1 = meshPtr->vertices.at(f[1]);
            const trimesh::vec3& v2 = meshPtr->vertices.at(f[2]);
            K::Point_3 a(v.x, v.y, v.z);
            K::Point_3 b(v1.x, v1.y, v1.z);
            K::Point_3 c(v2.x, v2.y, v2.z);
            if (0)
            {
                std::cout << "triangle_query Point==" << v.x << " " << v.y << " " << v.z<< std::endl;
                std::cout << "triangle_query Point==" << v1.x << " " << v1.y << " " << v1.z << std::endl;
                std::cout << "triangle_query Point==" << v2.x << " " << v2.y << " " << v2.z << std::endl;
            }

            Triangle3dFace triangle_query(a, b, c);
            retvalue = gTreeObjPtr->do_intersect(triangle_query);
            if (retvalue)
            {
                break;
            }

        }
#if 0
        std::cout << "faceSize: " << faceSize << std::endl;
        std::cout << "faceIndex: " << faceIndex << std::endl;
        std::cout << "do_intersect: " << retvalue << std::endl;
        std::cout <<  std::endl;
#endif
      return retvalue;

    }

    bool TriangleFaceIntersect(const trimesh::vec3 *v0, const trimesh::vec3 *v1, const trimesh::vec3 *v2)
    {
        //Triangle intersections
        bool retvalue = false;
        if (gTreeObjPtr == NULL)
        {
            std::cout << "Please Create AABBTree" << std::endl;
            return false;
        }
        K::Point_3 a(v0->x, v0->y, v0->z);
        K::Point_3 b(v1->x, v1->y, v1->z);
        K::Point_3 c(v2->x, v2->y, v2->z);
        if (0)
        {
           std::cout << "triangle_query Point==" << v0->x << " " << v0->y << " " << v0->z << std::endl;
           std::cout << "triangle_query Point==" << v1->x << " " << v1->y << " " << v1->z << std::endl;
           std::cout << "triangle_query Point==" << v2->x << " " << v2->y << " " << v2->z << std::endl;
        }
        Triangle3dFace triangle_query(a, b, c);
        retvalue = gTreeObjPtr->do_intersect(triangle_query);//true intersect,false no intersect
        //std::cout << "do_intersect"<<retvalue << std::endl;
        return retvalue;

    }
    void clearAABBTree()
    {
        if (gTreeObjPtr != NULL)
        {
            gTreeObjPtr->clear();
            delete gTreeObjPtr;
            gTreeObjPtr = NULL;
        }
    }
}
