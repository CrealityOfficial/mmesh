#include "extractSupportInfor.h"
#include "clusterPoint.h"
#include "compute_normals_sm.h"
#include "AABBtreeModle.h"

#include <CGAL/Surface_mesh.h>
#include <CGAL/Timer.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/subdivision_method_3.h>
#include <boost/lexical_cast.hpp>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Subdivision_method_3/subdivision_masks_3.h>
using namespace CGAL;
namespace params = CGAL::parameters;
static bool subdivideloopflg = false;

namespace extractSupportInfor
{

typedef CGAL::AABB_face_graph_triangle_primitive<ComputeNormalsSM::Surface_mesh> Primitive;
typedef CGAL::AABB_traits<ComputeNormalsSM::K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef ComputeNormalsSM::K::Ray_3 Ray;

typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;
typedef boost::optional<Tree::Primitive_id> Ray_faceIntersect;

typedef ComputeNormalsSM::Surface_mesh::Property_map<ComputeNormalsSM::face_descriptor, bool> FaceSupportMap;
typedef ComputeNormalsSM::Surface_mesh::Property_map<ComputeNormalsSM::halfedge_descriptor, bool> EdgeSupportMap;
typedef ComputeNormalsSM::Surface_mesh::Property_map<ComputeNormalsSM::vertex_descriptor, bool> VertexSupportMap;

/////////////////////////
    typedef struct SURFACE_MESH_INFOR
    {
        ComputeNormalsSM::Surface_mesh* surfaceMeshPtr;
        ComputeNormalsSM::fnormalsMap* fnormalsPtr;
        FaceSupportMap faceSupportMap;
        EdgeSupportMap edgeSupportMap;
        VertexSupportMap vertexSupportMap;

    }surfaceMeshInfor;
    struct Skip {
        ComputeNormalsSM::face_descriptor fd;

        Skip(const ComputeNormalsSM::face_descriptor fd)
            : fd(fd)
        {}

        bool operator()(const ComputeNormalsSM::face_descriptor& t) const
        {
            if (t == fd) {
                std::cerr << "ignore face_descriptor===" << t << std::endl;
            };
            return(t == fd);
        }

    };
    ////////////////////////////////
    template <class Poly>
    class WLoop_mask_3 {
        typedef Poly                                         PolygonMesh;

        typedef typename boost::graph_traits<PolygonMesh>::vertex_descriptor   vertex_descriptor;
        typedef typename boost::graph_traits<PolygonMesh>::halfedge_descriptor halfedge_descriptor;
        typedef typename boost::graph_traits<PolygonMesh>::face_descriptor      face_descriptor;

        typedef typename boost::property_map<PolygonMesh, CGAL::vertex_point_t>::type Vertex_pmap;
        typedef typename boost::property_traits<Vertex_pmap>::value_type Point;
        typedef typename boost::property_traits<Vertex_pmap>::reference Point_ref;
    public:
        PolygonMesh& pmesh;
        Vertex_pmap vpm;
        bool m_continue;

    public:
        WLoop_mask_3(PolygonMesh& pmesh)
            : pmesh(pmesh), vpm(get(CGAL::vertex_point, pmesh))
        {}

        void edge_node(halfedge_descriptor hd, Point& pt) {

            Point_ref p1 = get(vpm, target(hd, pmesh));
            Point_ref p2 = get(vpm, target(opposite(hd, pmesh), pmesh));

            //pt = Point((p1[0] + p2[0]) / 2,
            //    (p1[1] + p2[1]) / 2 ,
            //    (p1[2] + p2[2]) / 2);
            ComputeNormalsSM::K::FT face_area = CGAL::Polygon_mesh_processing::face_area(face(hd, pmesh), pmesh);
            if (face_area >0.01)
            {
            pt = Point((p1[0] + p2[0]) / 2,
               (p1[1] + p2[1]) / 2,
                (p1[2] + p2[2]) / 2);
            //    pt = CGAL::centroid(pmesh.point(source(hd, pmesh)),
            //        pmesh.point(target(hd, pmesh)),
            //        pmesh.point(target(next(hd, pmesh), pmesh)));

            subdivideloopflg = true;

            }
            else
            {
                pt = p1;
            }
            //pt = p1;
        }
        void vertex_node(vertex_descriptor vd, Point& pt) {
            Point_ref p1 = get(vpm, vd);
            pt = Point(p1[0] , p1[1], p1[2]);

            //halfedge_descriptor hd=halfedge(vd, pmesh);
            //pt = CGAL::centroid(pmesh.point(source(hd, pmesh)),
            //    pmesh.point(target(hd, pmesh)),
            //    pmesh.point(target(next(hd, pmesh), pmesh)));

        }

        void border_node(halfedge_descriptor hd, Point& ept, Point& vpt) {
            Point_ref ep1 = get(vpm, target(hd, pmesh));
            Point_ref ep2 = get(vpm, target(opposite(hd, pmesh), pmesh));
            ComputeNormalsSM::K::FT face_area = CGAL::Polygon_mesh_processing::face_area(face(hd, pmesh), pmesh);
            if (face_area > 0.01)
            {
                ept = Point((ep1[0] + ep2[0]) / 2, (ep1[1] + ep2[1]) / 2, (ep1[2] + ep2[2]) / 2);
                vpt = Point(ep1[0], ep1[1], ep1[2]);
                subdivideloopflg = true;

            }
            else
            {
                ept = Point(ep2[0], ep2[1], ep2[2]);
                vpt = Point(ep1[0], ep1[1], ep1[2]);

            }
        }

    };


    /////////////////////////////////////////////////
    #define ZNORMAL_UP ComputeNormalsSM::Vector(0.0,0.0,1.0)
    #define ZNORMAL_DOWN ComputeNormalsSM::Vector(0.0,0.0,-1.0)

    # define PI          3.1415927f
    static Tree* gTreeObjPtr = NULL;

    static surfaceMeshInfor* gSMinforPtr = NULL;
    ////////////////////////////////////////////////////////////////
    void getOverhangface(surfaceMeshInfor* smInforPtr,  std::vector<trimesh::vec3>& pointsOut)
    {
        int index = 0;
        CGAL::Timer t;
        t.start();
        if (smInforPtr == NULL)
        {
            std::cout << "should init surfaceMeshInfor before" << std::endl;
            return;
        }
        ComputeNormalsSM::Surface_mesh& mesh = *smInforPtr->surfaceMeshPtr;
        ComputeNormalsSM::fnormalsMap& fnormals = *smInforPtr->fnormalsPtr;
        std::vector<ComputeNormalsSM::face_descriptor> facesDescriptorV;

        for (ComputeNormalsSM::face_descriptor fd : faces(mesh))
        {


            // std::cout <<"gSMinforPtr->fnormals==="<< (*gSMinforPtr->fnormals)[fd] << std::endl;
            bool faceDownflg = false;
            bool faceUpdflg = false;
            float faceCosValue = ZNORMAL_DOWN * fnormals[fd];
            float faceThresCosValue = std::cos(20 * PI / 180.0);
            faceDownflg = !!((faceCosValue > 0) && (faceCosValue > faceThresCosValue));
            // faceUpdflg = !!((faceCosValue < 0) && (std::abs(faceCosValue) > faceThresCosValue));
            if (faceDownflg || faceUpdflg)// angle >45 relate to z_down
            {
                gSMinforPtr->faceSupportMap[fd] = true;
                ComputeNormalsSM::halfedge_descriptor hd = halfedge(fd, mesh);

                ComputeNormalsSM::Point point0 = mesh.point(target(hd, mesh));
                ComputeNormalsSM::Point point1 = mesh.point(target(next(hd, mesh), mesh));
                ComputeNormalsSM::Point point2 = mesh.point(target(next(next(hd, mesh), mesh), mesh));
                ComputeNormalsSM::K::FT face_area = CGAL::Polygon_mesh_processing::face_area(fd, mesh);

                ComputeNormalsSM::K::FT borderLength = CGAL::Polygon_mesh_processing::face_border_length(hd, mesh);

                if (face_area < 0.12|| borderLength<1.2)//PI*R*R,确保面积大于挤出点面积
                    continue;
#if 1
                pointsOut.emplace_back(trimesh::vec3(point0.x(), point0.y(), point0.z()));
                pointsOut.emplace_back(trimesh::vec3(point1.x(), point1.y(), point1.z()));
                pointsOut.emplace_back(trimesh::vec3(point2.x(), point2.y(), point2.z()));
#if 1
                static bool savetimes = true;
                if (savetimes)
                {
                    savetimes = false;
                    ComputeNormalsSM::Surface_mesh testmesh;
                    ComputeNormalsSM::vertex_descriptor u = testmesh.add_vertex(point0);
                    ComputeNormalsSM::vertex_descriptor v = testmesh.add_vertex(point1);
                    ComputeNormalsSM::vertex_descriptor w = testmesh.add_vertex(point2);
                    ComputeNormalsSM::face_descriptor f = testmesh.add_face(u, v, w);


                    std::cout << "keep_largest_connected_components was successfully computed\n";
                    std::ofstream outputbefore("Subdivisionbefore.off");
                    outputbefore.precision(17);
                    outputbefore << testmesh;
                    outputbefore.close();

                    //Subdivision_method_3::CatmullClark_subdivision(testmesh, CGAL::Subdivision_method_3::parameters::number_of_iterations(1));
                    //CGAL::Subdivision_method_3::PTQ(testmesh, WLoop_mask_3<ComputeNormalsSM::Surface_mesh>(testmesh), CGAL::Subdivision_method_3::parameters::number_of_iterations(1));
                    //CGAL::Subdivision_method_3::Loop_subdivision(testmesh, CGAL::Subdivision_method_3::parameters::vertex_point_map(get(CGAL::vertex_point, testmesh))
                    //    .number_of_iterations(1));
                    //CGAL::Subdivision_method_3::CatmullClark_subdivision(testmesh, CGAL::Subdivision_method_3::parameters::vertex_point_map(get(CGAL::vertex_point, testmesh))
                    //    .number_of_iterations(3));
                    do
                    {
                        //std::cout << "Subdivision_method_3 PTO" << std::endl;
                        WLoop_mask_3<ComputeNormalsSM::Surface_mesh> subloopmask(testmesh);
                        subdivideloopflg = false;
                        CGAL::Subdivision_method_3::PTQ(testmesh, subloopmask, CGAL::Subdivision_method_3::parameters::number_of_iterations(1));
                    } while (subdivideloopflg == true);

                    std::cout << "keep_largest_connected_components was successfully computed\n";
                    std::ofstream output("Subdivision.off");
                    output.precision(17);
                    output << testmesh;
                    output.close();
                }
#endif
#else
                ComputeNormalsSM::Surface_mesh testmesh;
                ComputeNormalsSM::vertex_descriptor u = testmesh.add_vertex(point0);
                ComputeNormalsSM::vertex_descriptor v = testmesh.add_vertex(point1);
                ComputeNormalsSM::vertex_descriptor w = testmesh.add_vertex(point2);
                ComputeNormalsSM::face_descriptor f = testmesh.add_face(u, v, w);

                //Subdivision_method_3::CatmullClark_subdivision(testmesh, CGAL::Subdivision_method_3::parameters::number_of_iterations(1));
                //CGAL::Subdivision_method_3::PTQ(testmesh, WLoop_mask_3<ComputeNormalsSM::Surface_mesh>(testmesh), CGAL::Subdivision_method_3::parameters::number_of_iterations(1));
                do
                {
                    //std::cout << "Subdivision_method_3 PTO" << std::endl;
                    WLoop_mask_3<ComputeNormalsSM::Surface_mesh> subloopmask(testmesh);
                    subdivideloopflg = false;
                    CGAL::Subdivision_method_3::PTQ(testmesh, subloopmask, CGAL::Subdivision_method_3::parameters::number_of_iterations(1));
                } while (subdivideloopflg == true);
                //CGAL::Subdivision_method_3::Loop_subdivision(testmesh, CGAL::Subdivision_method_3::parameters::vertex_point_map(get(CGAL::vertex_point, testmesh))
                //    .number_of_iterations(1));
                //CGAL::Subdivision_method_3::CatmullClark_subdivision(testmesh, CGAL::Subdivision_method_3::parameters::vertex_point_map(get(CGAL::vertex_point, testmesh))
                //    .number_of_iterations(3));

                unsigned int i = 0, end = testmesh.number_of_vertices() + testmesh.number_of_removed_vertices();
                for (i=0; i < end; ++i) 
                {
                    ComputeNormalsSM::vertex_descriptor vh(i);
                    if (testmesh.is_removed(vh))
                        continue;
                    const ComputeNormalsSM::Point& point0 = testmesh.point(vh);
                    pointsOut.emplace_back(trimesh::vec3(point0.x(), point0.y(), point0.z()));

                    //  std::cout << m.point(vh) << ((m.is_removed(vh)) ? "  R\n" : "\n");
                }

#endif

            }
            //else
            //{
            //    getOverhangLineBaseface(gSMinforPtr, fd);

            //}

        }
        std::cout << "get support infor times====" << t.time() << " sec." << std::endl;
        std::cout << "getOverhangface finish" << std::endl;


    }
    //////////////////////////////////////////////////////////////////
    void getOverhangLineBaseface(surfaceMeshInfor* smInforPtr, ComputeNormalsSM::face_descriptor fd, std::vector<trimesh::vec3>& pointsOut)
    {
        if (smInforPtr == NULL)
        {
            std::cout << "should init surfaceMeshInfor before" << std::endl;
            return;
        }
        ComputeNormalsSM::Surface_mesh& mesh = *smInforPtr->surfaceMeshPtr;
        ComputeNormalsSM::fnormalsMap& fnormals = *smInforPtr->fnormalsPtr;
        ComputeNormalsSM::Vector faceNor = fnormals[fd];
        ComputeNormalsSM::halfedge_descriptor hd = halfedge(fd, mesh);

        if (!is_border(hd, mesh))
        {
           ComputeNormalsSM::halfedge_descriptor hOpposite = opposite(hd, mesh);
           if (!is_border(hOpposite, mesh))
           {
               ComputeNormalsSM::face_descriptor fdOpposite = face(hOpposite, mesh);
               ComputeNormalsSM::Vector faceNorOpposite = fnormals[fdOpposite];

               ComputeNormalsSM::Vector totalFaceNor = faceNor + faceNorOpposite;
               float norbetweenAngle = acos(faceNor * faceNorOpposite);
               if (PI - norbetweenAngle > 45.0 * PI / 180.0)//确保悬吊线两个面的夹角小45度，因为两个面本身不是支撑面，当悬吊线足够突出才支撑
                   return;
               if (ZNORMAL_DOWN * totalFaceNor > 0)//两个相邻面面法线之和要向下
               {
                   if (gSMinforPtr->faceSupportMap[fdOpposite] == false)//相邻面也是非支撑面
                   {
                       float faceNorCoseValueDOWN = ZNORMAL_DOWN* faceNor;
                       float faceNorOpCoseValueDOWN = ZNORMAL_DOWN * faceNorOpposite;
                       if (faceNorCoseValueDOWN >0 || faceNorOpCoseValueDOWN >0)//至少有一个三角面的面法线与Z轴向下所成角度小于90度
                       {
                            ComputeNormalsSM::Point point0 = mesh.point(target(hd, mesh));
                           ComputeNormalsSM::Point point1 = mesh.point(source(hd, mesh));
                          //ComputeNormalsSM::halfedge_descriptor nexthd = next(hd, mesh);
                           // ComputeNormalsSM::Point point2 = mesh.point(target(nexthd, mesh));
                           // ComputeNormalsSM::Point pointOp2 = mesh.point(target(hOpposite, mesh));


                          ComputeNormalsSM::K::FT distanceValue         =CGAL::Polygon_mesh_processing::edge_length(hd, mesh);
                          ComputeNormalsSM::K::FT face_area             = CGAL::Polygon_mesh_processing::face_area(fd, mesh);
                          ComputeNormalsSM::K::FT face_areaOpposite     = CGAL::Polygon_mesh_processing::face_area(fdOpposite, mesh);

                          //std::cout << "line face_area ===" << face_area << std::endl;
                          //std::cout << "line face_areaOpposite ===" << face_areaOpposite << std::endl;
                          //if (ComputeNormalsSM::K::Compute_area_3()(point0, point1, point2) < 0.05)
                          //    return;
                         // if (ComputeNormalsSM::K::Compute_area_3()(point0, point1, pointOp2) < 0.05)
                         //     return;
                           //std::cout << "distanceValue===="<< distanceValue <<std::endl;
                          if (face_area < 0.12)
                              return;
                           if (distanceValue < 0.4)
                           {
                               return;
                           }
                           ComputeNormalsSM::Vector lineVector0 = point0 - point1;
                           bool lineAngleAvalible = false;
                           float coselinevalue = lineVector0 * ZNORMAL_UP;
                           float coseAngle = acos(coselinevalue);
                            #if 0
                           bool lineValible0 = !!((coselinevalue > 0) && (coseAngle < (0.25 * PI)));
                           bool lineValible1 = !!((coselinevalue < 0) && ((coseAngle-0.5*PI) < (0.25 * PI)));
                            #else
                           bool lineValible0 = !!((coselinevalue >= 0) && (((90.0 * PI / 180.0 - coseAngle) < (45.0 * PI / 180.0 * PI))));
                           bool lineValible1 = !!((coselinevalue < 0) && ((coseAngle - 90.0 * PI / 180.0) < (45.0 * PI / 180.0 * PI)));
                           //bool lineValible1 = !!((coselinevalue < 0) && ((180.0 * PI / 180.0 - coseAngle) < (20 * PI / 180.0 * PI)));

                            #endif
                           if (lineValible0|| lineValible0)//待支撑线与XY平面夹角小于45度
                           {

                               if (faceNorCoseValueDOWN * faceNorOpCoseValueDOWN < 0)//相邻三角面的面法线一个向上，一个向下，向下角度大
                               {
                                   float angleDown = 0.0;
                                   float angleUp = 0.0;
                                   float faceNorCoseValueUP = ZNORMAL_UP * faceNor;
                                   float faceNorOpCoseValueUP = ZNORMAL_UP * faceNorOpposite;

                                  // if (faceNorCoseValueUP < 0)
                                  // {
                                       //(angle1-90)-(90-angle2)>0
                                       if ((acos(faceNorCoseValueUP) + acos(faceNorOpCoseValueUP)-180.0*PI/180.0) > 0)
                                       {
                                           gSMinforPtr->edgeSupportMap[hd] = true;
                                           pointsOut.emplace_back(trimesh::vec3(point0.x(), point0.y(), point0.z()));
                                           pointsOut.emplace_back(trimesh::vec3(point1.x(), point1.y(), point1.z()));
                                       }
                                  // }
                                 //  else
                                 //  {
                                   //    if (acos(faceNorOpCoseValueUP)-(acos(faceNorCoseValueUP)) > 0)
                                   //    {
                                   //        gSMinforPtr->edgeSupportMap[hd] = true;
                                   //        pointsOut.emplace_back(trimesh::vec3(point0.x(), point0.y(), point0.z()));
                                   //        pointsOut.emplace_back(trimesh::vec3(point1.x(), point1.y(), point1.z()));
                                   //    }

                                   //}
                               }
                               else
                               {
                                   gSMinforPtr->edgeSupportMap[hd] = true;
                                   pointsOut.emplace_back(trimesh::vec3(point0.x(), point0.y(), point0.z()));
                                   pointsOut.emplace_back(trimesh::vec3(point1.x(), point1.y(), point1.z()));

                               }
                           }

                       }
                   }
               }
           }
           else// if the edge is border
           {
               std::cout << "the opposite edge is border" << std::endl;
           }
        }
        else// if the edge is border
        {
               std::cout << "the  edge is border" << std::endl;
        }
       // std::cout << "getOverhangLine finish" << std::endl;

    }
    void getOverhangSinglePoint(surfaceMeshInfor* smInforPtr, std::vector<trimesh::vec3>& pointsOut)
    {
        if (smInforPtr == NULL)
        {
            std::cout << "should init surfaceMeshInfor before" << std::endl;
            return;
        }

        ComputeNormalsSM::Surface_mesh& mesh = *smInforPtr->surfaceMeshPtr;
        ComputeNormalsSM::fnormalsMap& fnormals = *smInforPtr->fnormalsPtr;
        FaceSupportMap& faceSupportMap = smInforPtr->faceSupportMap;
        EdgeSupportMap& edgeSupportMap = smInforPtr->edgeSupportMap;
        VertexSupportMap& vertexSupportMap = smInforPtr->vertexSupportMap;


        for (const ComputeNormalsSM::vertex_descriptor vd : vertices(mesh))
        {
            ComputeNormalsSM::halfedge_descriptor hd = halfedge(vd, mesh);

            if (hd == ComputeNormalsSM::Surface_mesh::null_halfedge()) {
                return;
            }
            if (is_border_edge(hd, mesh))
                return;
            ComputeNormalsSM::face_descriptor fd = face(hd, mesh);
            ComputeNormalsSM::Vector FaceNormals = fnormals[fd];
        //if ((faceSupportMap[fd] == true)||(edgeSupportMap[hd] == true))
            //{
            //    std::cout << "faceSupportMap  ==" << faceSupportMap[fd] << std::endl;
            //    std::cout << "edgeSupportMap point ==" << edgeSupportMap[hd] << std::endl;

            //}

            if ((faceSupportMap[fd] == false) && (edgeSupportMap[hd] == false))
            {
                ComputeNormalsSM::halfedge_descriptor hdtemp = hd;
                ComputeNormalsSM::halfedge_descriptor done = hdtemp;
                ComputeNormalsSM::Point lowpos = mesh.point(vd);
                bool lowposflg = true;
                std::vector<ComputeNormalsSM::face_descriptor>  nearfdV;
                do {
                    ComputeNormalsSM::halfedge_descriptor nexthd = next(hdtemp, mesh);
                    ComputeNormalsSM::face_descriptor nearfd = face(nexthd, mesh);
                  
                    ComputeNormalsSM::Vector nearFaceNormals = fnormals[nearfd];
                    float norbetweenAngle = acos(FaceNormals * nearFaceNormals);

                    ComputeNormalsSM::Point lowpostemp = mesh.point(target(nexthd, mesh));
                    nearfdV.emplace_back(nearfd);
                    /*
                    * 1、相邻边的点大于该悬吊点值，
                    * 2、相领面法线向下
                    * 3、确保悬吊线两个面的夹角小45度，因为两个面本身不是支撑面，当悬吊点足够突出才支撑
                   */
                    if (lowpos.z() > lowpostemp.z()|| 
                        nearFaceNormals* ZNORMAL_DOWN>0||
                        (PI - norbetweenAngle > 45.0 * PI / 180.0)
                        )
                    {
                        lowposflg = false;
                        break;
                    }

                    hdtemp = opposite(nexthd, mesh);
                } while (hdtemp != done);
                if (lowposflg == true)
                {
                    ComputeNormalsSM::Point lowpostemp(lowpos.x(), lowpos.y(), lowpos.z() + 0.001);
                    Ray ray(lowpostemp,ZNORMAL_DOWN);
                    Skip skip(fd);
                    std::cout<<"*********************************number_of_intersected_primitives====="<< gTreeObjPtr->number_of_intersected_primitives(ray)<<std::endl;
                     Ray_faceIntersect intersection = gTreeObjPtr->first_intersected_primitive(ray, skip);
                   if (intersection)
                    {
                        ComputeNormalsSM::face_descriptor tempfd = intersection.value();
                        //for (ComputeNormalsSM::face_descriptor faceindex : nearfdV)
                        //{
                        //    if (faceindex == fd)
                        //        return;
                        //}
                        if (fnormals[tempfd] * ZNORMAL_DOWN < 0)//第一个相交面的法线向上
                        {
                            ComputeNormalsSM::Point point0 = mesh.point(target(halfedge(tempfd, mesh),mesh));
                            std::cout << "fd, tempfd ==" << fd<<","<< tempfd << std::endl;
                            std::cout << "single point ==" << lowpos << std::endl;
                            std::cout << "face point   ==" << point0 << std::endl;
                            std::cout << "single point facefnormals==" << fnormals[tempfd] << std::endl;
                            vertexSupportMap[vd] = true;
                            pointsOut.emplace_back((trimesh::vec3(lowpos.x(), lowpos.y(), lowpos.z())));
                        }
                    }
                   else
                   {
                            vertexSupportMap[vd] = true;
                            pointsOut.emplace_back((trimesh::vec3(lowpos.x(), lowpos.y(), lowpos.z())));

                   }
                }

            }

        }
        //std::cout << "getOverhangPoint finish" << std::endl;

    }

    void trimesh2cgal(const trimesh::TriMesh& mesh, ComputeNormalsSM::Surface_mesh& surfaceMesh)
    {
        int pointSize = (int)mesh.vertices.size();
        int facesSize = (int)mesh.faces.size();
        if (pointSize < 3 || facesSize < 1)
            return;
        for (int i = 0; i < pointSize; i++)
        {
            const trimesh::vec3& v = mesh.vertices.at(i);
            surfaceMesh.add_vertex(ComputeNormalsSM::K::Point_3(v.x, v.y, v.z));

        }

        for (int i = 0; i < facesSize; i++)
        {
            const trimesh::TriMesh::Face& f = mesh.faces.at(i);
            ComputeNormalsSM::vertex_descriptor vh0(f[0]);
            ComputeNormalsSM::vertex_descriptor vh1(f[1]);
            ComputeNormalsSM::vertex_descriptor vh2(f[2]);
            surfaceMesh.add_face(vh0, vh1, vh2);
        }
        ComputeNormalsSM::Surface_mesh mesh_cpy = surfaceMesh;


        ComputeNormalsSM::Surface_mesh::Property_map< ComputeNormalsSM::face_descriptor, std::size_t> fccmap;
        fccmap = surfaceMesh.add_property_map<ComputeNormalsSM::face_descriptor, std::size_t>("f:CC").first;
        std::size_t num = CGAL::Polygon_mesh_processing::connected_components(surfaceMesh, fccmap);
        if (num > 1)
        {
            std::cout << "connected_components num==" << num << std::endl;

            CGAL::Polygon_mesh_processing::keep_largest_connected_components(surfaceMesh, 1);
        }
        #if 0
        std::cout << "keep_largest_connected_components was successfully computed\n";
        std::ofstream output("union.off");
        output.precision(17);
        output << surfaceMesh;
        output.close();
        #endif

       // CGAL::Polygon_mesh_processing::remove_degenerate_edges(surfaceMesh, CGAL::Polygon_mesh_processing::parameters::all_default());
      //  CGAL::Polygon_mesh_processing::remove_degenerate_edges(surfaceMesh);
        
        /* all_removed = */ CGAL::Polygon_mesh_processing::remove_degenerate_faces(surfaceMesh, CGAL::Polygon_mesh_processing::parameters::all_default());
        // assert(all_removed);

    }

    extractSupportInfor::extractSupportInfor()
    {
    }
    extractSupportInfor::~extractSupportInfor()
    {
        if (gSMinforPtr)
            delete gSMinforPtr;
        gSMinforPtr = NULL;
    }
    void extractSupportInfor::setNeedSupportMesh(trimesh::TriMesh* meshObj)
    {
        static ComputeNormalsSM::Surface_mesh surfaceMesh;
        static ComputeNormalsSM::fnormalsMap fnormals;
        trimesh2cgal(*meshObj, surfaceMesh);
       // bool outflg = CGAL::Polygon_mesh_processing::is_outward_oriented(surfaceMesh) ;
       //if (outflg == false)
       // {
       //     std::cout << "is_outward_oriented" << outflg << std::endl;
       //     CGAL::Polygon_mesh_processing::orient(surfaceMesh, CGAL::Polygon_mesh_processing::parameters::outward_orientation(false));

       // }
        ComputeNormalsSM::getFaceNormals(&surfaceMesh, fnormals);

        if (gSMinforPtr)
        {
            delete gSMinforPtr;
        }
        gSMinforPtr = new surfaceMeshInfor;
        gSMinforPtr->surfaceMeshPtr = &surfaceMesh;
        gSMinforPtr->fnormalsPtr = &fnormals;
        gSMinforPtr->faceSupportMap = surfaceMesh.add_property_map<ComputeNormalsSM::face_descriptor, bool>("f:support", false).first;
        gSMinforPtr->edgeSupportMap = surfaceMesh.add_property_map<ComputeNormalsSM::halfedge_descriptor, bool>("e:support", false).first;
        gSMinforPtr->vertexSupportMap = surfaceMesh.add_property_map<ComputeNormalsSM::vertex_descriptor, bool>("v:support", false).first;
    //create AABBTree
       // AABBTreeModle::CreateAABBTree(meshObj, NULL);
       // AABBTreeModle::CreateAABBTree((void *)&surfaceMesh);
        if (gTreeObjPtr != NULL)
        {
            gTreeObjPtr->clear();
            delete gTreeObjPtr;
            gTreeObjPtr = NULL;
        }
        gTreeObjPtr = new Tree(faces(surfaceMesh).first, faces(surfaceMesh).second, surfaceMesh);
            // static Tree tree(faces(surfaceMesh).first, faces(surfaceMesh).second, surfaceMesh);
        gTreeObjPtr->build();

    }


    void extractSupportInfor::getSupportFace(std::vector<trimesh::vec3>& points)
    {
        getOverhangface(gSMinforPtr, points);

    }
    void extractSupportInfor::getSupportLine(std::vector<trimesh::vec3>& points)
    {
        ComputeNormalsSM::Surface_mesh& mesh = *gSMinforPtr->surfaceMeshPtr;

        for (ComputeNormalsSM::face_descriptor fd : faces(mesh))
        {
            if (gSMinforPtr->faceSupportMap[fd] == false)
            {
                getOverhangLineBaseface(gSMinforPtr, fd, points);
            }
        }
    
    }
    void extractSupportInfor::getSupportPoint(std::vector<trimesh::vec3>& points)
    {
        getOverhangSinglePoint(gSMinforPtr, points);
    }
    std::vector<std::vector<trimesh::vec3>>  extractSupportInfor::ClusterSupportPoint(std::vector<trimesh::vec3>& pointsF, std::vector<trimesh::vec3>& pointsL, std::vector<trimesh::vec3>& pointsP)
    {
        return ClusterPoint::ClusterAllPoints(pointsF, pointsL, pointsP);
    
    }


}//end extractSupportInfor