/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004-2016                                           \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/
#if __APPLE__
#else
#include <sstream>
#include <string>
#include "mmesh/vcg/trimesh_sampling .h"

#ifdef USE_VCG
#include<vcg/complex/complex.h>

#include<wrap/io_trimesh/import_off.h>
#include<wrap/io_trimesh/export_off.h>

#include<vcg/complex/algorithms/point_sampling.h>
#include<vcg/complex/algorithms/create/platonic.h>
#include<vcg/complex/algorithms/voronoi_remesher.h>
#include <vcg/space/index/grid_static_ptr.h>
#include <wrap/io_trimesh/export_obj.h>

#include<clipper/clipper.hpp>
#include "clipper_path_io.h"
namespace vcg
{
    namespace CX_PoissonAlg {

        using namespace std;
        

        class MyEdge;
        class MyFace;
        class MyVertex;
        struct MyUsedTypes : public UsedTypes<	Use<MyVertex>   ::AsVertexType,
            Use<MyEdge>     ::AsEdgeType,
            Use<MyFace>     ::AsFaceType> {};

        class MyVertex : public Vertex<MyUsedTypes, vcg::vertex::VFAdj, vertex::Qualityf, vertex::Color4b, vertex::Coord3f, vertex::Normal3f, vertex::Mark, vertex::BitFlags  > {};
        //class MyFace : public Face< MyUsedTypes, face::FFAdj, face::Normal3f, face::VertexRef, face::BitFlags > {};

        class MyFace : public Face  <MyUsedTypes, face::FFAdj, vcg::face::VFAdj, face::VertexRef, face::BitFlags, face::Mark, face::Normal3f> {};

        class MyEdge : public Edge<MyUsedTypes> {};
        class MyMesh : public tri::TriMesh< vector<MyVertex>, vector<MyFace>, vector<MyEdge>  > {};

        typedef typename MyMesh::CoordType CoordType;
        typedef typename MyMesh::VertexType     VertexType;
        typedef typename MyMesh::MeshType::ScalarType ScalarType;
        typedef typename VoroEdgeMeshAux::EdgeMeshType EdgeMeshType;

        typedef vcg::GridStaticPtr<MyMesh::FaceType, MyMesh::ScalarType> TriMeshGrid;
        //void borderSamperPointOff(MyMesh* MeshSource, EdgeMeshType* em, ClipperLib::Paths& contourpaths, PoissonAlgCfg* poissonCfgPtr, std::vector<CoordType>& outBorderVertexs);
        double PointsDistace(IntPoint pt1, IntPoint pt2)
        {
            double dx = (double)pt1.X - pt2.X;
            double dy = (double)pt1.Y - pt2.Y;
            double dz =0 ;// (double)pt1.Z - pt2.Z;
            return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
        }

        template <class MeshType>
        void  poissonRadiusSurface(MeshType& baseMesh,
            typename tri::SurfaceSampling<MeshType, tri::MeshSampler<MeshType> >::PoissonDiskParam &pp,
            typename MeshType::ScalarType _poissonRadiusSurface,
            std::vector<typename MeshType::CoordType>& outVertexes)
        {
            typename MeshType::ScalarType poissonRadiusSurface;
            MeshType poissonSurfaceMesh;

            MeshType montecarloSurfaceMesh;
            if (_poissonRadiusSurface == 0) 
                poissonRadiusSurface = baseMesh.bbox.Diag() / 50.0f;
            else 
                poissonRadiusSurface = _poissonRadiusSurface;
            ScalarType meshArea = Stat<MeshType>::ComputeMeshArea(baseMesh);
            int MontecarloSurfSampleNum = 10 * meshArea / (poissonRadiusSurface * poissonRadiusSurface);
            tri::MeshSampler<MeshType> sampler(montecarloSurfaceMesh);
            tri::SurfaceSampling<MeshType, tri::MeshSampler<MeshType> >::Montecarlo(baseMesh, sampler, MontecarloSurfSampleNum);
            montecarloSurfaceMesh.bbox = baseMesh.bbox; // we want the same bounding box
            poissonSurfaceMesh.Clear();
            tri::MeshSampler<MeshType> mps(poissonSurfaceMesh);

            tri::SurfaceSampling<MeshType, tri::MeshSampler<MeshType> >::PoissonDiskPruning(mps, montecarloSurfaceMesh, poissonRadiusSurface, pp);
            vcg::tri::UpdateBounding<MeshType>::Box(poissonSurfaceMesh);

           // printf("Surface Sampling radius %f - montecarlo %ivn - Poisson %ivn\n", poissonRadiusSurface, montecarloSurfaceMesh.vn, poissonSurfaceMesh.vn);
            //VertexConstDataWrapper<MeshType> ww(poissonSurfaceMesh);
            for (int index=0;index< poissonSurfaceMesh.vn; index++)
            {
                outVertexes.emplace_back(poissonSurfaceMesh.vert[index].P());
            }

        }
        bool mainSecond(MyMesh* surfaceMeshPtr, PoissonAlgCfg* poissonCfgPtr, std::vector<CoordType> inVertexes, std::vector<CoordType> inExternVertexes, int sampleNum, std::vector<CoordType>& outVertexes)
        {
            //std::cout << __LINE__ << "  " << __FUNCTION__ << std::endl;
            if (inExternVertexes.size() < 3)
            {
                outVertexes.swap(inExternVertexes);
                return true;
            }
            {
                //----------------------------------------------------------------------
                // Advanced Sample
                // Make a feature dependent Poisson Disk sampling
                MyMesh PoissonEdgeMesh;
                MyMesh MontecarloSurfaceMesh;

                float rad = poissonCfgPtr->userSampleRad*1000;
                //rad = ComputePoissonDiskRadius(*surfaceMeshPtr, sampleNum);

                //tri::io::ExporterOFF<MyMesh>::Save(m, "disc.off");
                tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::SamplingRandomGenerator().initialize(time(0));



                std::vector<CoordType> sampleVec;
                tri::TrivialSampler<MyMesh> mps(sampleVec);
                tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskParam pp;

                sampleVec.clear();
                //tri::io::ExporterOFF<MyMesh>::Save(MontecarloSurfaceMesh, "MontecarloSurfaceMesh.off");

                if (inVertexes.size() )
                {
                    tri::BuildMeshFromCoordVector(PoissonEdgeMesh, inVertexes);
                    pp.preGenMesh = &PoissonEdgeMesh;
                    pp.preGenFlag = true;
                }
                else
                {
                    pp.preGenFlag = false;
                }
                tri::BuildMeshFromCoordVector(MontecarloSurfaceMesh, inExternVertexes);

                tri::SurfaceSampling<MyMesh, tri::MeshSampler<MyMesh> >::PoissonDiskParam pptest;
                pptest.preGenMesh = &PoissonEdgeMesh;
                pptest.preGenFlag = true;

                //poissonRadiusSurface<MyMesh>(*surfaceMeshPtr, pptest, rad, sampleVec);
                //PoissonSamplingTest < MyMesh>(*surfaceMeshPtr, inVertexes, sampleVec, sampleNum, rad);
                //tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskPruning(mps, MontecarloSurfaceMesh, rad, pp);
                tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskPruning(mps, MontecarloSurfaceMesh, rad, pp);
                //tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskPruning(mps, *surfaceMeshPtr, rad, pp);
               // tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskPruningByNumber(mps, *surfaceMeshPtr, sampleNum, rad, pp,0.04,5);
               // tri::PoissonSampling<MyMesh>(*surfaceMeshPtr, sampleVec, sampleNum, rad);
                //tri::PoissonPruning<MyMesh>(*surfaceMeshPtr, sampleVec, rad);
                //tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::Montecarlo(m, mps, sampleNum);
                //tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::MontecarloPoisson(*surfaceMeshPtr, mps, sampleNum);

                
                
                outVertexes.swap(sampleVec);
                //tri::io::ExporterOFF<MyMesh>::Save(PoissonMesh, "PoissonMesh.off");
                //printf("Computed a feature aware poisson disk distribution of %i vertices radius is %6.3f\n", PoissonMesh.VN(), rad);
                return true;
            }
        }
        void borderSamper(MyMesh* surfaceMeshPtr, PoissonAlgCfg* poissonCfgPtr, std::vector<CoordType>& outVertexes)
        {

            typedef typename EdgeMeshType::CoordType EdgeCoord;

            EdgeMeshType em;
            ScalarType meshArea = Stat<MyMesh::MeshType>::ComputeMeshArea(*surfaceMeshPtr);
            ScalarType meshBorderLength = Stat<MyMesh::MeshType>::ComputeBorderLength(*surfaceMeshPtr);
            //if (meshArea < M_PIf * std::pow(m_PoissonAlgCfg.baseSampleRad, 2) *1.5)
            //{
            //    std::cout << "meshArea small" << std::endl;
            //    return ;
            //}
            //if ((meshArea / meshBorderLength) < (poissonCfgPtr->baseSampleRad*1000 / 3))
            //{
            //    std::cout << "meshBorderLength small" << std::endl;
            //    return;
            //}
            auto ExtractMeshBorders = [](MyMesh& mesh, EdgeMeshType& sides)
            {
                RequireFFAdjacency(mesh);

                // clean the edge mesh containing the borders
                sides.Clear();

                // gather into separate vertices lists
                std::vector<std::vector<VertexType*> > edges;
                for (auto fi = mesh.face.begin(); fi != mesh.face.end(); fi++)
                {
                    for (int e = 0; e < fi->VN(); e++)
                    {
                        if (vcg::face::IsBorder(*fi, e))
                        {
                            std::vector<VertexType*> tmp;
                            tmp.push_back(fi->V(e));
                            tmp.push_back(fi->V((e + 1) % fi->VN()));
                            edges.push_back(tmp);
                        }
                    }
                }

                // convert to edge mesh
                for (auto& e : edges)
                {
                    assert(e.size() >= 2);

                    std::vector<typename EdgeMeshType::VertexType*> newVtx;

                    // insert new vertices and store their pointer
                    auto vi = Allocator<EdgeMeshType>::AddVertices(sides, e.size());//???иоив???ж╠?
                    for (const auto& v : e)
                    {
                        vi->ImportData(*v);
                        newVtx.push_back(&(*vi++));
                    }

                    auto ei = Allocator<EdgeMeshType>::AddEdges(sides, e.size() - 1);
                    for (int i = 0; i < static_cast<int>(e.size() - 1); i++)
                    {
                        ei->V(0) = newVtx[i];
                        ei->V(1) = newVtx[i + 1];
                        ei++;
                    }
                }

                Clean<EdgeMeshType>::RemoveDuplicateVertex(sides);
            };
            ExtractMeshBorders(*surfaceMeshPtr, em);

            Allocator<EdgeMeshType>::CompactVertexVector(em);
            Allocator<EdgeMeshType>::CompactEdgeVector(em);
            // split on non manifold vertices of edgemesh
            vcg::tri::Clean<EdgeMeshType>::SelectNonManifoldVertexOnEdgeMesh(em);
            {
                // select also the visited vertices (coming from the non manifold vertices of the whole crease-cut mesh)
                for (auto& v : em.vert)
                {
                    if (v.IsV()) { v.SetS(); }
                }
            }
            const int manifoldSplits = vcg::tri::Clean<EdgeMeshType>::SplitSelectedVertexOnEdgeMesh(em);
            (void)manifoldSplits;

            //std::cout << manifoldSplits << " non-manifold splits" << std::endl;
             if (0)
             {
                 static  int svgindex = 0;
                 svgindex += 1;
                 char edgefilename[128];
                 sprintf(edgefilename, "./SVG/edgeMesh_%d.obj", em.vert.size());
                 tri::io::ExporterOBJ<EdgeMeshType>::Save(em, edgefilename, io::Mask::IOM_EDGEINDEX);
             }


            // Samples vector
            std::vector<EdgeCoord> borderSamples;
            TrivialSampler<EdgeMeshType> ps(borderSamples);
            float rad = poissonCfgPtr->userSampleRad*1000;
            // uniform edge sampling
            UpdateTopology<EdgeMeshType>::EdgeEdge(em);
            SurfaceSampling<EdgeMeshType>::EdgeMeshUniform(em, ps, rad, SurfaceSampling<EdgeMeshType>::Ceil);
            // BuildMeshFromCoordVector(poissonEdgeMesh, borderSamples);
            for (EdgeCoord& pt : borderSamples)
            {
                outVertexes.emplace_back(CoordType(pt.X(), pt.Y(), pt.Z()));
            }

            ////// remove duplicate vertices
            //Clean<MyMesh>::RemoveDuplicateVertex(poissonEdgeMesh, false);
            //Allocator<MyMesh>::CompactVertexVector(poissonEdgeMesh);

            ////// select all vertices (to mark them fixed)
            //UpdateFlags<MyMesh>::VertexSetS(poissonEdgeMesh);

            //tri::io::ExporterOFF<MyMesh>::Save(*meshPtr, "m.off");
            //tri::io::ExporterOFF<MyMesh>::Save(poissonEdgeMesh, "poissonEdgeMesh.off");
        }

        void borderPaths(MyMesh* surfaceMeshPtr, ClipperLib::Paths& outPaths, ClipperLib::Paths& outDiffPaths, EdgeMeshType &em)
        {

            typedef typename EdgeMeshType::CoordType EdgeCoord;
            //std::cout << __LINE__ <<"  "<< __FUNCTION__ << std::endl;
            auto ExtractMeshBorders = [](MyMesh& mesh, EdgeMeshType& sides)
            {
                RequireFFAdjacency(mesh);

                // clean the edge mesh containing the borders
                sides.Clear();

                // gather into separate vertices lists
                std::vector<std::vector<VertexType*> > edges;
                for (auto fi = mesh.face.begin(); fi != mesh.face.end(); fi++)
                {
                    for (int e = 0; e < fi->VN(); e++)
                    {
                        if (vcg::face::IsBorder(*fi, e))
                        {
                            std::vector<VertexType*> tmp;
                            tmp.push_back(fi->V(e));
                            tmp.push_back(fi->V((e + 1) % fi->VN()));
                            edges.push_back(tmp);
                        }
                    }
                }

                // convert to edge mesh
                for (auto& e : edges)
                {
                    assert(e.size() >= 2);

                    std::vector<typename EdgeMeshType::VertexType*> newVtx;

                    // insert new vertices and store their pointer
                    auto vi = Allocator<EdgeMeshType>::AddVertices(sides, e.size());//???иоив???ж╠?
                    for (const auto& v : e)
                    {
                        vi->ImportData(*v);
                        newVtx.push_back(&(*vi++));
                    }

                    auto ei = Allocator<EdgeMeshType>::AddEdges(sides, e.size() - 1);
                    for (int i = 0; i < static_cast<int>(e.size() - 1); i++)
                    {
                        ei->V(0) = newVtx[i];
                        ei->V(1) = newVtx[i + 1];
                        ei++;
                    }
                }

                Clean<EdgeMeshType>::RemoveDuplicateVertex(sides);
            };
            ExtractMeshBorders(*surfaceMeshPtr, em);
            if (0)
            {
                char edgefilename[128];
                char str[25];
#if _WIN32
                itoa(em.vert.size(), str, 10);
#else
                snprintf(str, 25, "%d", (int)em.vert.size());
#endif

                string gFilenamePre = str;
                sprintf(edgefilename, "./SVG/edgeMesh_%d.obj", em.vert.size());
                tri::io::ExporterOBJ<EdgeMeshType>::Save(em, edgefilename, io::Mask::IOM_EDGEINDEX);
            }
            ////////////////extract contour path///////////////////////////
            {
                ClipperLib::Clipper c;
                int  pathnum = 0;
                std::vector<bool> edgeflge(em.edge.size(), false);
                for (size_t edgeStartIndex = 0; edgeStartIndex < em.edge.size(); edgeStartIndex++)
                {
                    ClipperLib::Path subj;

                    EdgeMeshType::EdgeIterator edgeStart = em.edge.begin() + edgeStartIndex;
                    size_t edgeStartIndex0 = tri::Index(em, (*edgeStart).V(0));
                    auto vStart = em.vert[edgeStartIndex0];
                    auto ptStart = vStart.cP();
                    size_t edgeStartIndex1 = tri::Index(em, (*edgeStart).V(1));
                    if (edgeflge[edgeStartIndex] == true)
                        continue;
                    edgeflge[edgeStartIndex] = true;
                    subj << ClipperLib::IntPoint(ptStart.X() , ptStart.Y() , ptStart.Z() );

                    while (edgeStartIndex1 != edgeStartIndex0)
                    {
                        for (size_t edgeLinkIndex = 0; edgeLinkIndex < em.edge.size(); edgeLinkIndex++)
                        {
                            EdgeMeshType::EdgeIterator edgeLink = em.edge.begin() + edgeLinkIndex;
                            size_t edgeLinkIndex0 = tri::Index(em, (*edgeLink).V(0));
                            size_t edgeLinkIndex1 = tri::Index(em, (*edgeLink).V(1));
                            if (edgeflge[edgeLinkIndex] == true)
                            {
                                if (edgeLinkIndex == (em.edge.size() - 1))
                                {
                                    edgeStartIndex1 = edgeStartIndex0;
                                    //edgeflge[edgeLinkIndex] = true;
                                    //subj << ClipperLib::IntPoint(ptStart.X() * 1000, ptStart.Y() * 1000, ptStart.Z() * 1000);

                                }

                                continue;
                            }

                            if (edgeStartIndex1 == edgeLinkIndex0)
                            {
                                edgeflge[edgeLinkIndex] = true;
                                auto v2 = em.vert[edgeLinkIndex0];
                                auto pt2 = v2.cP();
                                //subj << ClipperLib::IntPoint(pt1.X() * 1000, pt1.Y() * 1000, pt1.Z() * 1000);
                                subj << ClipperLib::IntPoint(pt2.X() , pt2.Y() , pt2.Z() );
                                edgeStartIndex1 = edgeLinkIndex1;
                                break;
                            }
                            else if (edgeStartIndex1 == edgeLinkIndex1)
                            {
                                auto v2 = em.vert[edgeLinkIndex1];
                                auto pt2 = v2.cP();
                                //subj << ClipperLib::IntPoint(pt1.X() * 1000, pt1.Y() * 1000, pt1.Z() * 1000);
                                subj << ClipperLib::IntPoint(pt2.X() , pt2.Y(), pt2.Z());
                                edgeStartIndex1 = edgeLinkIndex0;
                                edgeflge[edgeLinkIndex] = true;
                                break;
                            }
                            else
                            {
                                if (edgeLinkIndex == (em.edge.size() - 1))
                                {
                                    edgeStartIndex1 = edgeStartIndex0;
                                    //edgeflge[edgeLinkIndex] = true;
                                    //subj << ClipperLib::IntPoint(ptStart.X() * 1000, ptStart.Y() * 1000, ptStart.Z() * 1000);

                                }
                            }

                        }
                    };
                    if (subj.size() > 3)
                    {
                        outPaths.emplace_back(subj);
                        if (pathnum == 0)
                        {
                            pathnum += 1;
                            c.AddPath(subj, ptSubject, true);

                        }
                        else
                            c.AddPath(subj, ptClip, true);
                    }

                    
                }
                outPaths.clear();
                c.Execute(ctUnion, outPaths, pftEvenOdd);
                c.Execute(ctDifference, outDiffPaths, pftEvenOdd);
                if(0)
                {
                    SVGBuilder svg;
                    svg.style.penWidth = 0.1;
                    svg.style.pft = pftEvenOdd;
                    svg.style.brushClr = 0x1200009C;
                    svg.style.penClr = 0xCCD3D3DA;
                    //svg.AddPaths(subj);
                    //svg.style.brushClr = 0x129C0000;
                    //svg.style.penClr = 0xCCFFA07A;
                    //svg.AddPaths(clip);
                    svg.style.brushClr = 0x6080ff9C;
                    svg.style.penClr = 0xFF003300;
                    svg.style.pft = pftNonZero;

                    svg.AddPaths(outPaths);
                    char str[25];
#if _WIN32
                    itoa(em.vert.size(), str, 10);
#else
                    snprintf(str, 25, "%d", (int)em.vert.size());
#endif
                    string gFilenamePre = str;

                    string filename = "./SVG/contours_" + gFilenamePre + ".svg";
                    svg.SaveToFile(filename, 0.01);
                }

            }
        }
       void borderSamperClipPoint(MyMesh* MeshSource, EdgeMeshType* em, ClipperLib::Paths& contourpaths, PoissonAlgCfg* poissonCfgPtr, std::vector<CoordType>& outContoursVertexs, std::vector<CoordType>& outBorderVertexs)
        {
            bool firstflg = true;
            bool firstContourflg = true;
            ClipperLib::Paths solution;
            TriMeshGrid static_grid;
            float borderSampleOff = poissonCfgPtr->borderSampleOff * 1000;
            float sampleDist = poissonCfgPtr->userSampleRad * 1000;
            static_grid.Set(MeshSource->face.begin(), MeshSource->face.end());
            float maxDist = MeshSource->bbox.max.Z() + 100;

            bool hitflg = false;
            outBorderVertexs.clear();
            //std::cout << __LINE__ << "  " << __FUNCTION__ << "contourpaths.size==" << contourpaths.size() << std::endl;
            //std::cout << __LINE__ << "  " << __FUNCTION__ << "borderSampleOff==" << poissonCfgPtr->borderSampleOff << std::endl;
            if (contourpaths.size() == 0)
                return;
            if (0)
            {
                for (ClipperLib::Path& solutionSub : contourpaths)
                {
                    for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                    {
                        ClipperLib::IntPoint ptNew = solutionSub[ptIndex];

                        outBorderVertexs.emplace_back(trimesh::vec3(ptNew.X, ptNew.Y, ptNew.Z));
                        continue;
                        MyMesh::FaceType* rf;
                        float t;
                        Point3f dir(0.0, 0.0, 1.0);
                        vcg::Ray3f ray;
                        ray.SetOrigin(CoordType(ptNew.X, ptNew.Y, 0.0));
                        ray.SetDirection(dir);
                        rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                        if (rf)
                        {
                            CoordType raypt = ray.Origin() + ray.Direction() * t;
                            outBorderVertexs.emplace_back(raypt);
                        }
                    }
                }
                return;
            }


            do
            {
                ClipperLib::ClipperOffset co;
                std::vector<CoordType> borderVertexs;
                if (firstflg == true)
                {
                    co.AddPaths(contourpaths, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
                    co.Execute(solution, borderSampleOff);
                    if (solution.size() == 0)
                    {
                        co.Clear();
                        borderSampleOff = -poissonCfgPtr->baseSampleRad * 1000;

                        co.AddPaths(contourpaths, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
                        co.Execute(solution, -0.1*1000);
                    }
                    else
                        borderSampleOff = -poissonCfgPtr->userSampleRad * 1000;
                }
                else
                {
                    ClipperLib::Paths solutionNext= solution;

                    if (solution.size() == 0)
                        return;

                    co.AddPaths(solution, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
                    co.Execute(solution, borderSampleOff);
                    if (solution.size() == 0)
                    {
                        co.Clear();
                        borderSampleOff = -poissonCfgPtr->baseSampleRad * 1000;
                        co.AddPaths(solutionNext, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
                        co.Execute(solution, borderSampleOff);
                    }

                }




                if (0)
                {
                    SVGBuilder svg;
                    svg.style.penWidth = 0.1;
                    svg.style.pft = pftEvenOdd;
                    svg.style.brushClr = 0x1200009C;
                    svg.style.penClr = 0xCCD3D3DA;
                    //svg.AddPaths(subj);
                    //svg.style.brushClr = 0x129C0000;
                    //svg.style.penClr = 0xCCFFA07A;
                    //svg.AddPaths(clip);
                    svg.style.brushClr = 0x6080ff9C;
                    svg.style.penClr = 0xFF003300;
                    svg.style.pft = pftNonZero;

                    svg.AddPaths(solution);
                    char str[25];
#if _WIN32
                    itoa(em->vert.size(), str, 10);
#else
                    snprintf(str, 25, "%d", (int)em->vert.size());
#endif
                    string gFilenamePre = str;
                    string filename = "./SVG/off_" + gFilenamePre + ".svg";
                    svg.SaveToFile(filename, 0.01);
                }

                if (0)
                {
                    for (ClipperLib::Path& solutionSub : solution)
                    {
                        for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                        {
                            ClipperLib::IntPoint ptNew = solutionSub[ptIndex];

                            outBorderVertexs.emplace_back(trimesh::vec3(ptNew.X, ptNew.Y, ptNew.Z));
                            continue;
                            MyMesh::FaceType* rf;
                            float t;
                            Point3f dir(0.0, 0.0, 1.0);
                            vcg::Ray3f ray;
                            ray.SetOrigin(CoordType(ptNew.X, ptNew.Y, 0.0));
                            ray.SetDirection(dir);
                            rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                            if (rf)
                            {
                                CoordType raypt = ray.Origin() + ray.Direction() * t;
                                outBorderVertexs.emplace_back(raypt);
                            }
                        }
                    }
                    return;
                }
                ////////
                double areaTotal = 0.0;
                for (ClipperLib::Path& solutionSub : solution)
                {
                    double area = ClipperLib::Area(solutionSub);
                    areaTotal += area;
                    //int size = (int)solutionSub.size();
                    //float rad = 0.05 * 1000;
                    //if (area > (ClipperLib::pi * std::pow(sampleDist, 2)*0.7))
                    {
                        ScalarType lendelt = 0;
                        ClipperLib::IntPoint ptNew = solutionSub[0];
                        ScalarType  curLen = 0;
                        ScalarType  totalLen = 0;
                        size_t      sampleCnt = 0;
                        size_t      sampleNum = 1;
                        for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                        {
                            ClipperLib::IntPoint& pt = solutionSub[ptIndex];
                            ClipperLib::IntPoint ptNext = pt;
                            if (ptIndex == (solutionSub.size() - 1))
                            {
                                ptNext = solutionSub[0];

                            }
                            else
                                ptNext = solutionSub[ptIndex + 1];

                            totalLen = totalLen + PointsDistace(ptNext, pt);
                        }
                        sampleNum = std::ceil(totalLen / sampleDist);

                        for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                        {
                            ClipperLib::IntPoint& pt = solutionSub[ptIndex];
                            ClipperLib::IntPoint ptNext = pt;
                            if (ptIndex == (solutionSub.size() - 1))
                            {
                                ptNext = solutionSub[0];
                            }
                            else
                                ptNext = solutionSub[ptIndex + 1];

                            Point3f linedir(ptNext.X - pt.X, ptNext.Y - pt.Y, ptNext.Z - pt.Z);
                            linedir.Normalize();
                            {
                                ScalarType edgeLen = PointsDistace(pt, ptNext);
                                ScalarType d0 = curLen;
                                ScalarType d1 = d0 + edgeLen;

                                ClipperLib::IntPoint ptNew = pt;
                                while (d1 > sampleCnt * sampleDist && sampleCnt < sampleNum)
                                {
                                    ScalarType off = (sampleCnt * sampleDist - d0);

                                    Point3f lineOff = linedir * off;
                                    ptNew.X = pt.X + lineOff.X();
                                    ptNew.Y = pt.Y + lineOff.Y();
                                    ptNew.Z = pt.Z + lineOff.Z();

                                    MyMesh::FaceType* rf;
                                    float t;
                                    Point3f dir(0.0, 0.0, 1.0);
                                    vcg::Ray3f ray;
                                    ray.SetOrigin(CoordType(ptNew.X, ptNew.Y, ptNew.Z));
                                    ray.SetDirection(dir);
                                    rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                                    if (rf)
                                    {
                                        CoordType raypt = ray.Origin() + ray.Direction() * t;
                                        borderVertexs.emplace_back(raypt);
                                        //outBorderVertexs.emplace_back(raypt);
                                        hitflg = true;
                                    }
                                    sampleCnt++;
                                }
                                curLen += edgeLen;
                            }
                        }
                        if (hitflg == false)
                        {
                            std::cout << "raypt  no hit  gFilenamePre==" << std::endl;
                        }

                    }
                }
                {
                    if (firstflg == true)
                    {
                        if (areaTotal  >(ClipperLib::pi * std::pow(poissonCfgPtr->baseSampleRad * 1000, 2) * 0.7))
                        {
                            CoordType lowCenterPos(MeshSource->bbox.Center().X(), MeshSource->bbox.Center().Y(), 0.0);
                            MyMesh::FaceType* rf;
                            float t;
                            Point3f dir(0.0, 0.0, 1.0);
                            vcg::Ray3f ray;
                            ray.SetOrigin(lowCenterPos);
                            ray.SetDirection(dir);
                            rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                            if (rf)
                            {
                                CoordType raypt = ray.Origin() + ray.Direction() * t;
                                borderVertexs.emplace_back(raypt);
                            }
                        }
                        outContoursVertexs.insert(outContoursVertexs.end(), borderVertexs.begin(), borderVertexs.end());
                    }
                    firstflg = false;
                }

                {

                    outBorderVertexs.insert(outBorderVertexs.end(), borderVertexs.begin(), borderVertexs.end());
                    if (areaTotal < (ClipperLib::pi * std::pow(poissonCfgPtr->baseSampleRad * 1000, 2) * 0.7))
                        break;
                }



            } while (1);

            //std::cout << " outBorderVertexs===" << outBorderVertexs.size() << std::endl;

        }

#if 1
        void borderSamperPointOff(MyMesh* MeshSource, EdgeMeshType* em, ClipperLib::Paths& contourpaths, PoissonAlgCfg* poissonCfgPtr, std::vector<CoordType>& outBorderVertexs)
        {

            ClipperLib::Paths solution;
            ClipperLib::ClipperOffset co;
            TriMeshGrid static_grid;
            static_grid.Set(MeshSource->face.begin(), MeshSource->face.end());
            float maxDist =  MeshSource->bbox.max.Z() + 100;
            ScalarType  sampleDist = poissonCfgPtr->userSampleRad * 1000;
            bool hitflg = false;
            outBorderVertexs.clear();
            std::cout << __LINE__ << "  " << __FUNCTION__ << "contourpaths.size==" << contourpaths.size() << std::endl;
            if (contourpaths.size() == 0)
                return;
            if (0)
            {
                for (ClipperLib::Path& solutionSub : contourpaths)
                {
                    for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                    {
                        ClipperLib::IntPoint ptNew = solutionSub[ptIndex];

                        outBorderVertexs.emplace_back(trimesh::vec3(ptNew.X, ptNew.Y, ptNew.Z));
                        continue;
                        MyMesh::FaceType* rf;
                        float t;
                        Point3f dir(0.0, 0.0, 1.0);
                        vcg::Ray3f ray;
                        ray.SetOrigin(CoordType(ptNew.X, ptNew.Y, 0.0));
                        ray.SetDirection(dir);
                        rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                        if (rf)
                        {
                            CoordType raypt = ray.Origin() + ray.Direction() * t;
                            outBorderVertexs.emplace_back(raypt);
                        }
                    }
                }
                return;
            }

            co.AddPaths(contourpaths, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
            co.Execute(solution, poissonCfgPtr->borderSampleOff * 1000);

            if (0)
            {
                SVGBuilder svg;
                svg.style.penWidth = 0.1;
                svg.style.pft = pftEvenOdd;
                svg.style.brushClr = 0x1200009C;
                svg.style.penClr = 0xCCD3D3DA;
                //svg.AddPaths(subj);
                //svg.style.brushClr = 0x129C0000;
                //svg.style.penClr = 0xCCFFA07A;
                //svg.AddPaths(clip);
                svg.style.brushClr = 0x6080ff9C;
                svg.style.penClr = 0xFF003300;
                svg.style.pft = pftNonZero;

                svg.AddPaths(solution);
                char str[25];
#if _WIN32
                itoa(em->vert.size(), str, 10);
#else
                snprintf(str, 25, "%d", (int)em->vert.size());
#endif
                string gFilenamePre = str;
                string filename = "./SVG/off_" + gFilenamePre + ".svg";
                svg.SaveToFile(filename, 0.01);
            }

            if (0)
            {
                for (ClipperLib::Path& solutionSub : solution)
                {
                    for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                    {
                        ClipperLib::IntPoint ptNew = solutionSub[ptIndex];

                        outBorderVertexs.emplace_back(trimesh::vec3(ptNew.X, ptNew.Y, ptNew.Z));
                        continue;
                        MyMesh::FaceType* rf;
                        float t;
                        Point3f dir(0.0, 0.0, 1.0);
                        vcg::Ray3f ray;
                        ray.SetOrigin(CoordType(ptNew.X, ptNew.Y, 0.0));
                        ray.SetDirection(dir);
                        rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                        if (rf)
                        {
                            CoordType raypt = ray.Origin() + ray.Direction() * t;
                            outBorderVertexs.emplace_back(raypt);
                        }
                    }
                }
                return;
            }
            ////////
            for (ClipperLib::Path& solutionSub : solution)
            {
                //double area = ClipperLib::Area(solutionSub);
                //int size = (int)solutionSub.size();
                //float rad = 0.05 * 1000;
                //if (area > ClipperLib::pi * std::pow(rad , 2))
                {
                    ScalarType lendelt = 0;
                    ClipperLib::IntPoint ptNew = solutionSub[0];
                    ScalarType  curLen = 0;
                    ScalarType  totalLen = 0;
                    size_t      sampleCnt = 0;
                    size_t      sampleNum = 1;
                    for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                    {
                        ClipperLib::IntPoint& pt = solutionSub[ptIndex];
                        ClipperLib::IntPoint ptNext = pt;
                        if (ptIndex == (solutionSub.size() - 1))
                        {
                            ptNext = solutionSub[0];

                        }
                        else
                            ptNext = solutionSub[ptIndex + 1];

                        totalLen = totalLen + PointsDistace(ptNext, pt);
                    }
                    sampleNum = std::ceil(totalLen / sampleDist);

                    for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                    {
                        ClipperLib::IntPoint& pt = solutionSub[ptIndex];
                        ClipperLib::IntPoint ptNext = pt;
                        if (ptIndex == (solutionSub.size() - 1))
                        {
                            ptNext = solutionSub[0];
                        }
                        else
                            ptNext = solutionSub[ptIndex + 1];

                        Point3f linedir(ptNext.X - pt.X, ptNext.Y - pt.Y, ptNext.Z - pt.Z);
                        linedir.Normalize();
                        {
                            ScalarType edgeLen = PointsDistace(pt, ptNext);
                            ScalarType d0 = curLen;
                            ScalarType d1 = d0 + edgeLen;

                            ClipperLib::IntPoint ptNew = pt;
                            while (d1 > sampleCnt * sampleDist && sampleCnt < sampleNum)
                            {
                                ScalarType off = (sampleCnt * sampleDist - d0);

                                Point3f lineOff = linedir * off;
                                ptNew.X = pt.X + lineOff.X();
                                ptNew.Y = pt.Y + lineOff.Y();
                                ptNew.Z = pt.Z + lineOff.Z();

                                MyMesh::FaceType* rf;
                                float t;
                                Point3f dir(0.0, 0.0, 1.0);
                                vcg::Ray3f ray;
                                ray.SetOrigin(CoordType(ptNew.X, ptNew.Y, ptNew.Z));
                                ray.SetDirection(dir);
                                rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                                if (rf)
                                {
                                    CoordType raypt = ray.Origin() + ray.Direction() * t;
                                    outBorderVertexs.emplace_back(raypt);
                                    hitflg = true;
                                }
                                sampleCnt++;
                            }
                            curLen += edgeLen;
                        }
                    }
                    if (hitflg == false)
                    {
                        std::cout << "raypt  no hit  gFilenamePre==" << std::endl;
                        //for (ClipperLib::Path& solutionSub : solution)
                        //{
                        //    for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                        //    {
                        //        ClipperLib::IntPoint& pt = solutionSub[ptIndex];
                        //        outBorderVertexs.emplace_back(CoordType(pt.X, pt.Y, pt.Z+5000));
                        //    }
                        //}
                    }
                }
            }
            std::cout << " outBorderVertexs===" << outBorderVertexs.size() << std::endl;

        }
#else
void borderSamperPointOff(MyMesh* MeshSource, EdgeMeshType* em, ClipperLib::Paths& contourpaths, PoissonAlgCfg* poissonCfgPtr, std::vector<CoordType>& outBorderVertexs)
{

    ClipperLib::Paths solution;
    PolyTree polytree;
    ClipperLib::ClipperOffset co;
    TriMeshGrid static_grid;
    static_grid.Set(MeshSource->face.begin(), MeshSource->face.end());
    float maxDist = 10000 * 1000; //MeshSource->bbox.max.Z() + 100;
    bool hitflg = false;
    std::cout << __LINE__ << "  " << __FUNCTION__ << "contourpaths.size==" << contourpaths.size() << std::endl;
    if (contourpaths.size() == 0)
        return;
    if (0)
    {
        SVGBuilder svg;
        svg.style.penWidth = 0.1;
        svg.style.pft = pftEvenOdd;
        svg.style.brushClr = 0x1200009C;
        svg.style.penClr = 0xCCD3D3DA;
        //svg.AddPaths(subj);
        //svg.style.brushClr = 0x129C0000;
        //svg.style.penClr = 0xCCFFA07A;
        //svg.AddPaths(clip);
        svg.style.brushClr = 0x6080ff9C;
        svg.style.penClr = 0xFF003300;
        svg.style.pft = pftNonZero;

        svg.AddPaths(contourpaths);
        char str[25];
		#if _WIN32
                itoa(em.vert.size(), str, 10);
		#else
                snprintf(str, 25, "%d", (int)em.vert.size());
		#endif

        string gFilenamePre = str;

        string filename = "./SVG/contours_" + gFilenamePre + ".svg";
        svg.SaveToFile(filename, 0.01);
    }
    if (0)
    {
        for (ClipperLib::Path& solutionSub : contourpaths)
        {
            for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
            {
                ClipperLib::IntPoint ptNew = solutionSub[ptIndex];

                outBorderVertexs.emplace_back(trimesh::vec3(ptNew.X, ptNew.Y, ptNew.Z));
                continue;
                MyMesh::FaceType* rf;
                float t;
                Point3f dir(0.0, 0.0, 1.0);
                vcg::Ray3f ray;
                ray.SetOrigin(CoordType(ptNew.X, ptNew.Y, 0.0));
                ray.SetDirection(dir);
                rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                if (rf)
                {
                    CoordType raypt = ray.Origin() + ray.Direction() * t;
                    outBorderVertexs.emplace_back(raypt);
                }
            }
        }
        return;
    }

    co.AddPaths(contourpaths, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
    // co.Execute(solution, poissonCfgPtr->borderSampleOff*1000);
    co.Execute(polytree, -0.5 * 1000);

    if (0)
    {
        SVGBuilder svg;
        svg.style.penWidth = 0.1;
        svg.style.pft = pftEvenOdd;
        svg.style.brushClr = 0x1200009C;
        svg.style.penClr = 0xCCD3D3DA;
        //svg.AddPaths(subj);
        //svg.style.brushClr = 0x129C0000;
        //svg.style.penClr = 0xCCFFA07A;
        //svg.AddPaths(clip);
        svg.style.brushClr = 0x6080ff9C;
        svg.style.penClr = 0xFF003300;
        svg.style.pft = pftNonZero;

        svg.AddPaths(solution);
        char str[25];
		#if _WIN32
        itoa(MeshSource->vert.size(), str, 10);
		#else
        snprintf(str, 25, "%d", (int)MeshSource->vert.size());
		#endif

        string gFilenamePre = str;
        string filename = "./SVG/off_" + gFilenamePre + ".svg";
        svg.SaveToFile(filename, 0.01);
    }


    PolyNode* polynode = polytree.GetFirst();
    if (polynode)
    {
        ClipperLib::Path& solutionSub = polynode->Contour;

        if (0)
        {
            //for (ClipperLib::Path& solutionSub : solution)
            //{
            for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
            {
                ClipperLib::IntPoint ptNew = solutionSub[ptIndex];

                MyMesh::FaceType* rf;
                float t;
                Point3f dir(0.0, 0.0, 1.0);
                vcg::Ray3f ray;
                ray.SetOrigin(CoordType(ptNew.X, ptNew.Y, ptNew.Z));
                ray.SetDirection(dir);
                rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                if (rf)
                {
                    CoordType raypt = ray.Origin() + ray.Direction() * t;
                    outBorderVertexs.emplace_back(raypt);
                }
            }
            //}
            return;
        }

        // for (ClipperLib::Path& solutionSub : solution)
        {
            //double area = ClipperLib::Area(solutionSub);
            //int size = (int)solutionSub.size();
            //float rad = 0.05 * 1000;
            //if (area > ClipperLib::pi * std::pow(rad , 2))
            {
                ScalarType lendelt = 0;
                ClipperLib::IntPoint ptNew = solutionSub[0];
                ScalarType  curLen = 0;
                ScalarType  totalLen = 0;
                ScalarType  sampleDist = poissonCfgPtr->userSampleRad * 1000;
                size_t      sampleCnt = 0;
                size_t      sampleNum = 1;
                for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                {
                    ClipperLib::IntPoint& pt = solutionSub[ptIndex];
                    ClipperLib::IntPoint ptNext = pt;
                    if (ptIndex == (solutionSub.size() - 1))
                    {
                        ptNext = solutionSub[0];

                    }
                    else
                        ptNext = solutionSub[ptIndex + 1];

                    totalLen = totalLen + PointsDistace(ptNext, pt);
                }
                sampleNum = std::ceil(totalLen / sampleDist);
                std::cout << " size===" << solutionSub.size() << std::endl;
                //std::cout << " area===" << area << std::endl;
                std::cout << " sampleNum===" << sampleNum << std::endl;

                for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                {
                    ClipperLib::IntPoint& pt = solutionSub[ptIndex];
                    ClipperLib::IntPoint ptNext = pt;
                    if (ptIndex == (solutionSub.size() - 1))
                    {
                        ptNext = solutionSub[0];
                    }
                    else
                        ptNext = solutionSub[ptIndex + 1];

                    Point3f linedir(ptNext.X - pt.X, ptNext.Y - pt.Y, ptNext.Z - pt.Z);
                    linedir.Normalize();
                    {
                        ScalarType edgeLen = PointsDistace(pt, ptNext);
                        ScalarType d0 = curLen;
                        ScalarType d1 = d0 + edgeLen;

                        ClipperLib::IntPoint ptNew = pt;
                        while (d1 > sampleCnt * sampleDist && sampleCnt < sampleNum)
                        {
                            ScalarType off = (sampleCnt * sampleDist - d0);

                            Point3f lineOff = linedir * off;
                            ptNew.X = pt.X + lineOff.X();
                            ptNew.Y = pt.Y + lineOff.Y();
                            ptNew.Z = pt.Z + lineOff.Z();

                            MyMesh::FaceType* rf;
                            float t;
                            Point3f dir(0.0, 0.0, 1.0);
                            vcg::Ray3f ray;
                            ray.SetOrigin(CoordType(ptNew.X, ptNew.Y, ptNew.Z));
                            ray.SetDirection(dir);
                            rf = tri::DoRay<MyMesh, TriMeshGrid>(*MeshSource, static_grid, ray, maxDist, t);
                            if (rf)
                            {
                                CoordType raypt = ray.Origin() + ray.Direction() * t;
                                outBorderVertexs.emplace_back(raypt);
                                hitflg = true;
                            }
                            sampleCnt++;
                        }
                        curLen += edgeLen;
                    }
                }
                if (hitflg == false)
                {
                    std::cout << "raypt  no hit  gFilenamePre==" << std::endl;
                    //for (ClipperLib::Path& solutionSub : solution)
                    //{
                    //    for (int ptIndex = 0; ptIndex < solutionSub.size(); ptIndex++)
                    //    {
                    //        ClipperLib::IntPoint& pt = solutionSub[ptIndex];
                    //        outBorderVertexs.emplace_back(CoordType(pt.X, pt.Y, pt.Z+5000));
                    //    }
                    //}
                }
            }
        }
    }


}

#endif
        PoissonFunc::PoissonFunc()
        {
            m_PoissonAlgCfg.baseSampleRad = 0.5;
            m_PoissonAlgCfg.userSampleRad = 2.0;
            m_SurfaceMeshSource = NULL;
            m_BasicPoissonMesh = NULL;
        }
        PoissonFunc::~PoissonFunc()
        {
            releaseData();
        
        }
        void PoissonFunc::setPoissonCfg(PoissonAlgCfg* cfgPtr)
        {
            m_PoissonAlgCfg = *cfgPtr;
        }
        bool PoissonFunc::main(const std::vector<trimesh::vec3>& inVertexes, const std::vector<trimesh::TriMesh::Face>& inFaces, std::vector<trimesh::vec3>& outVertexes, bool secondflg)
        {

            int sampleNum = 0;
            int sampleRatioNum = 0;
            float rad = 1.0;
            vector<CoordType> coordVec;
            vector<Point3i> indexVec;
            coordVec.clear();
            indexVec.clear();
            MyMesh BasicPoissonMesh;
            MyMesh m;
#if 1
            for (trimesh::vec3 pt : inVertexes) {
                coordVec.push_back(Point3f(pt.x*1000, pt.y * 1000, pt.z * 1000));
            }
            for (trimesh::TriMesh::Face faceIndex : inFaces) {
                indexVec.push_back(Point3i(faceIndex[0], faceIndex[1], faceIndex[2]));
            }
#else
            int faceIndexIndex = 0;
            for (trimesh::TriMesh::Face faceIndex : inFaces) {
                coordVec.push_back(Point3f(inVertexes[faceIndex[0]].x, inVertexes[faceIndex[0]].y, inVertexes[faceIndex[0]].z));
                coordVec.push_back(Point3f(inVertexes[faceIndex[1]].x, inVertexes[faceIndex[1]].y, inVertexes[faceIndex[1]].z));
                coordVec.push_back(Point3f(inVertexes[faceIndex[2]].x, inVertexes[faceIndex[2]].y, inVertexes[faceIndex[2]].z));
                indexVec.push_back(Point3i(faceIndexIndex, faceIndexIndex + 1, faceIndexIndex + 2));
                faceIndexIndex += 3;
            }

#endif

            tri::BuildMeshFromCoordVectorIndexVector(m, coordVec /*coordVec*/, indexVec);//diskMesh
            //tri::BuildMeshFromCoordVector(m, coordVec);//diskMesh
            //tri::io::ExporterOFF<MyMesh>::Save(m, "disc.off");


            int dup = tri::Clean<MyMesh>::RemoveDuplicateVertex(m);
            int unref = tri::Clean<MyMesh>::RemoveUnreferencedVertex(m);

            //if (dup > 0 || unref > 0)
            //    printf("Removed %i duplicate and %i unreferenced vertices from mesh \n", dup, unref);

           tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::SamplingRandomGenerator().initialize(time(0));
            //----------------------------------------------------------------------
            // Advanced Sample
            // Make a feature dependent Poisson Disk sampling


            std::vector<CoordType> sampleVec;
            std::vector<CoordType> outBorderVertexs;
            std::vector<CoordType> outExternVertexes;
            std::vector<CoordType> outSecondVertexs;
            std::vector<CoordType> outSampleVec;
            tri::TrivialSampler<MyMesh> mps(sampleVec);

            tri::UpdateTopology<MyMesh>::FaceFace(m);
            tri::UpdateNormal<MyMesh>::PerFace(m);
            tri::UpdateFlags<MyMesh>::FaceEdgeSelCrease(m, math::ToRad(40.0f));

            tri::UpdateBounding<MyMesh>::Box(m);
            tri::UpdateNormal<MyMesh>::PerFaceNormalized(m);
            tri::UpdateNormal<MyMesh>::PerVertexAngleWeighted(m);
            tri::UpdateNormal<MyMesh>::NormalizePerVertex(m);

            Clean<MyMesh>::RemoveDuplicateVertex(m, true);
            Allocator<MyMesh>::CompactVertexVector(m);

            tri::Clean<MyMesh>::RemoveUnreferencedVertex(m);
            tri::Allocator<MyMesh>::CompactEveryVector(m);

            tri::UpdateTopology<MyMesh>::VertexFace(m);


            ////// select all vertices (to mark them fixed)
            UpdateFlags<MyMesh>::VertexSetS(m);

            //tri::InitFaceIMark(m);

            //MyMesh::FaceIterator f;

            //for (f = m.face.begin(); f != m.face.end(); ++f)
            //    if (!(*f).IsD() && (*f).IsR() && (*f).IsW())
            //    {
            //        tri::UnMarkAll(m);
            //        tri::Mark(m, &*f);

            //    }
            ////        (*f).InitIMark();

            //for (auto fi = m.face.begin(); fi != m.face.end(); fi++)
            //{
            //    tri::UnMarkAll(m);
            //    if (!(*fi).IsD())
            //    {
            //        if (!tri::IsMarked(m, &*fi))
            //        {
            //            tri::Mark(m, &*fi);
            //        }
            //    }

            //}

            //tri::RequirePerFaceMark(m);

            tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskParam pp;
            ////////////////////////////////////////////////
            
            sampleVec.clear();
            rad = m_PoissonAlgCfg.baseSampleRad*1000;
            sampleNum = tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh>>::ComputePoissonSampleNum(m, rad);
#ifdef DEBUG
            std::cout << "sampleNum origal===" << sampleNum << std::endl;
#endif
            if (sampleNum == 0)
                return false;
            sampleNum = sampleNum * m_PoissonAlgCfg.ratio;
            if (sampleNum < 1)
            {
                sampleNum = 1;
            }
            //std::cout << "sampleNum baseSampleRad===" << m_PoissonAlgCfg.baseSampleRad << std::endl;
            //std::cout << "sampleNum userSampleRad===" << m_PoissonAlgCfg.userSampleRad << std::endl;
            //std::cout << "sampleNum borderSampleOff===" << m_PoissonAlgCfg.borderSampleOff << std::endl;
            //std::cout << "sampleNum ratio===" << m_PoissonAlgCfg.ratio << std::endl;
            //std::cout << "sampleNum ===" << sampleNum << std::endl;

            //tri::PoissonSampling<MyMesh>(m, sampleVec, sampleNum, rad);

            //tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::Montecarlo(m, mps, sampleNum);
            //tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::MontecarloPoisson(m, mps, sampleNum);
                
            //tri::BuildMeshFromCoordVector(BasicPoissonMesh, sampleVec);
            //tri::io::ExporterOFF<MyMesh>::Save(BasicPoissonMesh, "BasicPoissonMesh.off");

            ///////////////////////////////////////////
            //{
            //    // tri::io::ExporterOFF<MyMesh>::Save(MontecarloSurfaceMesh, "MontecarloSurfaceMesh.off");
            //    for (Point3f pt : sampleVec)
            //    {
            //        outVertexes.emplace_back(trimesh::vec3(pt.X(), pt.Y(), pt.Z()));
            //    }
            //}
            //if (sampleVec.size())
            {
                //tri::BuildMeshFromCoordVector(BasicPoissonMesh, sampleVec);
#if 0
                borderSamper(&m, &m_PoissonAlgCfg, outBorderVertexs);
#else
                {
                    ClipperLib::Paths outPaths;
                    ClipperLib::Paths outDiffPaths;
                    EdgeMeshType em;
                    borderPaths(&m, outPaths, outDiffPaths, em);
                    //borderSamperPointOff(&m, &em, outPaths, &m_PoissonAlgCfg, outBorderVertexs);
                    borderSamperClipPoint(&m, &em, outPaths, &m_PoissonAlgCfg, outBorderVertexs, outExternVertexes);
                }
#endif
                mainSecond(&m, &m_PoissonAlgCfg, outBorderVertexs, outExternVertexes, sampleNum + outBorderVertexs.size(), outSecondVertexs);
                //if (outBorderVertexs.size())
                //{

                //    if (outBorderVertexs.size() < outSecondVertexs.size())
                //    {
                //        outSampleVec.swap(outSecondVertexs);
                //    }
                //    else
                //        outSampleVec.swap(outBorderVertexs);

                //}
                //else
                //{
                //    outSampleVec.swap(sampleVec);
                //}
            }
            for (auto &pt : outSecondVertexs)
            {
                outVertexes.emplace_back(trimesh::vec3(pt.X()/1000, pt.Y()/1000, pt.Z()/1000));
            }

            //tri::io::ExporterOFF<MyMesh>::Save(PoissonMesh, "PoissonMesh.off");
            //printf("Computed a feature aware poisson disk distribution of %i vertices radius is %6.3f\n", PoissonMesh.VN(), rad);
            return true;
        }

        void PoissonFunc::releaseData()
        {
            if (m_BasicPoissonMesh)
            {
                MyMesh& basicPoissonMesh = *(MyMesh*)m_BasicPoissonMesh;

                basicPoissonMesh.Clear();
                delete m_BasicPoissonMesh;
                m_BasicPoissonMesh = NULL;
            }
            if (m_SurfaceMeshSource)
            {
                MyMesh& surfaceMeshSource = *(MyMesh*)m_SurfaceMeshSource;
                surfaceMeshSource.Clear();
                delete m_SurfaceMeshSource;
                m_SurfaceMeshSource = NULL;
            }

        }
    }//CX_PoissonAlg End
}//vcg End

#endif // USE_VCG
#endif
