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

        class MyVertex : public Vertex<MyUsedTypes, vertex::Coord3f, vertex::Normal3f, vertex::BitFlags  > {};
        class MyFace : public Face< MyUsedTypes, face::FFAdj, face::Normal3f, face::VertexRef, face::BitFlags > {};
        class MyEdge : public Edge<MyUsedTypes> {};
        class MyMesh : public tri::TriMesh< vector<MyVertex>, vector<MyFace>, vector<MyEdge>  > {};

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
        bool PoissonFunc::first(const std::vector<trimesh::vec3> &inVertexes,const std::vector<trimesh::TriMesh::Face> &inFaces, std::vector<trimesh::vec3>& outVertexes,bool secondflg)
        {

            int sampleNum = 0;
            int sampleRatioNum = 0;
            float rad = 1.0;
            vector<Point3f> coordVec;
            vector<Point3i> indexVec;
            coordVec.clear();
            indexVec.clear();
            if (m_SurfaceMeshSource != NULL)
            {
                delete m_SurfaceMeshSource;
                m_SurfaceMeshSource = NULL;
            }
            m_SurfaceMeshSource = new MyMesh;
            if (m_SurfaceMeshSource == NULL)
                return false;
            if (m_BasicPoissonMesh != NULL)
            {
                delete m_BasicPoissonMesh;
                m_BasicPoissonMesh = NULL;
            }
            m_BasicPoissonMesh = new MyMesh;
            if (m_BasicPoissonMesh == NULL)
                return false;
            if (m_SurfaceMeshSource == NULL)
                return false;
            MyMesh &m= *(MyMesh*)m_SurfaceMeshSource;
#if 1
            for (trimesh::vec3 pt : inVertexes) {
                coordVec.push_back(Point3f(pt.x, pt.y, pt.z));
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
           
            tri::BuildMeshFromCoordVectorIndexVector(m, coordVec, indexVec);//diskMesh
            //tri::BuildMeshFromCoordVector(m, coordVec);//diskMesh
            //tri::io::ExporterOFF<MyMesh>::Save(m, "disc.off");
            tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::SamplingRandomGenerator().initialize(time(0));
            //----------------------------------------------------------------------
            // Advanced Sample
            // Make a feature dependent Poisson Disk sampling

            MyMesh& BasicPoissonMesh = *(MyMesh*)m_BasicPoissonMesh;

            std::vector<Point3f> sampleVec;
            tri::TrivialSampler<MyMesh> mps(sampleVec);
            tri::UpdateTopology<MyMesh>::FaceFace(m);
            tri::UpdateNormal<MyMesh>::PerFace(m);
            tri::UpdateFlags<MyMesh>::FaceEdgeSelCrease(m, math::ToRad(40.0f));
            tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskParam pp;
            ////////////////////////////////////////////////
 
            sampleVec.clear();
            rad = m_PoissonAlgCfg.baseSampleRad;
            sampleNum = tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh>>::ComputePoissonSampleNum(m, rad);
            #ifdef DEBUG
            std::cout << "sampleNum origal===" << sampleNum << std::endl;
            #endif
            if (sampleNum == 0)
                return false;
            sampleNum= sampleNum * m_PoissonAlgCfg.ratio;
            if (sampleNum <1)
            {
                sampleNum = 1;
            }
            tri::PoissonSampling<MyMesh>(m, sampleVec, sampleNum, rad);

            //tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::Montecarlo(m, mps, sampleNum)
            //tri::BuildMeshFromCoordVector(BasicPoissonMesh, sampleVec);
            //tri::io::ExporterOFF<MyMesh>::Save(BasicPoissonMesh, "BasicPoissonMesh.off");

///////////////////////////////////////////
            
            if (secondflg)
            {
                tri::BuildMeshFromCoordVector(BasicPoissonMesh, sampleVec);
                m_sampleNum = sampleVec.size();
            }
           
            {
                // tri::io::ExporterOFF<MyMesh>::Save(MontecarloSurfaceMesh, "MontecarloSurfaceMesh.off");
                for (Point3f pt : sampleVec)
                {
                    outVertexes.emplace_back(trimesh::vec3(pt.X(), pt.Y(), pt.Z()));
                }
            }
            //tri::io::ExporterOFF<MyMesh>::Save(PoissonMesh, "PoissonMesh.off");
            //printf("Computed a feature aware poisson disk distribution of %i vertices radius is %6.3f\n", PoissonMesh.VN(), rad);
            return true;
        }
        bool PoissonFunc::mainSecond(std::vector<trimesh::vec3> inVertexes,std::vector<trimesh::vec3>& outVertexes)
        {
            if (m_BasicPoissonMesh)
            {
                //----------------------------------------------------------------------
                // Advanced Sample
                // Make a feature dependent Poisson Disk sampling
                MyMesh& MontecarloSurfaceMesh = *(MyMesh*)m_BasicPoissonMesh;;
                MyMesh PoissonEdgeMesh;

                float rad = m_PoissonAlgCfg.userSampleRad;
                //float rad = m_sampleRadius;
                 rad = ComputePoissonDiskRadius(MontecarloSurfaceMesh, m_sampleNum+ inVertexes.size());

                //tri::io::ExporterOFF<MyMesh>::Save(m, "disc.off");
                tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::SamplingRandomGenerator().initialize(time(0));

                vector<Point3f> coordVec;
                for (trimesh::vec3 pt : inVertexes) {
                    coordVec.push_back(Point3f(pt.x, pt.y, pt.z));
                }
                if(coordVec.size()>3)
                    tri::BuildMeshFromCoordVector(PoissonEdgeMesh, coordVec);

                std::vector<Point3f> sampleVec;
                tri::TrivialSampler<MyMesh> mps(sampleVec);
                tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskParam pp;

                // tri::io::ExporterOFF<MyMesh>::Save(MontecarloSurfaceMesh, "MontecarloSurfaceMesh.off");
                pp.preGenMesh = &PoissonEdgeMesh;
                pp.preGenFlag = true;
                sampleVec.clear();
                tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskPruning(mps, MontecarloSurfaceMesh, rad, pp);

                for (Point3f pt : sampleVec)
                {
                    outVertexes.emplace_back(trimesh::vec3(pt.X(), pt.Y(), pt.Z()));
                }

                //tri::io::ExporterOFF<MyMesh>::Save(PoissonMesh, "PoissonMesh.off");
                //printf("Computed a feature aware poisson disk distribution of %i vertices radius is %6.3f\n", PoissonMesh.VN(), rad);
                return true;
            }
            else
                return false;
        }
        void PoissonFunc::borderSamper(std::vector<trimesh::vec3>& outVertexes)
        {
            MyMesh poissonEdgeMesh;
            if(m_SurfaceMeshSource==NULL)
                return;
            MyMesh* meshPtr = (MyMesh*)m_SurfaceMeshSource;
            typedef typename VoroEdgeMeshAux::EdgeMeshType EdgeMeshType;
            typedef typename EdgeMeshType::CoordType Coord;
            typedef typename MyMesh::VertexType     VertexType;
            EdgeMeshType em;
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
                    auto vi = Allocator<EdgeMeshType>::AddVertices(sides, e.size());
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
            ExtractMeshBorders(*meshPtr, em);

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
            //io::ExporterOBJ<EdgeMeshType>::Save(em, QString("edgeMesh_%1.obj").arg(idx).toStdString().c_str(), io::Mask::IOM_EDGEINDEX);


            // Samples vector
            std::vector<Coord> borderSamples;
            TrivialSampler<EdgeMeshType> ps(borderSamples);
            float rad = m_PoissonAlgCfg.userSampleRad;
            // uniform edge sampling
            UpdateTopology<EdgeMeshType>::EdgeEdge(em);
            SurfaceSampling<EdgeMeshType>::EdgeMeshUniform(em, ps, rad, SurfaceSampling<EdgeMeshType>::Ceil);
           // BuildMeshFromCoordVector(poissonEdgeMesh, borderSamples);
            for (Coord &pt : borderSamples)
            {
                //const InCoordType& vv = v[i];
                //in.vert[i].P() = CoordType(vv[0], vv[1], vv[2]);

                outVertexes.emplace_back(trimesh::vec3(pt.X(), pt.Y(), pt.Z()));
            }

            //// remove duplicate vertices
            //Clean<MyMesh>::RemoveDuplicateVertex(poissonEdgeMesh, false);
            //Allocator<MyMesh>::CompactVertexVector(poissonEdgeMesh);

            //// select all vertices (to mark them fixed)
            //UpdateFlags<MyMesh>::VertexSetS(poissonEdgeMesh);

            //tri::io::ExporterOFF<MyMesh>::Save(*meshPtr, "m.off");
            //tri::io::ExporterOFF<MyMesh>::Save(poissonEdgeMesh, "poissonEdgeMesh.off");
        }
        void PoissonFunc::releaseData()
        {
            if (m_BasicPoissonMesh)
            {
                delete m_BasicPoissonMesh;
                m_BasicPoissonMesh = NULL;
            }
            if (m_SurfaceMeshSource)
            {
                delete m_SurfaceMeshSource;
                m_SurfaceMeshSource = NULL;
            }

        }
    }//CX_PoissonAlg End
}//vcg End

#endif
