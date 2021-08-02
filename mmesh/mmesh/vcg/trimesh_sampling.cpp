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
            m_SurfaceMeshInner = NULL;
        }
        PoissonFunc::~PoissonFunc()
        {
            if (m_SurfaceMeshInner)
                delete m_SurfaceMeshInner;
        
        }
        void PoissonFunc::setPoissonCfg(PoissonAlgCfg* cfgPtr)
        {
            m_PoissonAlgCfg = *cfgPtr;
        }
        bool PoissonFunc::main(std::vector<trimesh::vec3> inVertexes, std::vector<trimesh::TriMesh::Face> inFaces, std::vector<trimesh::vec3>& outVertexes,bool secondflg)
        {

            MyMesh m;
            vector<Point3f> coordVec;
            vector<Point3i> indexVec;
            int sampleNum = 0;
            int sampleRatioNum = 0;
            float rad = 1.0;
            for (trimesh::vec3 pt : inVertexes) {
                coordVec.push_back(Point3f(pt.x, pt.y, pt.z));
            }
            for (trimesh::TriMesh::Face faceIndex : inFaces) {
                indexVec.push_back(Point3i(faceIndex[0], faceIndex[1], faceIndex[2]));
            }
            tri::BuildMeshFromCoordVectorIndexVector(m, coordVec, indexVec);//diskMesh
            //tri::io::ExporterOFF<MyMesh>::Save(m, "disc.off");
            tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::SamplingRandomGenerator().initialize(time(0));
            //----------------------------------------------------------------------
            // Advanced Sample
            // Make a feature dependent Poisson Disk sampling
            if (m_SurfaceMeshInner)
            {
                delete m_SurfaceMeshInner;
            }
            m_SurfaceMeshInner = new MyMesh;

            MyMesh& MontecarloSurfaceMesh = *(MyMesh*)m_SurfaceMeshInner;;

            std::vector<Point3f> sampleVec;
            tri::TrivialSampler<MyMesh> mps(sampleVec);
            tri::UpdateTopology<MyMesh>::FaceFace(m);
            tri::UpdateNormal<MyMesh>::PerFace(m);
            tri::UpdateFlags<MyMesh>::FaceEdgeSelCrease(m, math::ToRad(40.0f));
            tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::PoissonDiskParam pp;

            sampleVec.clear();
            rad = m_PoissonAlgCfg.baseSampleRad;
            sampleNum = tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh>>::ComputePoissonSampleNum(m, rad);
            #ifdef DEBUG
            std::cout << "sampleNum origal===" << sampleNum << std::endl;
            #endif
            if (sampleNum == 0)
                return false;
            //std::cout << "sampleNum==" << sampleNum << std::endl;

            //sampleRatioNum= sampleNum * m_PoissonAlgCfg.sampleRatio;
            //sampleRatioNum = sampleRatioNum > 0 ? sampleRatioNum : 1;
            //do 
            //{
            //    rad += 0.1;
            //    sampleNum = tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh>>::ComputePoissonSampleNum(m, rad);
            //    std::cout << "sampleNum New===" << sampleNum << std::endl;
            //    std::cout << "rad New===" << rad << std::endl;
            //} while (sampleNum > sampleRatioNum);

            //sampleNum = sampleNum < sampleRatioNum ? sampleNum : 1;

            tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::Montecarlo(m, mps, sampleNum);
            
            if (secondflg)
            {
                tri::BuildMeshFromCoordVector(MontecarloSurfaceMesh, sampleVec);
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
        bool PoissonFunc::mainSecond(std::vector<trimesh::vec3>& outVertexes)
        {
            void* SurfaceMeshBetween = m_SurfaceMeshInner;
            if (SurfaceMeshBetween)
            {

                int sampleNum = 0;
                float rad = m_PoissonAlgCfg.userSampleRad;
                //tri::io::ExporterOFF<MyMesh>::Save(m, "disc.off");
                tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh> >::SamplingRandomGenerator().initialize(time(0));
                //----------------------------------------------------------------------
                // Advanced Sample
                // Make a feature dependent Poisson Disk sampling
                MyMesh& MontecarloSurfaceMesh = *(MyMesh*)SurfaceMeshBetween;;
                MyMesh PoissonEdgeMesh;

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
    }//CX_PoissonAlg End
}//vcg End

#endif
