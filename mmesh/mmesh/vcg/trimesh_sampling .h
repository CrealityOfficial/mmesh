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

#include <vector>
#include "trimesh2/TriMesh.h"

#ifdef USE_VCG
namespace vcg
{
    namespace CX_PoissonAlg {
		typedef struct PoissonAlgConfigure
		{
			float baseSampleRad=1.0;//基础采样半径
			float userSampleRad=2.0;//用户选择采样半径
			float borderSampleOff=-0.5;//沿边采样半径向量偏移量
			float ratio=1.0;//用户选择采样半径
		}PoissonAlgCfg;

		class PoissonFunc
		{
		public:
			PoissonFunc();
			~PoissonFunc();

			bool main(const std::vector<trimesh::vec3> &inVertexes, const std::vector<trimesh::TriMesh::Face> &inFaces, std::vector<trimesh::vec3>& outVertexes, bool secondflg=false);
			void setPoissonCfg(PoissonAlgCfg* cfgPtr);
			void releaseData();
		private:
			PoissonAlgCfg m_PoissonAlgCfg;
			void* m_BasicPoissonMesh;
			void* m_SurfaceMeshSource;
		};
    }//PoissonAlg
}//vcg

#endif USE_VCG

