#include "hollowout.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "mmesh/create/createcube.h"

namespace mmesh
{
	 trimesh::TriMesh* hollowout(trimesh::TriMesh* inputMesh)
	 {
		trimesh::TriMesh* boxMesh = new trimesh::TriMesh();
		
		inputMesh->need_bbox();
		boxMesh = mmesh::createCube(inputMesh->bbox,1.0);

		std::vector<trimesh::TriMesh*> inMeshes;
		mmesh::reverseTriMesh(inputMesh);
		inMeshes.push_back(inputMesh);
		inMeshes.push_back(boxMesh);

		trimesh::TriMesh* outputMesh = new trimesh::TriMesh();
		mmesh::mergeTriMesh(outputMesh, inMeshes);
		 return outputMesh;
	 }
}