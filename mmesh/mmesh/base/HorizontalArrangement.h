#pragma once
#include "MMeshT.h"
#include "iostream"
#include "algorithm"
#include "numeric"
#include "mmesh/trimesh/trimeshutil.h"
#include "trimesh2/quaternion.h"

namespace mmesh 
{
	void getConvexHullMesh(trimesh::TriMesh* mesh, std::vector<trimesh::TriMesh*>& meshs);
	trimesh::fxform adjustmentMesh(trimesh::TriMesh* inmesh ,trimesh::vec3& normal);
	bool checkMesh(trimesh::TriMesh* inmesh, trimesh::vec3& normal);
	void indentationMesh(trimesh::TriMesh* inmesh);
	void triangularization(std::vector<trimesh::TriMesh*>& meshs);
}