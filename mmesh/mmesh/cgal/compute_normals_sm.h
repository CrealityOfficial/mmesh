#ifndef __COMPUTE_NORMALS_SM_H__
#define __COMPUTE_NORMALS_SM_H__

#if defined(WIN32) && defined(USE_CGAL)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <iostream>
#include <fstream>

namespace ComputeNormalsSM
{
	typedef CGAL::Simple_cartesian<float>	K;
	typedef K::Point_3						Point;
	typedef K::Vector_3						Vector;

	typedef CGAL::Surface_mesh<Point>		Surface_mesh;
	typedef Surface_mesh::Vertex_index		vertex_descriptor;
	typedef Surface_mesh::halfedge_index	halfedge_descriptor;
	typedef Surface_mesh::Face_index		face_descriptor;
	typedef Surface_mesh::Property_map<face_descriptor, Vector> fnormalsMap;
	typedef CGAL::GetVertexPointMap<Surface_mesh>::const_type   VPMap;




	void getFaceNormals(Surface_mesh* meshPtr, fnormalsMap & fnormals);
}    
#endif

#endif