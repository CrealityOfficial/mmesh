#include "trimesh_simplify.h"

#include "trimesh2/TriMesh.h"

#ifdef USE_VCG
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/create/platonic.h>
#include "vcg/complex/algorithms/clustering.h"
#include "wrap/io_trimesh/export.h"

namespace mmesh
{
	class VcgFace;
	class VcgVertex;
	struct VcgUsedTypes : public vcg::UsedTypes<vcg::Use<VcgVertex>::AsVertexType, vcg::Use<VcgFace>::AsFaceType> {};
	class VcgVertex : public vcg::Vertex<VcgUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::Color4b, vcg::vertex::BitFlags> {};
	class VcgFace : public vcg::Face<VcgUsedTypes, vcg::face::VertexRef, vcg::face::Normal3f, vcg::face::BitFlags> {};
	class VcgTriMesh : public vcg::tri::TriMesh<std::vector<VcgVertex>, std::vector<VcgFace>> {};

	// convert "trimesh2" mesh to "vcglib" mesh
	static void trimesh_to_vcg(trimesh::TriMesh *trimesh, VcgTriMesh *mymesh)
	{
		mymesh->Clear();

		std::vector<vcg::Point3f> verts;
		std::vector<vcg::Point3i> faces;

		for (size_t i = 0; i < trimesh->vertices.size(); i++)
		{
			verts.emplace_back(trimesh->vertices[i].x, trimesh->vertices[i].y, trimesh->vertices[i].z);
		}

		for (size_t i = 0; i < trimesh->faces.size(); i++)
		{
			faces.emplace_back(trimesh->faces[i][0], trimesh->faces[i][1], trimesh->faces[i][2]);
		}

		vcg::tri::BuildMeshFromCoordVectorIndexVector(*mymesh, verts, faces);
	}

	static void vcg_to_trimesh(VcgTriMesh *mymesh, trimesh::TriMesh *trimesh)
	{
		trimesh->clear();

		int vertex_id_start = 0;

		for (size_t i = 0; i < mymesh->face.size(); i++)
		{
			vertex_id_start = trimesh->vertices.size();

			trimesh->vertices.emplace_back(mymesh->face[i].V(0)->P().X(), mymesh->face[i].V(0)->P().Y(), mymesh->face[i].V(0)->P().Z());
			trimesh->vertices.emplace_back(mymesh->face[i].V(1)->P().X(), mymesh->face[i].V(1)->P().Y(), mymesh->face[i].V(1)->P().Z());
			trimesh->vertices.emplace_back(mymesh->face[i].V(2)->P().X(), mymesh->face[i].V(2)->P().Y(), mymesh->face[i].V(2)->P().Z());

			trimesh->faces.emplace_back(vertex_id_start, vertex_id_start + 1, vertex_id_start + 2);
		}
	}


	TrimeshSimplify::TrimeshSimplify(trimesh::TriMesh* mesh)
	{
		myVcgMesh = new VcgTriMesh();
		trimesh_to_vcg(mesh, myVcgMesh);
	}

	TrimeshSimplify::~TrimeshSimplify()
	{
		for (size_t i = 0; i < myVcgMesh->face.size(); i++)
		{
			vcg::tri::Allocator<VcgTriMesh>::DeleteFace(*myVcgMesh, myVcgMesh->face[i]);
		}

		for (size_t i = 0; i < myVcgMesh->vert.size(); i++)
		{
			vcg::tri::Allocator<VcgTriMesh>::DeleteVertex(*myVcgMesh, myVcgMesh->vert[i]);
		}

		delete myVcgMesh;
	}

	trimesh::TriMesh* TrimeshSimplify::Perform(double percent)
	{
		double myCellSize = myVcgMesh->bbox.Diag() * (percent < 0.01 ? 0.01 : (percent > 1.0 ? 1.0 : percent));
		vcg::tri::Clustering<VcgTriMesh, vcg::tri::AverageColorCell<VcgTriMesh>> clustering;

		clustering.Init(myVcgMesh->bbox, 100000, myCellSize);

		if (myVcgMesh->FN() == 0)
		{
			clustering.AddPointSet(*myVcgMesh);
		}
		else
		{
			clustering.AddMesh(*myVcgMesh);
		}

		VcgTriMesh extracted_mesh;
		clustering.ExtractMesh(extracted_mesh);

		trimesh::TriMesh *output_mesh = new trimesh::TriMesh();
		vcg_to_trimesh(&extracted_mesh, output_mesh);

		return output_mesh;
	}
}

#endif