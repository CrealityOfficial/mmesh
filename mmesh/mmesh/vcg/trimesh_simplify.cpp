#include "trimesh_simplify.h"

#include "trimesh2/TriMesh.h"
#include "vcg/complex/complex.h"
#include "vcg/complex/algorithms/create/platonic.h"
#include "vcg/complex/algorithms/clustering.h"
#include "vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h"
#include "wrap/io_trimesh/export.h"


namespace mmesh
{
    class VcgVertex;
    class VcgEdge;
	class VcgFace;
	struct VcgUsedTypes : public vcg::UsedTypes<
        vcg::Use<VcgVertex>::AsVertexType,
        vcg::Use<VcgEdge>::AsEdgeType,
        vcg::Use<VcgFace>::AsFaceType> {};
	class VcgVertex : public vcg::Vertex<
        VcgUsedTypes,
        vcg::vertex::InfoOcf,
        vcg::vertex::Coord3f,
        vcg::vertex::BitFlags,
        vcg::vertex::Normal3f,
        vcg::vertex::Qualityf,
        vcg::vertex::Color4b,
        vcg::vertex::VFAdjOcf,
        vcg::vertex::MarkOcf,
        vcg::vertex::TexCoordfOcf,
        vcg::vertex::CurvaturefOcf,
        vcg::vertex::CurvatureDirfOcf,
        vcg::vertex::RadiusfOcf> {};
    class VcgEdge : public vcg::Edge<
        VcgUsedTypes,
        vcg::edge::BitFlags,
        vcg::edge::EVAdj,
        vcg::edge::EEAdj> {};
	class VcgFace : public vcg::Face<
        VcgUsedTypes,
        vcg::face::InfoOcf,
        vcg::face::VertexRef,
        vcg::face::BitFlags,
        vcg::face::Normal3f,
        vcg::face::QualityfOcf,
        vcg::face::MarkOcf,
        vcg::face::Color4bOcf,
        vcg::face::FFAdjOcf,
        vcg::face::VFAdjOcf,
        vcg::face::CurvatureDirfOcf,
        vcg::face::WedgeTexCoordfOcf> {};
	class VcgTriMesh : public vcg::tri::TriMesh<
        vcg::vertex::vector_ocf<VcgVertex>,
        vcg::face::vector_ocf<VcgFace>> {};

    typedef	vcg::SimpleTempData<
        VcgTriMesh::VertContainer,
        vcg::math::Quadric<double>> QuadricTemp;
    class QHelper
    {
    public:
        QHelper() {}
        static void Init() {}
        static vcg::math::Quadric<double> &Qd(VcgVertex &v) { return TD()[v]; }
        static vcg::math::Quadric<double> &Qd(VcgVertex *v) { return TD()[*v]; }
        static VcgVertex::ScalarType W(VcgVertex * /*v*/) { return 1.0; }
        static VcgVertex::ScalarType W(VcgVertex & /*v*/) { return 1.0; }
        static void Merge(VcgVertex & /*v_dest*/, VcgVertex const & /*v_del*/) {}
        static QuadricTemp* &TDp() { static QuadricTemp *td; return td; }
        static QuadricTemp &TD() { return *TDp(); }
    };
    typedef BasicVertexPair<VcgVertex> VertexPair;
    class MyTriEdgeCollapse : public vcg::tri::TriEdgeCollapseQuadric<
        VcgTriMesh,
        VertexPair,
        MyTriEdgeCollapse,
        QHelper> {
    public:
        typedef vcg::tri::TriEdgeCollapseQuadric< VcgTriMesh, VertexPair, MyTriEdgeCollapse, QHelper> TECQ;
        inline MyTriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp) :TECQ(p, i, pp) {}
    };

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
            if (mymesh->face[i].IsD())
            {
                continue;
            }

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

    // copy from the source code of meshlab
	trimesh::TriMesh* TrimeshSimplify::Perform(double cell_size_percent, trimesh::TriMesh* target_mesh)
	{
        if (!target_mesh)
        {
            return nullptr;
        }

		double myCellSize = myVcgMesh->bbox.Diag() * (cell_size_percent < 0.01 ? 0.01 : (cell_size_percent > 1.0 ? 1.0 : cell_size_percent));
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

		vcg_to_trimesh(&extracted_mesh, target_mesh);

		return target_mesh;
	}

    // copy from the source code of meshlab
    trimesh::TriMesh* TrimeshSimplify::Perform_quadric(double percent, trimesh::TriMesh* target_mesh)
    {
        if (!target_mesh)
        {
            return nullptr;
        }

        percent = percent < 0.01 ? 0.01 : (percent > 1.0 ? 1.0 : percent);

        // clear deleted flag
        for (size_t i = 0; i < myVcgMesh->face.size(); i++)
        {
            if (myVcgMesh->face[i].IsD())
            {
                myVcgMesh->face[i].ClearD();
            }
        }

        // helper
        vcg::math::Quadric<double> qzero;
        qzero.SetZero();
        QuadricTemp td(myVcgMesh->vert, qzero);
        QHelper::TDp() = &td;

        // topo
        myVcgMesh->vert.EnableVFAdjacency();
        myVcgMesh->face.EnableVFAdjacency();
        vcg::tri::UpdateTopology<VcgTriMesh>::VertexFace(*myVcgMesh);
        myVcgMesh->vert.EnableMark();
        vcg::tri::UpdateFlags<VcgTriMesh>::FaceBorderFromVF(*myVcgMesh);

        int target_face_num = percent * myVcgMesh->fn;

        // simplify
        vcg::tri::TriEdgeCollapseQuadricParameter pp;
        vcg::LocalOptimization<VcgTriMesh> session(*myVcgMesh, &pp);
        session.Init<MyTriEdgeCollapse>();
        session.SetTargetSimplices(target_face_num);
        session.SetTimeBudget(0.1);
        while (session.DoOptimization() && myVcgMesh->fn > target_face_num);
        session.Finalize<MyTriEdgeCollapse>();
        QHelper::TDp() = nullptr;

        // auto clean (repair)
        vcg::tri::Clean<VcgTriMesh>::RemoveFaceOutOfRangeArea(*myVcgMesh, 0);
        vcg::tri::Clean<VcgTriMesh>::RemoveDuplicateVertex(*myVcgMesh);
        vcg::tri::Clean<VcgTriMesh>::RemoveUnreferencedVertex(*myVcgMesh);

        // update box and normals
        vcg::tri::UpdateBounding<VcgTriMesh>::Box(*myVcgMesh);
        vcg::tri::UpdateNormal<VcgTriMesh>::PerFaceNormalized(*myVcgMesh);
        vcg::tri::UpdateNormal<VcgTriMesh>::PerVertexAngleWeighted(*myVcgMesh);
        vcg::tri::UpdateNormal<VcgTriMesh>::NormalizePerFace(*myVcgMesh);
        vcg::tri::UpdateNormal<VcgTriMesh>::PerVertexFromCurrentFaceNormal(*myVcgMesh);
        vcg::tri::UpdateNormal<VcgTriMesh>::NormalizePerVertex(*myVcgMesh);

        vcg_to_trimesh(myVcgMesh, target_mesh);

        return target_mesh;
    }
}