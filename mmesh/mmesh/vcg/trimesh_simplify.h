#pragma once

#ifdef USE_VCG

namespace trimesh
{
	class TriMesh;
}

namespace mmesh
{
	class VcgTriMesh;

	class TrimeshSimplify
	{
	public:
		// ref: MeshLab => "Filters" -> "Remeshing, Simplification and Reconstruction" -> "Simplification: Clustering Decimation"
		TrimeshSimplify(trimesh::TriMesh* mesh);
		~TrimeshSimplify();

		// @param <percent>: [0.01, 1.0], big value leads to coarse mesh
		// @return: a new trimesh::TriMesh object (ALLOCATED in this function!)
		trimesh::TriMesh* Perform(double percent);

	private:
		VcgTriMesh *myVcgMesh;
	};
}

#endif