#pragma once

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

        // clustering decimation
		// @param <cell_size_percent>: [0.01, 1.0], BIG value leads to COARSE mesh
        // @param <target_mesh>: a TriMesh object to store the result
        // @return: just return the <target_mesh>
		trimesh::TriMesh* Perform(double cell_size_percent, trimesh::TriMesh* target_mesh);

        // quadric edge collapse decimation
        // @param <percent>: indicates the <target face number> = <percent> * <origin face number>
        // @param <target_mesh>: a TriMesh object to store the result
        // @return: just return the <target_mesh>
        trimesh::TriMesh* Perform_quadric(double percent, trimesh::TriMesh* target_mesh);

	private:
		VcgTriMesh *myVcgMesh;
	};
}