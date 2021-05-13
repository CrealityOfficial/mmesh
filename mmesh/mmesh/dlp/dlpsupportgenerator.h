#ifndef MMESH_DLPSUPPORTGENERATOR_1619594835240_H
#define MMESH_DLPSUPPORTGENERATOR_1619594835240_H
#include <trimesh2/TriMesh.h>

namespace mmesh
{
	struct DLPSupportGeneratorParam
	{
		bool pad_enable;
		int support_points_density_relative;
		float support_points_minimal_distance;

		DLPSupportGeneratorParam()
		{
			pad_enable = true;
			support_points_density_relative = 100;
			support_points_minimal_distance = 5.0f;
		}
	};

	class DLPSupportGenerator
	{
	public:
		DLPSupportGenerator();
		~DLPSupportGenerator();

		void setInput(trimesh::TriMesh* mesh);
		void setParam(const DLPSupportGeneratorParam& param);
		trimesh::TriMesh* generate();
	protected:
		trimesh::TriMesh* generateSupport(trimesh::TriMesh* mesh);
		trimesh::TriMesh* generatePad();

	protected:
		DLPSupportGeneratorParam m_param;
		trimesh::TriMesh* m_mesh;
	};
}

#endif // MMESH_DLPSUPPORTGENERATOR_1619594835240_H