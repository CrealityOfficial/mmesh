#ifndef MMESH_CYLINDERGENERATOR_1610770732066_H
#define MMESH_CYLINDERGENERATOR_1610770732066_H
#include "trimesh2/Vec.h"
#include <vector>

namespace mmesh
{
	class CylinderGenerator
	{
	public:
		CylinderGenerator(int n, float theta);
		~CylinderGenerator();

		void generate(const trimesh::vec3& start, const trimesh::vec3& end, float radius,
			std::vector<trimesh::vec3>& triangles, bool withoutBU = true);
	protected:
		std::vector<trimesh::vec3> m_baseNormals;
		int m_hPart;
	};
}

#endif // MMESH_CYLINDERGENERATOR_1610770732066_H