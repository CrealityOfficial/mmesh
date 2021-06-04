#pragma once
#include "polygonstack.h"


namespace mmesh
{
	class PolygonStackEx : public PolygonStack
	{
	public:
		PolygonStackEx();
		~PolygonStackEx();

		void generates(std::vector<std::vector<int>>& polygons, std::vector<trimesh::dvec2>& points, trimesh::TriMesh* mesh);

	};
}