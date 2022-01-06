#ifndef _MESHREPAIR_1638265408178_H
#define _MESHREPAIR_1638265408178_H
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	class TriMeshRepair {
	public:
		struct face_Faces {
			std::vector<int> f0;
			std::vector<int> f1;
			std::vector<int> f2;
		};

		TriMeshRepair() {};
		virtual ~TriMeshRepair();
		//
		trimesh::TriMesh* repair(trimesh::TriMesh* mesh);

	public:
		void need_normalsFaces(bool simple_area_weighted = false);
		void removeNorFaces();
		void remove_unconnected_facets();
		void fix_normal_directions();
		void fix_volume();
		void reverse_facet(int facet_num, bool update = true);
		void need_face_faces();
		void need_across_edge2();
		float get_volume();
		float get_area(int facet);
		void clears();
	private:
		std::vector<face_Faces> face_faces; //face to faces
		//std::vector<trimesh::TriMesh::Face> face_faces_nums; //face to faces
		std::vector<trimesh::vec> normalsFaces;
		std::shared_ptr<trimesh::TriMesh> m_mesh;
	};
}

#endif // _MESHREPAIR_1638265408178_H