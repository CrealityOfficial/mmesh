#pragma once
#include "trimesh2/Vec.h"
#include "trimesh2/TriMesh.h"


namespace mmesh
{
	class MMeshFace
	{
	public:
		MMeshFace() {};
		virtual ~MMeshFace() {};	
		int vertex_index[3] = { -1 };
		int connected_face_index[3];
		bool bUnique = true;
		bool bAdd = false;

		trimesh::point normal;

	private:
		enum faceflag
		{
			MF_DELETE = 0x00000001,
			MF_SELECT = 0x00000002,
			MF_BORDER = 0x00000004,
			MF_VISITED= 0x00000008
		};
		int flag=0;
	public:
		inline void SetD() { flag |= MF_DELETE; }
		inline bool IsD() { return (MF_DELETE & flag)!=0 ? 1 : 0; }
		inline void ClearD() { flag &= ~MF_DELETE; }

		inline void SetS() { flag |= MF_SELECT; }
		inline bool IsS() { return (MF_SELECT & flag) != 0 ? 1 : 0; }
		inline void ClearS() { flag &= ~MF_SELECT; }

		inline void SetB() { flag |= MF_BORDER; }
		inline bool IsB() { return (MF_BORDER & flag) != 0 ? 1 : 0; }
		inline void ClearB() { flag &= ~MF_BORDER; }

		inline void SetV() { flag |= MF_VISITED; }
		inline bool IsV() { return (MF_VISITED & flag) != 0 ? 1 : 0; }
		inline void ClearV() { flag &= ~MF_VISITED; }
	};
}