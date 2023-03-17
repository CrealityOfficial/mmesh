#pragma once

#include "trimesh2/Vec.h"
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	class MMeshVertex
	{
	public:
		MMeshVertex() {};
		MMeshVertex(trimesh::point p):p(p) { connected_faces.reserve(8); };
		virtual ~MMeshVertex() {};

		trimesh::point p;
		std::vector<uint32_t> connected_faces;

	private:
		enum vertexflag
		{
			MV_DELETE = 0x00000001,
			MV_SELECT = 0x00000002,
			MV_BORDER = 0x00000004,
			MV_VISITED= 0x00000008
		};
		int flag = 0;
	public:
		inline void SetD() { flag |= MV_DELETE; }
		inline bool IsD() { return (MV_DELETE & flag) != 0 ? 1 : 0; }
		inline void ClearD() { flag &= ~MV_DELETE; }

		inline void SetS() { flag |= MV_SELECT; }
		inline bool IsS() { return (MV_SELECT & flag) != 0 ? 1 : 0; }
		inline void ClearS() { flag &= ~MV_SELECT; }

		inline void SetB() { flag |= MV_BORDER; }
		inline bool IsB() { return (MV_BORDER & flag) != 0 ? 1 : 0; }
		inline void ClearB() { flag &= ~MV_BORDER; }

		inline void SetV() { flag |= MV_VISITED; }
		inline bool IsV() { return (MV_VISITED & flag) != 0 ? 1 : 0; }
		inline void ClearV() { flag &= ~MV_VISITED; }
	};
}