#pragma once
#include "trimesh2/Vec.h"
#include "trimesh2/TriMesh.h"
#include "MMestFace.h"
#include "MMeshVertex.h"
#include "FPoint3.h"

/*Maybe only head file ?*/
namespace mmesh 
{
	class MMeshT
	{
	public:
		MMeshT(){};
		MMeshT(const MMeshT& mt) {};
		MMeshT(MMeshT& mt) {};	
		MMeshT& operator=(MMeshT mt) {};
		virtual ~MMeshT() {};

		std::vector<MMeshVertex> vertices;
		std::vector<MMeshFace> faces;
	public:
		static inline double det(trimesh::point& p0, trimesh::point& p1, trimesh::point& p2);
		inline double det(int faceIndex);
		inline double det(int VertexIndex1, int VertexIndex2, int VertexIndex3);
		static int getFaceIdxWithPoints(MMeshT* ameshtest, int idx0, int idx1, int notFaceIdx, int notFaceVertexIdx);
		void trimesh2meshtest(trimesh::TriMesh* currentMesh);

		inline int VN() { return vn; }
		inline int FN() { return fn; }

	private:
		int vn=0;
		int fn=0;
		
	};

	double  getTotalArea(std::vector<trimesh::point>& inVertices);	
}