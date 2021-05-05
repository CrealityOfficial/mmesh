#ifndef CREATIVE_KERNEL_MESHTOPO_1596006983404_H
#define CREATIVE_KERNEL_MESHTOPO_1596006983404_H
#include "trimesh2/Vec.h"
#include "trimesh2/TriMesh.h"

namespace mmesh
{
	class MeshTopo 
	{
	public:
		MeshTopo();
		~MeshTopo();

		inline int halfcode(int face, int index)
		{
			return (face << 2) + index;
		}
		inline void halfdecode(int half, int& face, int& index)
		{
			index = half & 3;
			face = half >> 2;
		}

		inline int faceid(int half)
		{
			return half >> 2;
		}

		inline int startvertexid(int half)
		{
			int face = half >> 2;
			int idx = half & 3;
			return m_mesh->faces.at(face)[idx];
		}

		inline int endvertexid(int half)
		{
			int face = half >> 2;
			int idx = ((half & 3) + 1) % 3;
			return m_mesh->faces.at(face)[idx];
		}

		bool builded();
		void build(trimesh::TriMesh* mesh);

		void lowestVertex(std::vector<trimesh::vec3>& vertexes, std::vector<int>& indices);
		void hangEdge(std::vector<trimesh::vec3>& vertexes, std::vector<trimesh::vec3>& normals, std::vector<float>& dotValues, float faceCosValue, std::vector<trimesh::ivec2>& edges);
		void chunkFace(std::vector<float>& dotValues, std::vector<std::vector<int>>& faces, float faceCosValue);
		void flipFace(std::vector<int>& facesChunk);

		trimesh::TriMesh* m_mesh;

		std::vector<std::vector<int>> m_outHalfEdges;//存储某点相关联的面以及第三个点，其中int 组合成面和三个点值
		std::vector<trimesh::ivec3> m_oppositeHalfEdges;//存储某个面相关的三个顶组合成的int值
		bool m_topoBuilded;
	};
}
#endif // CREATIVE_KERNEL_MESHTOPO_1596006983404_H