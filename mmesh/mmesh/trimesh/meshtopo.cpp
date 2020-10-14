#include "mmesh/trimesh/meshtopo.h"

using namespace trimesh;
namespace mmesh
{
	MeshTopo::MeshTopo()
		:m_topoBuilded(false)
		, m_mesh(nullptr)
	{
	}
	
	MeshTopo::~MeshTopo()
	{
	}

	bool MeshTopo::builded()
	{
		return m_topoBuilded;
	}

	void MeshTopo::build(TriMesh* mesh)
	{
		m_mesh = mesh;
		if (m_mesh && !m_topoBuilded)
		{
			int vertexNum = (int)m_mesh->vertices.size();
			int faceNum = (int)m_mesh->faces.size();
			//build topo
			if (vertexNum) m_outHalfEdges.resize(vertexNum);
			if (faceNum) m_oppositeHalfEdges.resize(faceNum, ivec3(-1, -1, -1));
			for (int i = 0; i < faceNum; ++i)
			{
				TriMesh::Face& f = m_mesh->faces.at(i);
				bool old[3] = { false };
				for (int j = 0; j < 3; ++j)
				{
					std::vector<int>& outHalfs = m_outHalfEdges.at(f[j]);
					old[j] = outHalfs.size() > 0;
				}

				ivec3& halfnew = m_oppositeHalfEdges.at(i);
				for (int j = 0; j < 3; ++j)
				{
					int start = j;
					int end = (j + 1) % 3;

					int v2 = f[start];
					int v1 = f[end];

					std::vector<int>& halfs1 = m_outHalfEdges.at(v1);

					int nh = halfcode(i, j);
					if (old[start] && old[end])
					{
						int hsize = (int)halfs1.size();
						for (int k = 0; k < hsize; ++k)
						{
							int h = halfs1.at(k);
							if (endvertexid(h) == v2)
							{
								halfnew.at(j) = h;

								int faceid = 0;
								int idxid = 0;
								halfdecode(h, faceid, idxid);
								m_oppositeHalfEdges.at(faceid)[idxid] = nh;
								break;
							}
						}
					}
				}

				for (int j = 0; j < 3; ++j)
				{
					std::vector<int>& outHalfs = m_outHalfEdges.at(f[j]);
					outHalfs.push_back(halfcode(i, j));
				}
			}

			m_topoBuilded = true;
		}
	}

	void MeshTopo::lowestVertex(std::vector<trimesh::vec3>& vertexes, std::vector<int>& indices)
	{
		int vertexNum = vertexes.size();
		for (int vertexID = 0; vertexID < vertexNum; ++vertexID)
		{
			std::vector<int>& vertexHalfs = m_outHalfEdges.at(vertexID);
			int halfSize = (int)vertexHalfs.size();
			vec3& vertex = vertexes.at(vertexID);

			bool lowest = true;
			for (int halfIndex = 0; halfIndex < halfSize; ++halfIndex)
			{
				int endVertexID = endvertexid(vertexHalfs.at(halfIndex));
				if (vertexes.at(endVertexID).z < vertex.z)
				{
					lowest = false;
					break;
				}
			}
			if (lowest)
			{
				indices.push_back(vertexID);
			}
		}
	}

	void MeshTopo::hangEdge(std::vector<trimesh::vec3>& vertexes, std::vector<trimesh::vec3>& normals, std::vector<float>& dotValues, std::vector<trimesh::ivec2>& supportEdges)
	{
		int faceNum = (int)m_mesh->faces.size();
		std::vector<ivec3> edgesFlags(faceNum, ivec3(0, 0, 0));
		float edgeCosValue = cosf(M_PIf * 70.0f / 180.0f);
		float edgeFaceCosValue = cosf(M_PIf * 60.0f / 180.0f);
		for (int faceID = 0; faceID < faceNum; ++faceID)
		{
			ivec3& oppoHalfs = m_oppositeHalfEdges.at(faceID);
			ivec3& edgeFlag = edgesFlags.at(faceID);
			TriMesh::Face& tFace = m_mesh->faces.at(faceID);
			vec3& faceNormal = normals.at(faceID);

			bool faceSupport = dotValues.at(faceID) < (-edgeFaceCosValue);
			for (int edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
			{
				if (edgeFlag[edgeIndex] == 0)
				{
					edgeFlag[edgeIndex] = 1;

					int vertexID1 = tFace[edgeIndex];
					int vertexID2 = tFace[(edgeIndex + 1) % 3];
					vec3 edge = vertexes.at(vertexID1) - vertexes.at(vertexID2);
					vec3 nedge = normalized(edge);

					if (abs(trimesh::dot(nedge, vec3(0.0f, 0.0f, 1.0f))) < edgeCosValue)
					{
						int oppoHalf = oppoHalfs.at(edgeIndex);
						bool shouldAdd = false;
						if (oppoHalf >= 0)
						{
							int oppoFaceID;
							int edgeID;
							halfdecode(oppoHalf, oppoFaceID, edgeID);
							edgesFlags.at(oppoFaceID)[edgeID] = 1;

							vec3& oppoFaceNormal = normals.at(oppoFaceID);
							bool oppoFaceSupport = dotValues.at(oppoFaceID) < (-edgeFaceCosValue);
							if (oppoFaceSupport && faceSupport)
							{

								if (trimesh::dot(faceNormal, oppoFaceNormal) < 0.0f)
								{
									shouldAdd = true;
								}
							}
						}
						else  // hole edge
						{
							shouldAdd = faceSupport;
						}

						if (shouldAdd)
						{
							ivec2 edgeID(vertexID1, vertexID2);
							supportEdges.push_back(edgeID);
						}
					}
				}
			}
		}
	}

	void MeshTopo::chunkFace(std::vector<float>& dotValues, std::vector<std::vector<int>>& supportFaces)
	{
		int faceNum = (int)m_mesh->faces.size();
		std::vector<bool> visitFlags(faceNum, false);
		std::vector<int> visitStack;
		std::vector<int> nextStack;
		float faceCosValue = cosf(M_PIf * 30.0f / 180.0f);
		for (int faceID = 0; faceID < faceNum; ++faceID)
		{
			if (dotValues.at(faceID) < -faceCosValue && visitFlags.at(faceID) == false)
			{
				visitFlags.at(faceID) = true;
				visitStack.push_back(faceID);

				std::vector<int> facesChunk;
				facesChunk.push_back(faceID);
				while (!visitStack.empty())
				{
					int seedSize = (int)visitStack.size();
					for (int seedIndex = 0; seedIndex < seedSize; ++seedIndex)
					{
						int cFaceID = visitStack.at(seedIndex);
						ivec3& oppoHalfs = m_oppositeHalfEdges.at(cFaceID);
						for (int halfID = 0; halfID < 3; ++halfID)
						{
							int oppoHalf = oppoHalfs.at(halfID);
							if (oppoHalf >= 0)
							{
								int oppoFaceID = faceid(oppoHalf);
								if (dotValues.at(oppoFaceID) < -faceCosValue && (visitFlags.at(oppoFaceID) == false))
								{
									nextStack.push_back(oppoFaceID);
									facesChunk.push_back(oppoFaceID);
									visitFlags.at(oppoFaceID) = true;
								}
								else
								{
									visitFlags.at(oppoFaceID) = true;
								}
							}
						}
					}

					visitStack.swap(nextStack);
					nextStack.clear();
				}

				supportFaces.push_back(facesChunk);
			}
			else
			{
				visitFlags.at(faceID) = true;
			}
		}
	}
}