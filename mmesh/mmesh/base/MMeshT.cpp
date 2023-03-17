#include "MMeshT.h"

namespace mmesh
{
	inline double MMeshT::det(trimesh::point& p0, trimesh::point& p1, trimesh::point& p2)
	{
		trimesh::vec3 a = p0 - p1;
		trimesh::vec3 b = p1 - p2;
		return sqrt(pow((a.y * b.z - a.z * b.y), 2) + pow((a.z * b.x - a.x * b.z), 2)
			+ pow((a.x * b.y - a.y * b.x), 2)) / 2.0f;
	}

	inline double MMeshT::det(int VertexIndex1, int VertexIndex2, int VertexIndex3)
	{
		trimesh::point p0 = this->vertices[VertexIndex1].p;
		trimesh::point p1 = this->vertices[VertexIndex2].p;
		trimesh::point p2 = this->vertices[VertexIndex3].p;
		trimesh::vec3 a = p0 - p1;
		trimesh::vec3 b = p1 - p2;
		return sqrt(pow((a.y * b.z - a.z * b.y), 2) + pow((a.z * b.x - a.x * b.z), 2)
			+ pow((a.x * b.y - a.y * b.x), 2)) / 2.0f;
	}

	inline double MMeshT::det(int faceIndex)
	{
		MMeshFace f = this->faces[faceIndex];
		return MMeshT::det(f.vertex_index[0], f.vertex_index[1], f.vertex_index[2]);
	}

	int MMeshT::getFaceIdxWithPoints(MMeshT* ameshtest, int idx0, int idx1, int notFaceIdx, int notFaceVertexIdx)
	{
		std::vector<int> candidateFaces;
		for (int f : ameshtest->vertices[idx0].connected_faces)
		{
			if (f == notFaceIdx)
			{
				continue;
			}
			if (ameshtest->faces[f].vertex_index[0] == idx1
				|| ameshtest->faces[f].vertex_index[1] == idx1
				|| ameshtest->faces[f].vertex_index[2] == idx1
				)  candidateFaces.push_back(f);

		}

		if (candidateFaces.size() == 0)
		{
			// has_disconnected_faces = true;
			return -1;
		}
		if (candidateFaces.size() == 1) { return candidateFaces[0]; }


		if (candidateFaces.size() % 2 == 0)
		{
			//cura::logDebug("Warning! Edge with uneven number of faces connecting it!(%i)\n", candidateFaces.size() + 1);
			//has_disconnected_faces = true;
		}

		FPoint3 vn = ameshtest->vertices[idx1].p - ameshtest->vertices[idx0].p;
		FPoint3 n = vn / vn.vSize(); // the normal of the plane in which all normals of faces connected to the edge lie => the normalized normal
		FPoint3 v0 = ameshtest->vertices[idx1].p - ameshtest->vertices[idx0].p;

		// the normals below are abnormally directed! : these normals all point counterclockwise (viewed from idx1 to idx0) from the face, irrespective of the direction of the face.
		FPoint3 n0 = FPoint3(ameshtest->vertices[notFaceVertexIdx].p - ameshtest->vertices[idx0].p).cross(v0);

		if (n0.vSize() <= 0)
		{
			//cura::logDebug("Face %i has zero area!", notFaceIdx);
		}

		double smallestAngle = 1000; // more then 2 PI (impossible angle)
		int bestIdx = -1;

		for (int candidateFace : candidateFaces)
		{
			int candidateVertex;
			{// find third vertex belonging to the face (besides idx0 and idx1)
				for (candidateVertex = 0; candidateVertex < 3; candidateVertex++)
					if (ameshtest->faces[candidateFace].vertex_index[candidateVertex] != idx0 && ameshtest->faces[candidateFace].vertex_index[candidateVertex] != idx1)
						break;
			}

			FPoint3 v1 = ameshtest->vertices[ameshtest->faces[candidateFace].vertex_index[candidateVertex]].p - ameshtest->vertices[idx0].p;
			FPoint3 n1 = v0.cross(v1);

			double dot = n0 * n1;
			double det = n * n0.cross(n1);
			double angle = std::atan2(det, dot);
#define  M_PI 3.1415926
			if (angle < 0) angle += 2 * M_PI; // 0 <= angle < 2* M_PI

			if (angle == 0)
			{
				//cura::logDebug("Overlapping faces: face %i and face %i.\n", notFaceIdx, candidateFace);
				//has_overlapping_faces = true;
			}
			if (angle < smallestAngle)
			{
				smallestAngle = angle;
				bestIdx = candidateFace;
			}
		}
		if (bestIdx < 0)
		{
			//cura::logDebug("Couldn't find face connected to face %i.\n", notFaceIdx);
			//has_disconnected_faces = true;
		}
		return bestIdx;
	}

	void MMeshT::trimesh2meshtest(trimesh::TriMesh* currentMesh)
	{
		for (trimesh::point apoint : currentMesh->vertices)
		{		
			this->vertices.push_back(apoint);
		}

		for (unsigned int i = 0; i < currentMesh->faces.size(); i++)
		{
			trimesh::TriMesh::Face& face = currentMesh->faces[i];
			this->faces.emplace_back();
			this->faces[i].vertex_index[0] = face.at(0);
			this->faces[i].vertex_index[1] = face.at(1);
			this->faces[i].vertex_index[2] = face.at(2);

			this->vertices[face.at(0)].connected_faces.push_back(i);
			this->vertices[face.at(1)].connected_faces.push_back(i);
			this->vertices[face.at(2)].connected_faces.push_back(i);

			this->faces[i].normal = trimesh::normalized(trimesh::trinorm(this->vertices[face.at(0)].p,
																			this->vertices[face.at(1)].p,
																			this->vertices[face.at(2)].p));

		}

		for (unsigned int i = 0; i < currentMesh->faces.size(); i++)
		{
			trimesh::TriMesh::Face& face = currentMesh->faces[i];
			// faces are connected via the outside
			this->faces[i].connected_face_index[0] = MMeshT::getFaceIdxWithPoints(this, face.at(0), face.at(1), i, face.at(2));
			this->faces[i].connected_face_index[1] = MMeshT::getFaceIdxWithPoints(this, face.at(1), face.at(2), i, face.at(0));
			this->faces[i].connected_face_index[2] = MMeshT::getFaceIdxWithPoints(this, face.at(2), face.at(0), i, face.at(1));
		}
	}

	double  getTotalArea(std::vector<trimesh::point>& inVertices)
	{
		double sum = 0.0f;
		for (int n = 1; n < inVertices.size() - 1; n++)
		{
			sum += MMeshT::det(inVertices[0], inVertices[n], inVertices[n + 1]);
		}
		return abs(sum);
	}
	
}