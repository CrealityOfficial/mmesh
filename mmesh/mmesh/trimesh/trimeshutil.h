#ifndef MMESH_TRIMESHUTIL_1602590289222_H
#define MMESH_TRIMESHUTIL_1602590289222_H
#include <vector>
#include <fstream>
#include <memory>

#include "trimesh2/XForm.h"
#include "trimesh2/Box.h"

namespace trimesh
{
	class TriMesh;
}
typedef std::shared_ptr<trimesh::TriMesh> TriMeshPointer;

namespace ccglobal
{
	class Tracer;
}

struct edge {
	trimesh::vec3 p0;
	trimesh::vec3 p1;
};

namespace mmesh
{
	void mergeTriMesh(trimesh::TriMesh* outMesh, const std::vector<trimesh::TriMesh*>& inMeshes, bool fanzhuan = false);
	void mergeTriMesh_omp(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes);
	trimesh::TriMesh* LinePlusline2Model(std::vector<edge>& edge, float _radius);
	void reverseTriMesh(trimesh::TriMesh* Mesh);

	void mergeTrianglesTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes); // inMeshes is triangles

	trimesh::TriMesh* partMesh(const std::vector<int>& indices, trimesh::TriMesh* inMesh);

	bool testNeedfitMesh(trimesh::TriMesh* mesh, float& scale);
	bool dumplicateMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer = nullptr, const float& ratio = 0.3f);
	bool dumplicateMeshExTest(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer = nullptr, const float& ratio = 0.3f);

	void removeNorFaces(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer = nullptr);
	void removeInvalidVertex(trimesh::TriMesh* mesh);

	void mergeTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes, const trimesh::fxform& globalMatrix, bool fanzhuan);

	trimesh::vec3 moveTrimesh2Center(trimesh::TriMesh* mesh, bool zZero = true);
	trimesh::vec3 moveToOriginal(trimesh::TriMesh* mesh);
	void moveMeshes2BoxCenter(std::vector<trimesh::TriMesh*> meshes, const trimesh::box3& box, bool zZero = true);

	void convertUV2Azi(trimesh::TriMesh* mesh);
	void convertUV2Equ(trimesh::TriMesh* mesh);

	void flipZ(trimesh::TriMesh* mesh);

	trimesh::fxform beltXForm(const trimesh::vec3& offset, float angle,int beltType=1);
	trimesh::fxform xformFromPlane(const trimesh::vec3& pos, const trimesh::vec3& normal);
	void fillTriangleSoupFaceIndex(trimesh::TriMesh* mesh);
	void indexTriangle2Soup(trimesh::TriMesh* mesh);

	void meshMerge(TriMeshPointer& outMesh,const std::vector<TriMeshPointer>& meshes, const trimesh::fxform& globalMatrix, bool fanzhuan, ccglobal::Tracer* tracer);
	std::vector<std::vector<TriMeshPointer>> meshSplit(const std::vector<TriMeshPointer>& meshes, ccglobal::Tracer* tracer = nullptr);

	void loadTrimesh(std::fstream& in, trimesh::TriMesh& mesh);
	void saveTrimesh(std::fstream& out, trimesh::TriMesh& mesh);

	template<class T>
	void loadT(std::fstream& in, T& t)
	{
		in.read((char*)&t, sizeof(T));
	}

	template<class T>
	void saveT(std::fstream& out, T& t)
	{
		out.write((const char*)&t, sizeof(T));
	}

	template<class T>
	void loadVectorT(std::fstream& in, std::vector<T>& vecs)
	{
		int num = 0;
		loadT(in, num);
		if (num > 0)
		{
			vecs.resize(num);
			in.read((char*)&vecs.at(0), num * sizeof(T));
		}
	}

	template<class T>
	void saveVectorT(std::fstream& out, std::vector<T>& vecs)
	{
		int num = (int)vecs.size();
		saveT(out, num);
		if (num > 0)
			out.write((const char*)&vecs.at(0), num * sizeof(T));
	}

	trimesh::dvec3 vec32dvec3(const trimesh::vec3& v);
}

#endif // MMESH_TRIMESHUTIL_1602590289222_H