#ifndef MMESH_BOOLEANSERAIL_1623205107388_H
#define MMESH_BOOLEANSERAIL_1623205107388_H
#include "trimesh2/TriMesh.h"
#include <fstream>

namespace mmesh
{
	void saveBoolean(const std::string& fileName, trimesh::TriMesh* mesh1, trimesh::TriMesh* mesh2);
	void loadBoolean(const std::string& fileName, trimesh::TriMesh*& mesh1, trimesh::TriMesh*& mesh2);

	void saveTriMesh(trimesh::TriMesh* mesh, std::fstream& out);
	void loadTriMesh(trimesh::TriMesh* mesh, std::fstream& in);

	template<class T>
	void saveVectorT(std::vector<T>& datas, std::fstream& out)
	{
		int size = datas.size();
		out.write((const char*)&size, sizeof(int));
		if (size > 0)
			out.write((const char*)&datas.at(0), size * sizeof(T));
	}

	template<class T>
	void loadVectorT(std::vector<T>& datas, std::fstream& in)
	{
		int size = 0;
		in.read((char*)&size, sizeof(int));
		if (size > 0)
		{
			datas.resize(size);
			in.read((char*)&datas.at(0), size * sizeof(T));
		}
	}
}

#endif // MMESH_BOOLEANSERAIL_1623205107388_H