#include "pickplane.h"
#include "ccglobal/tracer.h"

#include "ccglobal/spycc.h"


namespace mmesh
{

	void pickPlane(trimesh::TriMesh* mesh, trimesh::vec3 positon, trimesh::vec3 normal, std::vector<int>& faceIndex, ccglobal::Tracer* tracer)
	{
		if (!mesh)
		{
			return;
		}
		mesh->clear_normals();
		mesh->need_normals();

		int faceNum = mesh->faces.size();
		std::vector<trimesh::vec>	fn(faceNum);

		std::vector<int> sameNormal;
		for (size_t i = 0; i < faceNum; i++)
		{
			trimesh::TriMesh::Face& f = mesh->faces[i];
			fn[i]= trimesh::normalized(trimesh::trinorm(mesh->vertices[f.x], mesh->vertices[f.y], mesh->vertices[f.z]));
			if (fn[i] == normal)
			{
				int a = 0;
			}
		}


	}

	trimesh::TriMesh::Face checkChild(trimesh::vec3& curFn, trimesh::vec3& fn1, trimesh::vec3& fn2, trimesh::vec3& fn3)
	{
		trimesh::TriMesh::Face f(-1,-1,-1);

		if (std::abs(curFn.at(0) - fn1.at(0)) < 0.01
			&& std::abs(curFn.at(1) - fn1.at(1)) < 0.01
			&& std::abs(curFn.at(2) - fn1.at(2)) < 0.01)
		{
			f.x = 1;
		}
		if (std::abs(curFn.at(0) - fn2.at(0)) < 0.01
			&& std::abs(curFn.at(1) - fn2.at(1)) < 0.01
			&& std::abs(curFn.at(2) - fn2.at(2)) < 0.01)
		{
			f.y = 1;
		}
		if (std::abs(curFn.at(0) - fn3.at(0)) < 0.01
			&& std::abs(curFn.at(1) - fn3.at(1)) < 0.01
			&& std::abs(curFn.at(2) - fn3.at(2)) < 0.01)
		{
			f.z = 1;
		}

		return f;
	}

	bool existedElement(std::vector<int>& fIndexSamePlane, int chindf)
	{
		for (size_t j = 0; j < fIndexSamePlane.size(); j++)
		{
			if (fIndexSamePlane[j] == chindf)
				return true;
		}
		return false;
	}

	void pickPlane(trimesh::TriMesh* mesh, int& curFaceIndex, std::vector<int>& faceIndex, ccglobal::Tracer* tracer)
	{
		if (!mesh)
			return;
		if (curFaceIndex < 0 || curFaceIndex > mesh->faces.size())
			return;

		mesh->clear_normals();
		mesh->need_normals();
		mesh->need_across_edge();

		int faceNum = mesh->faces.size();
		std::vector<trimesh::vec3>	fn(faceNum);

		for (size_t i = 0; i < faceNum; i++)
		{
			trimesh::TriMesh::Face& f = mesh->faces[i];
			fn[i] = trimesh::normalized(trimesh::trinorm(mesh->vertices[f.x], mesh->vertices[f.y], mesh->vertices[f.z]));
		}

		std::vector<int> fIndexCnt;
		fIndexCnt.push_back(curFaceIndex);
		faceIndex.push_back(curFaceIndex);
		while (fIndexCnt.size())
		{
			int i = fIndexCnt.front();
			trimesh::TriMesh::Face& f = mesh->across_edge[i];
			trimesh::TriMesh::Face fchind = checkChild(fn[i], fn[f.x], fn[f.y], fn[f.z]);
			if (fchind.x > -1
				&& !existedElement(faceIndex, f.x))
			{

				fIndexCnt.push_back(f.x);
				faceIndex.push_back(f.x);
			}
			if (fchind.y > -1  
				&& !existedElement(faceIndex, f.y))
			{
				fIndexCnt.push_back(f.y);
				faceIndex.push_back(f.y);
			}
			if (fchind.z > -1
				&& !existedElement(faceIndex, f.z))
			{
				fIndexCnt.push_back(f.z);
				faceIndex.push_back(f.z);
			}
			fIndexCnt.erase(fIndexCnt.begin());
		}	
	}
}