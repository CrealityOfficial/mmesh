#include "dumplicate.h"
#include "ccglobal/tracer.h"

#include <unordered_map>
#include <thread>

#include "ccglobal/spycc.h"

//calc time
//#include <ctime>

namespace mmesh
{
	struct point3dWithIndex
	{
		trimesh::point P;
		int Index;
		point3dWithIndex(trimesh::point p, int index)
		{
			P = p;
			Index = index;
		}

		int CompareTo(point3dWithIndex other)
		{
			if (P.x != other.P.x)
				return P.x > other.P.x ? 1 : -1;//P.X.CompareTo(other.P.X);
			else
			{
				if (P.y != other.P.y)
					return  P.y > other.P.y ? 1 : -1;//P.Y.CompareTo(other.P.Y);
				else
				{
					if (P.z != other.P.z)
						return  P.z > other.P.z ? 1 : -1;//P.Z.CompareTo(other.P.Z);
					else
						return 0;
				}
			}
		}
	};

	static void swap(std::vector<point3dWithIndex>& A, int index1, int index2)
	{
		point3dWithIndex temp = A[index1];
		A[index1] = A[index2];
		A[index2] = temp;
	}

	int partition(std::vector<point3dWithIndex>& A, int st, int ed, int partionPos)//SELECT VALUE ON PAPOS AND MAKE THE ARRAY(ST TO ED) INTO TWO PARTS: BEFORE RET SMALLER THAN VALUE AFTER BIGGER
	{
		if (partionPos != st)
			swap(A, partionPos, st);
		point3dWithIndex value = A[st];
		int boundary = st;//BOUNDARY REFER TO THE LAST SMALLER INDEX

		for (int i = st + 1; i <= ed; i++)
		{
			if (A[i].CompareTo(value) < 0)
			{
				if (boundary + 1 != i)
					swap(A, i, boundary + 1);
				boundary++;
			}
		}
		if (st != boundary)
			swap(A, boundary, st);
		return boundary;
	}

	void quickSort(std::vector<point3dWithIndex>& A, int st, int ed)
	{
		if (st < ed)
		{
			int pa = partition(A, st, ed, (st + ed) / 2);
			quickSort(A, st, pa - 1);
			quickSort(A, pa + 1, ed);
		}
	}

	void sortPoint(std::vector<point3dWithIndex>& A)
	{
		quickSort(A, 0, A.size() - 1);
	}

	void weldingMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
	{
		////calc time
		//std::clock_t start = clock();

		if (tracer)
		{
			tracer->progress(0.2f);
		}

		//size_t vertexNum = mesh->vertices.size();
		std::vector<trimesh::point>& vertices = mesh->vertices;

		std::vector<point3dWithIndex> pVertices;
		pVertices.reserve(vertices.size());
		for (size_t i = 0; i < vertices.size(); i++)
		{
			pVertices.push_back(point3dWithIndex(vertices[i],i));
		}
		if (tracer)
		{
			tracer->progress(0.4f);
		}

		sortPoint(pVertices);

		std::vector<int> tempArray(pVertices.size(),-1);
		for (int i = 0; i < pVertices.size(); i++)
		{
			tempArray[pVertices[i].Index] = i;
		}

		if (tracer)
		{
			tracer->progress(0.5f);
		}

		int lastIndex = 0;
		for (int i = 0; i < pVertices.size(); i++)
		{
			if (pVertices[i].CompareTo(pVertices[lastIndex]) == 0)
			{
				tempArray[pVertices[i].Index] = lastIndex;
				continue;
			}
			else
			{
				pVertices[lastIndex + 1] = pVertices[i];
				tempArray[pVertices[i].Index] = lastIndex + 1;
				lastIndex++;
			}
		}

		if (tracer)
		{
			tracer->progress(0.6f);
		}

		trimesh::TriMesh* omesh = new trimesh::TriMesh();
		omesh->vertices.reserve(lastIndex+1);
		for (int i = 0; i <= lastIndex; i++)
		{
			omesh->vertices.push_back(pVertices[i].P);
		}

		if (tracer)
		{
			tracer->progress(0.8f);
		}

		omesh->faces.swap(mesh->faces);
		for (int i = 0; i < omesh->faces.size(); i++)
		{
			trimesh::TriMesh::Face t = omesh->faces[i];
			t.x = tempArray[omesh->faces[i].x];
			t.y = tempArray[omesh->faces[i].y];
			t.z = tempArray[omesh->faces[i].z];
			omesh->faces[i] = t;
		}

		if (tracer)
		{
			tracer->progress(1.0f);
		}

		mesh->vertices.swap(omesh->vertices);
		mesh->faces.swap(omesh->faces);
		mesh->need_bbox();
		delete omesh;

		////calc time
		//clock_t end = clock();
		//double endtime = (double)(end - start);
		//std::printf("total time: %f\n", endtime);
	}

    void removeNorVector2(trimesh::TriMesh* mesh)
    {
        if (!mesh)
        {
            return;
        }

        int nf = mesh->faces.size();
        int nv = mesh->vertices.size();

        if (nv <= 0 )
        {
            return;
        }

        std::vector<bool> isIsolated(nv, false);
        trimesh::point p = mesh->vertices[0];
        bool bchanged = false;
#if defined(_OPENMP)
#pragma omp parallel for
#endif
        for (int i = 0; i < nf; i++) {
            const int& f1 = mesh->faces[i].x;
            const int& f2 = mesh->faces[i].y;
            const int& f3 = mesh->faces[i].z;
            if (f1 != f2 && f1 != f3 && f2 != f3)
            {
                if (!isIsolated[mesh->faces[i].x])
                    isIsolated[mesh->faces[i].x] = true;
                if (!isIsolated[mesh->faces[i].y])
                    isIsolated[mesh->faces[i].y] = true;
                if (!isIsolated[mesh->faces[i].z])
                    isIsolated[mesh->faces[i].z] = true;

                if (!bchanged)
                {
                    p = mesh->vertices[mesh->faces[i].x];
                    bchanged = true;
                }
            }
        }
         
        for (size_t i = 0; i < nv; i++)
        {
            if (!isIsolated[i])
            {
                mesh->vertices[i] = p;
            }
        }
    }
	bool compareTriMesh(const trimesh::TriMesh& mesh1, const trimesh::TriMesh& mesh2)
    {
        const auto& faces1 = mesh1.faces;
        const auto& faces2 = mesh2.faces;
        if (faces1.size() != faces2.size()) return false;
        const int& facenums = faces1.size();
        for (int i = 0; i < facenums; ++i) {
            if (faces1[i] != faces2[i]) return false;
        }
        const auto& points1 = mesh1.vertices;
        const auto& points2 = mesh2.vertices;
        if (points1.size() != points2.size()) return false;
        const int& pointnums = points1.size();
        for (int i = 0; i < pointnums; ++i) {
            if (points1[i] != points2[i]) return false;
        }
        return true;
    }
}