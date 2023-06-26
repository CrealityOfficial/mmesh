#ifndef MMESH_SPLIT_1612341980784_H
#define MMESH_SPLIT_1612341980784_H
#include "trimesh2/TriMesh.h"
#include <map>
#include "list"
#include "trimesh2/XForm.h"
#include "trimesh2/Vec3Utils.h"
#include "mmesh/trimesh/polygonstack.h"
#include <float.h>

struct segment
{
	int start;
	int end;
};

class point_cmp
{
public:
	point_cmp(float e = FLT_MIN) :eps(e) {}

	bool operator()(const trimesh::vec3& v0, const trimesh::vec3& v1) const
	{
		if (fabs(v0.x - v1.x) <= eps)
		{
			if (fabs(v0.y - v1.y) <= eps)
			{
				return (v0.z < v1.z - eps);
			}
			else return (v0.y < v1.y - eps);
		}
		else return (v0.x < v1.x - eps);
	}
private:
	float eps;
};

struct IndexPolygon
{
	std::list<int> polygon;
	int start;
	int end;

	bool closed()
	{
		return (polygon.size() >= 2) && (polygon.front() == polygon.back());
	}
};

typedef std::map<trimesh::vec3, int, point_cmp> unique_point;
typedef unique_point::iterator point_iterator;

namespace mmesh
{
	bool split(trimesh::TriMesh* inputMesh, float z, const trimesh::vec3& normal,
		trimesh::TriMesh** mesh1, trimesh::TriMesh** mesh2, float x = 0.0f, float y=0.0f);

	//切割区间
	bool splitRangeZ(trimesh::TriMesh* inputMesh, float Upz, float Dowmz,trimesh::TriMesh** mesh);

	//切成小方块
	bool splitRangeXYZ(trimesh::TriMesh* inputMesh
		, std::vector<trimesh::vec3>& horizon
		, std::vector<trimesh::vec3>& vertical
		, float interval
		, std::vector < trimesh::TriMesh*>& outMesh);

	void FaceGenerateMesh(trimesh::TriMesh* newMesh, trimesh::TriMesh* inputMesh, std::vector<trimesh::TriMesh::Face>& inputface);

	void fcollid(std::vector<trimesh::vec3>& lines, trimesh::TriMesh* inputMesh, std::vector<float>& distances,
		trimesh::TriMesh* meshUP, trimesh::TriMesh* meshDown,
		int Point0Index, int Point1Index, int Point2Index);

	void addFaceVertices(trimesh::TriMesh* mesh, const trimesh::vec3& v1, const trimesh::vec3& v2, const trimesh::vec3& v3);

	void lines2polygon(std::vector<trimesh::vec3>& lines, std::vector<std::vector<int>>& polygons, std::vector<trimesh::vec3>& uniPoints);

	int generateIndex(unique_point& points, const trimesh::vec3& aPoint);

	int check(std::vector<bool>& used);
}

#endif // MMESH_SPLIT_1612341980784_H
