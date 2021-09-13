#ifndef MMESH_UNIFORMPOINTS_1631469381297_H
#define MMESH_UNIFORMPOINTS_1631469381297_H
#include "trimesh2/Vec.h"
#include <unordered_map>
#include <map>

namespace mmesh
{
	class UniformPoints
	{
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

		typedef std::map<trimesh::vec3, int, point_cmp> unique_point;
		typedef unique_point::iterator point_iterator;

	public:
		UniformPoints();
		~UniformPoints();

		int add(const trimesh::vec3& point);
		int uniformSize();
		void toVector(std::vector<trimesh::vec3>& points);
		void clear();
	protected:
		unique_point upoints;
	};
}

#endif // MMESH_UNIFORMPOINTS_1631469381297_H