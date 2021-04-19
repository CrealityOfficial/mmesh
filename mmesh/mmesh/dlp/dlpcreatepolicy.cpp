#include "dlpcreatepolicy.h"

using namespace trimesh;
namespace mmesh
{
	DLPISource generateSource(const vec3& position, const vec3& normal)
	{
		DLPISource source;
		source.position = position;
		source.normal = normal;
#if 0
		float smallTheta = 25.0f * M_PIf / 180.0f;
		if (trimesh::dot(normal, trimesh::vec3(0.0f, 0.0f, -1.0f)) < sinf(smallTheta))
		{
			trimesh::vec3 dir = normal;
			dir.z = 0.0f;
			float z = len(dir) * tanf(smallTheta);
			dir.z = -z;
			source.normal = trimesh::normalized(dir);
		}
#endif
		return source;
	}
}