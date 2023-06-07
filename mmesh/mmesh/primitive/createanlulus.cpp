#include "createanlulus.h"
#include "createsphere.h"

namespace mmesh
{
	trimesh::TriMesh* createtorusMesh(float middler, float tuber, int sides,int slices)
	{

        Mesh_sphere mesh;

        // generate vertices
        for (size_t i = 0; i < sides; i++) {
            for (size_t j = 0; j < sides; j++) {
                float u = (float)j / sides * M_PI * 2.0;
                float v = (float)i / slices * M_PI * 2.0;
                float x = (middler + tuber * std::cos(v)) * std::cos(u);
                float y = (middler + tuber * std::cos(v)) * std::sin(u);
                float z = tuber * std::sin(v);
                mesh.vertices.push_back(trimesh::point (x, y, z));
            }
        }

        // add quad faces
        for (size_t i = 0; i < slices; i++) {
            auto i_next = (i + 1) % slices;
            for (size_t j = 0; j < sides; j++) {
                auto j_next = (j + 1) % sides;
                auto i0 = i * sides + j;
                auto i1 = i * sides + j_next;
                auto i2 = i_next * sides + j_next;
                auto i3 = i_next * sides + j;
                mesh.addQuad((i0), (i1), (i2), (i3));
            }
        }

   
        return mesh.convert();

	}

}