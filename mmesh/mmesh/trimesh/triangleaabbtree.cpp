#include "mmesh/trimesh/triangleaabbtree.h"
#include "mmesh/trimesh/algrithm3d.h"

namespace mmesh
{

	TriangleAABBTree::TriangleAABBTree()
		:m_mesh(nullptr)
	{
	}

	TriangleAABBTree::~TriangleAABBTree()
	{
	}

	void TriangleAABBTree::build(trimesh::TriMesh* mesh, trimesh::fxform& xf)
	{
		if (m_mesh == mesh)
			return;

		clear();
		m_mesh = mesh;
		m_xf = xf;
		build();
	}

	void TriangleAABBTree::build()
	{
		if (m_mesh && m_mesh->vertices.size() > 1 && m_mesh->faces.size() > 1)
		{
			size_t size = m_mesh->faces.size();
			clearNodes();

			m_nodes.resize(size - 1);
			std::vector<int> indices;
			std::vector<trimesh::box3> boxMap;
			indices.resize(size);
			boxMap.resize(size);
			for (size_t i = 0; i < size; ++i)
			{
				indices.at(i) = (int)i;
				trimesh::TriMesh::Face& f = m_mesh->faces.at(i);
				for (int j = 0; j < 3; ++j)
				{
					boxMap.at(i) += m_xf * m_mesh->vertices.at(f[j]);
				}
			}
			int* first = &indices.at(0);
			int* beyond = first + size;
			m_nodes.at(0).expand(first, beyond, size, boxMap);
		}
	}

	void TriangleAABBTree::clear()
	{
		clearNodes();
	}

	void TriangleAABBTree::clearNodes()
	{
		m_nodes.clear();
	}

	void TriangleAABBTree::zQuery(trimesh::vec3& p, std::vector<ZQueryResult>& collides)
	{
		std::vector<int> indices;
		if (m_nodes.size() > 0)
		{
			m_nodes.at(0).ztraversal(p, m_mesh->faces.size(), indices);
		}

		if (indices.size() > 0)
		{
			trimesh::fxform invXf = trimesh::inv(m_xf);
			trimesh::vec3 n(0.0f, 0.0f, 1.0f);
			n = trimesh::norm_xf(invXf) * n;
			trimesh::normalize(n);

			p.z = 0.0f;
			p = invXf * p;

			collides.reserve(indices.size());
			for (int index : indices)
			{
				trimesh::TriMesh::Face& f = m_mesh->faces.at(index);
				trimesh::vec3 v0 = m_mesh->vertices.at(f.x);
				trimesh::vec3 v1 = m_mesh->vertices.at(f.y);
				trimesh::vec3 v2 = m_mesh->vertices.at(f.z);

				float t, u, v;
				if (rayIntersectTriangle(p, n, v0, v1, v2, &t, &u, &v))
				{
					trimesh::vec3 c = (1.0f - u - v) * v0 + u * v1 + v * v2;

					ZQueryResult query;
					query.index = index;
					query.p = m_xf * c;
					collides.push_back(query);
				}
			}
		}
	}

	void TriangleAABBTree::lineQuery(trimesh::vec3& p, trimesh::vec3& n, std::vector<int>& indices)
	{
	}

}