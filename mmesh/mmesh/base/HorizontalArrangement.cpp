#include"HorizontalArrangement.h"

namespace mmesh
{
	void getConvexHullMesh(trimesh::TriMesh* mesh, std::vector<trimesh::TriMesh*>& meshs)
	{
		mmesh::dumplicateMesh(mesh);
		MMeshT* meshT = new MMeshT();
		meshT->trimesh2meshtest(mesh);

		const int num_of_facets = meshT->faces.size();
		std::vector<int>  facet_queue(num_of_facets, 0);
		//std::vector<bool> facet_visited(num_of_facets, false);
		int               facet_queue_cnt = 0;
		trimesh::point normal_ptr;
		while (1)
		{
			// Find next unvisited triangle:
			int facet_idx = 0;
			for (; facet_idx < num_of_facets; ++facet_idx)
			{
				//if (!facet_visited[facet_idx])
				if (!meshT->faces[facet_idx].IsV())
				{
					facet_queue[facet_queue_cnt++] = facet_idx;
					meshT->faces[facet_idx].SetV();

					normal_ptr = meshT->faces[facet_idx].normal;
					trimesh::TriMesh* bottomMesh = new trimesh::TriMesh();
					meshs.push_back(bottomMesh);
					break;
				}
			}

			if (facet_idx == num_of_facets)
				break; // Everything was visited already

			while (facet_queue_cnt > 0)
			{
				int facet_idx = facet_queue[--facet_queue_cnt];

				trimesh::point this_normal = meshT->faces[facet_idx].normal;
				if (std::abs(this_normal.at(0) - normal_ptr.at(0)) < 0.001
					&& std::abs(this_normal.at(1) - normal_ptr.at(1)) < 0.001
					&& std::abs(this_normal.at(2) - normal_ptr.at(2)) < 0.001)
					//if(abs((this_normal DOT normal_ptr)-1)<0.001)
				{
					for (int j = 0; j < 3; ++j)
					{
						int vertex_idx = meshT->faces[facet_idx].vertex_index[j];
						meshs.back()->vertices.push_back(meshT->vertices[vertex_idx].p);
					}
					//facet_visited[facet_idx] = true;
					meshT->faces[facet_idx].SetV();

					for (int j = 0; j < 3; ++j)
					{
						int neighbor_idx = meshT->faces[facet_idx].connected_face_index[j];
						//if (!facet_visited[neighbor_idx])
						if (!meshT->faces[neighbor_idx].IsV())
							facet_queue[facet_queue_cnt++] = neighbor_idx;
					}
				}
			}
			meshs.back()->normals.push_back(normal_ptr);
		}
		//按面积对polygon 进行排序，只保留面积最大的50个polygon:
		std::sort(meshs.rbegin(), meshs.rend(), [](trimesh::TriMesh * a, trimesh::TriMesh * b)
			{
				return getTotalArea(a->vertices) < getTotalArea(b->vertices);
			});
		meshs.resize(std::min((int)meshs.size(), 50));//50
		
	}

	trimesh::fxform adjustmentMesh(trimesh::TriMesh* inmesh, trimesh::vec3& normal)
	{
		//trimesh::TriMesh*& currentMesh = meshs[polygon_id];
		trimesh::TriMesh*& currentMesh = inmesh;
		normal = currentMesh->normals[0];

		//将面轻微抬起，防止与模型的面重叠
		for (trimesh::point& apoint : currentMesh->vertices)
		{
			apoint += normal * 0.1;
		}

		//绕z和y旋转，使平面变平
		const trimesh::vec3 XYnormal(0.0f, 0.0f, 1.0f);
		trimesh::quaternion q = q.rotationTo(normal, XYnormal);
		trimesh::fxform xf = fromQuaterian(q);
		for (trimesh::point& apoint : currentMesh->vertices)
		{
			apoint = (xf * apoint);
		}
		return xf;
	}

	bool checkMesh(trimesh::TriMesh* inmesh, trimesh::vec3& normal)
	{
		trimesh::TriMesh*& currentMesh = inmesh;		
		currentMesh->normals.push_back(normal);
		std::vector<trimesh::point>& polygon = currentMesh->vertices;

		//检查多边形的内角，并丢弃角小于以下阈值的多边形
		static constexpr double PI = 3.141592653589793238;		
		const double angle_threshold = ::cos(10.0 * (double)PI / 180.0);
		for (unsigned int i = 0; i < polygon.size(); ++i)
		{
			const trimesh::point& prec = polygon[(i == 0) ? polygon.size() - 1 : i - 1];
			const trimesh::point & curr = polygon[i];
			const trimesh::point & next = polygon[(i == polygon.size() - 1) ? 0 : i + 1];
			if (normalized(prec - curr).dot(normalized(next - curr)) > angle_threshold)
			{
				return true;

			}
		}
		return false;
	}

	void indentationMesh(trimesh::TriMesh* inmesh)
	{
		trimesh::TriMesh*& currentMesh = inmesh;
		std::vector<trimesh::point>& polygon = currentMesh->vertices;

		trimesh::point centroid = std::accumulate(polygon.begin(), polygon.end(), trimesh::point(0.0, 0.0, 0.0));
		centroid /= (double)polygon.size();
		for (auto& vertex : polygon)
		{
			vertex = 0.9f * vertex + 0.1f * centroid;
		}

		//多边形现在是简单和凸的，我们将使尖角变圆，这样看起来更好
		//该算法取一个顶点，计算各自边的中间值，然后移动顶点
		//接近平均水平(由“aggressivity”控制)。重复k次。
		const unsigned int k = 10; // number of iterations
		const float aggressivity = 0.2f;  // agressivity
		const unsigned int N = polygon.size();
		std::vector<std::pair<unsigned int, unsigned int>> neighbours;
		if (k != 0)
		{
			std::vector<trimesh::point> points_out(2 * k * N); // vector long enough to store the future vertices
			for (unsigned int j = 0; j < N; ++j) {
				points_out[j * 2 * k] = polygon[j];
				neighbours.push_back(std::make_pair((int)(j * 2 * k - k) < 0 ? (N - 1) * 2 * k + k : j * 2 * k - k, j * 2 * k + k));
			}

			for (unsigned int i = 0; i < k; ++i) {
				// Calculate middle of each edge so that neighbours points to something useful:
				for (unsigned int j = 0; j < N; ++j)
					if (i == 0)
						points_out[j * 2 * k + k] = 0.5f * (points_out[j * 2 * k] + points_out[j == N - 1 ? 0 : (j + 1) * 2 * k]);
					else {
						float r = 0.2 + 0.3 / (k - 1) * i; // the neighbours are not always taken in the middle
						points_out[neighbours[j].first] = r * points_out[j * 2 * k] + (1 - r) * points_out[neighbours[j].first - 1];
						points_out[neighbours[j].second] = r * points_out[j * 2 * k] + (1 - r) * points_out[neighbours[j].second + 1];
					}
				// Now we have a triangle and valid neighbours, we can do an iteration:
				for (unsigned int j = 0; j < N; ++j)
					points_out[2 * k * j] = (1 - aggressivity) * points_out[2 * k * j] +
					aggressivity * 0.5f * (points_out[neighbours[j].first] + points_out[neighbours[j].second]);

				for (auto& n : neighbours) {
					++n.first;
					--n.second;
				}
			}
			polygon = points_out; // replace the coarse polygon with the smooth one that we just created
		}		
	}


	void triangularization(std::vector<trimesh::TriMesh*>& meshs)
	{
		//三角化
		for (trimesh::TriMesh* amesh : meshs)
		{
			std::vector<trimesh::point> apoints = amesh->vertices;
			amesh->vertices.clear();
			if (apoints.size() < 3)
			{
				continue;
			}

			for (int n = 2; n < apoints.size(); n++)
			{
				int index = amesh->vertices.size();
				amesh->vertices.push_back(apoints[0]);
				amesh->vertices.push_back(apoints[n - 1]);
				amesh->vertices.push_back(apoints[n]);
				amesh->faces.push_back(trimesh::TriMesh::Face(index, index + 1, index + 2));
			}
		}
	}
}