#include"HorizontalArrangement.h"
#include "mmesh/util/mnode.h"
#include "qhullWrapper/hull/meshconvex.h"
#include <queue>
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
		//�������polygon ��������ֻ�����������50��polygon:
		std::sort(meshs.rbegin(), meshs.rend(), [](trimesh::TriMesh * a, trimesh::TriMesh * b)
			{
				return getTotalArea(a->vertices) < getTotalArea(b->vertices);
			});
		meshs.resize(std::min((int)meshs.size(), 50));//50
		
	}

    void getConvexHullMesh2(trimesh::TriMesh* inputMesh, std::vector<trimesh::TriMesh>& partMeshes)
    {
        mmesh::dumplicateMesh(inputMesh);
        const auto& faces = inputMesh->faces;
        const auto& vertexs = inputMesh->vertices;
        const int facenums = faces.size();
        std::vector<trimesh::point> normals;
        normals.reserve(facenums);
        std::vector<float> areas;
        areas.reserve(facenums);
        for (const auto& f : faces) {
            const auto& a = vertexs[f[0]];
            const auto& b = vertexs[f[1]];
            const auto& c = vertexs[f[2]];
            const auto& n = 0.5 * (b - a)TRICROSS(c - a);
            const auto & area = len(n);
            areas.emplace_back(area);
            normals.emplace_back(n / area);
        }
        inputMesh->need_across_edge();
        std::vector<trimesh::TriMesh::Face> neighbors = inputMesh->across_edge;
        std::vector<bool> masks(facenums, true);
        std::vector<std::vector<int>> selectFaces;
        selectFaces.reserve(facenums);
        std::vector<trimesh::TriMesh> faceMeshes;
        for (int f = 0; f < facenums; ++f) {
            if (masks[f]) {
                const auto& nf = normals[f];
                std::vector<int>currentFaces;
                std::queue<int>currentQueue;
                currentQueue.emplace(f);
                currentFaces.emplace_back(f);
                masks[f] = false;
                while (!currentQueue.empty()) {
                    auto& fr = currentQueue.front();
                    currentQueue.pop();
                    const auto& neighbor = neighbors[fr];
                    for (const auto& fa : neighbor) {
                        if (fa < 0) continue;
                        if (masks[fa]) {
                            const auto& na = normals[fa];
                            const auto& nr = normals[fr];
                            if ((nr DOT na) > 0.95 && (nf DOT na) > 0.95) {
                                currentQueue.emplace(fa);
                                currentFaces.emplace_back(fa);
                                masks[fa] = false;
                            }
                        }
                    }
                }
                trimesh::TriMesh regionMesh;
                const size_t num = currentFaces.size();
                regionMesh.faces.reserve(num);
                regionMesh.vertices.reserve(3 * num);
                for (size_t i = 0; i < num; ++i) {
                    const auto& f = currentFaces[i];
                    for (int j = 0; j < 3; ++j) {
                        regionMesh.vertices.emplace_back(std::move(vertexs[faces[f][j]]));
                    }
                    regionMesh.faces.emplace_back(std::move(trimesh::TriMesh::Face(3 * i, 3 * i + 1, 3 * i + 2)));
                }
                if (num > 1) mmesh::dumplicateMesh(&regionMesh);
                faceMeshes.emplace_back(std::move(regionMesh));
            }
        }
        for (auto& m : faceMeshes) {
            m.need_pointareas();
            m.need_normals();
        }
        std::sort(faceMeshes.begin(), faceMeshes.end(), [](const trimesh::TriMesh & a, const trimesh::TriMesh & b) {
            double sa = 0, sb = 0;
            for (const auto& area : a.pointareas) {
                sa += area;
            }
            for (const auto& area : b.pointareas) {
                sb += area;
            }
            return sa > sb;
        });
        const int regions = faceMeshes.size();
        for (int i = 0; i < faceMeshes.size(); ++i) {
            faceMeshes[i].write("test/hullfaces" + std::to_string(i) + ".stl");
        }
        //
        partMeshes.reserve(regions);
        for (unsigned int polygon_id = 0; polygon_id < regions; ++polygon_id) {
            trimesh::vec3 normal;
            trimesh::TriMesh* mesh = &faceMeshes[polygon_id];
            trimesh::fxform xf = mmesh::adjustmentMesh(mesh, normal);
            mesh = qhullWrapper::convex_hull_2d(mesh);
            if (mmesh::checkMesh(mesh, normal)) {
                faceMeshes.erase(faceMeshes.begin() + (polygon_id--));
                continue;
            }
            mmesh::indentationMesh(mesh);
            //ͨ�������ת������ά����
            for (trimesh::point& apoint : mesh->vertices) {
                apoint = trimesh::inv(xf) * apoint;
            }
            partMeshes.emplace_back(*mesh);
        }

        //���ǻ�
        for (trimesh::TriMesh& amesh : partMeshes) {
            std::vector<trimesh::point> apoints = amesh.vertices;
            amesh.vertices.clear();
            if (apoints.size() < 3) {
                continue;
            }

            for (size_t n = 2; n < apoints.size(); n++) {
                int index = amesh.vertices.size();
                amesh.vertices.push_back(apoints[0]);
                amesh.vertices.push_back(apoints[n - 1]);
                amesh.vertices.push_back(apoints[n]);
                amesh.faces.push_back(trimesh::TriMesh::Face(index, index + 1, index + 2));
            }
        }
        for (int i = 0; i < partMeshes.size(); ++i) {
            partMeshes[i].write("test/hullfaces" + std::to_string(i) + ".stl");
        }
    }

	trimesh::fxform adjustmentMesh(trimesh::TriMesh* inmesh, trimesh::vec3& normal)
	{
		//trimesh::TriMesh*& currentMesh = meshs[polygon_id];
		trimesh::TriMesh*& currentMesh = inmesh;
		normal = currentMesh->normals[0];

		//������΢̧�𣬷�ֹ��ģ�͵����ص�
		for (trimesh::point& apoint : currentMesh->vertices)
		{
			apoint += normal * 0.1;
		}

		//��z��y��ת��ʹƽ���ƽ
		const trimesh::vec3 XYnormal(0.0f, 0.0f, 1.0f);
		trimesh::quaternion q = q.rotationTo(normal, XYnormal);
		trimesh::fxform xf = mmesh::fromQuaterian(q);
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

		//������ε��ڽǣ���������С��������ֵ�Ķ����
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

		//����������Ǽ򵥺�͹�ģ����ǽ�ʹ��Ǳ�Բ����������������
		//���㷨ȡһ�����㣬������Աߵ��м�ֵ��Ȼ���ƶ�����
		//�ӽ�ƽ��ˮƽ(�ɡ�aggressivity������)���ظ�k�Ρ�
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
		//���ǻ�
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