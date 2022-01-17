#include "meshrepair.h"
#include "ccglobal/tracer.h"

#include "cmesh/mesh/meshwrapper.h"

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace mmesh
{
	TriMeshRepair::TriMeshRepair()
	{
		m_meshWrapper = std::make_shared<cmesh::MeshWrapper>();
	}

	TriMeshRepair::~TriMeshRepair()
	{
		if (m_mesh)
		{
			delete m_mesh;
			m_mesh = nullptr;
		}
	}

	void TriMeshRepair::initMesh(trimesh::TriMesh* mesh)
	{
		if (!mesh)
			return;
		if (mesh)
		{
			m_mesh = mesh;
		}
		m_meshWrapper->initMesh(mesh);
	}

	trimesh::TriMesh* TriMeshRepair::repair(ccglobal::Tracer* tracer)
	{
		if (m_mesh == nullptr)
		{
			if (tracer)
				tracer->failed("model mesh is empty.");
			return nullptr;
		}
		//m_mesh = mesh;

		removeNorFaces();
		if (tracer)
		{
			tracer->progress(0.2f);
		}
		need_face_faces();
		m_mesh->need_neighbors();
		remove_unconnected_facets();
		if (tracer)
		{
			tracer->progress(0.4f);
		}
		need_face_faces();
		need_across_edge2();
		need_normalsFaces();

		fix_normal_directions();
		if (tracer)
		{
			tracer->progress(0.6f);
		}

		fix_holes(tracer);

		normalsFaces.clear();
		m_mesh->need_normals();
		need_normalsFaces();

		fix_volume();
		if (tracer)
		{
			tracer->progress(0.8f);
		}
		trimesh::TriMesh* result = (trimesh::TriMesh*)new trimesh::TriMesh();
		result->faces = m_mesh->faces;
		result->vertices = m_mesh->vertices;

		if (tracer)
		{
			tracer->progress(0.9f);
		}
		return result;
	}

	void TriMeshRepair::clears()
	{

	}

	void TriMeshRepair::removeNorFaces()
	{
		int nf = m_mesh->faces.size();
		std::vector<char> del_facet(nf, 0);

#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < nf; i++) {
			trimesh::TriMesh::Face& facet = m_mesh->faces[i];
			if (m_mesh->vertices[facet[0]] == m_mesh->vertices[facet[1]] || m_mesh->vertices[facet[1]] == m_mesh->vertices[facet[2]] || m_mesh->vertices[facet[0]] == m_mesh->vertices[facet[2]])
			{
				del_facet[i] = 1;
			}
		}

		std::vector<trimesh::TriMesh::Face> validFaces;
		//#if defined(_OPENMP)
		//#pragma omp parallel for
		//#endif
		for (int i = 0; i < m_mesh->faces.size(); ++i)
		{
			if (del_facet[i] == 0)
				validFaces.push_back(m_mesh->faces.at(i));
		}
		m_mesh->faces.swap(validFaces);
	}

	float TriMeshRepair::get_volume()
	{
		// Choose a point, any point as the reference.
		trimesh::point p0 = m_mesh->vertices[m_mesh->faces[0][0]];
		float volume = 0.f;
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < m_mesh->faces.size(); ++i) {
			// Do dot product to get distance from point to plane.
			float height = normalsFaces[i].dot(m_mesh->vertices[m_mesh->faces[i][0]] - p0);
			float area = get_area(i);
			volume += (area * height) / 3.0f;
		}
		return volume;
	}

	float TriMeshRepair::get_area(int facet)
	{
		/* cast to double before calculating cross product because large coordinates
		 can result in overflowing product
		(bad area is responsible for bad volume and bad facets reversal) */
		double cross[3][3];
		//for (int i = 0; i < 3; i++) {
		//	cross[i][0] = (((double)facet->vertex[i](1) * (double)facet->vertex[(i + 1) % 3](2)) -
		//		((double)facet->vertex[i](2) * (double)facet->vertex[(i + 1) % 3](1)));
		//	cross[i][1] = (((double)facet->vertex[i](2) * (double)facet->vertex[(i + 1) % 3](0)) -
		//		((double)facet->vertex[i](0) * (double)facet->vertex[(i + 1) % 3](2)));
		//	cross[i][2] = (((double)facet->vertex[i](0) * (double)facet->vertex[(i + 1) % 3](1)) -
		//		((double)facet->vertex[i](1) * (double)facet->vertex[(i + 1) % 3](0)));
		//}

		float a = m_mesh->vertices[0][1];
		for (int i = 0; i < 3; i++) {
			cross[i][0] = (((double)m_mesh->vertices[m_mesh->faces[facet][i]][1] * (double)m_mesh->vertices[m_mesh->faces[facet][(i + 1) % 3]][2]) -
				((double)m_mesh->vertices[m_mesh->faces[facet][i]][2] * (double)m_mesh->vertices[m_mesh->faces[facet][(i + 1) % 3]][1]));
			cross[i][1] = (((double)m_mesh->vertices[m_mesh->faces[facet][i]][2] * (double)m_mesh->vertices[m_mesh->faces[facet][(i + 1) % 3]][0]) -
				((double)m_mesh->vertices[m_mesh->faces[facet][i]][0] * (double)m_mesh->vertices[m_mesh->faces[facet][(i + 1) % 3]][2]));
			cross[i][2] = (((double)m_mesh->vertices[m_mesh->faces[facet][i]][0] * (double)m_mesh->vertices[m_mesh->faces[facet][(i + 1) % 3]][1]) -
				((double)m_mesh->vertices[m_mesh->faces[facet][i]][1] * (double)m_mesh->vertices[m_mesh->faces[facet][(i + 1) % 3]][0]));
		}

		trimesh::vec sum;
		sum[0] = cross[0][0] + cross[1][0] + cross[2][0];
		sum[1] = cross[0][1] + cross[1][1] + cross[2][1];
		sum[2] = cross[0][2] + cross[1][2] + cross[2][2];

		// This should already be done.  But just in case, let's do it again.
		//FIXME this is questionable. the "sum" normal should be accurate, while the normal "n" may be calculated with a low accuracy.
		trimesh::vec n;
		//stl_calculate_normal(n, facet);
		//stl_normalize_vector(n);
		n = normalsFaces[facet];
		normalize(n);
		return 0.5f * n.dot(sum);
	}

	void TriMeshRepair::need_face_faces()
	{
		if (!face_faces.empty())
			face_faces.clear();

		m_mesh->adjacentfaces.clear();
		m_mesh->need_adjacentfaces();

		int nf = m_mesh->faces.size();
		face_faces.resize(nf);

		//face_faces_nums.resize(nf, trimesh::TriMesh::Face(0, 0, 0));
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < nf; i++) {
			for (int j = 0; j < 3; j++) {
				int v1 = m_mesh->faces[i][j];
				int v2 = m_mesh->faces[i][NEXT_MOD3(j)];
				const std::vector<int>& a1 = m_mesh->adjacentfaces[v1];
				const std::vector<int>& a2 = m_mesh->adjacentfaces[v2];

				//vector<int>vTarget;
				//vTarget.resize(min(a1.size(), a2.size()));
				//set_intersection(a1.begin(), a1.end(), a2.begin(), a2.end(), vTarget.begin());

				std::vector<int>vTarget;
				for (const int& n1 : a1)
				{
					for (const int& n2 : a2)
					{
						if (n1 == n2)
						{
							vTarget.push_back(n1);

						}
					}
				}

				for (size_t k1 = 0; k1 < vTarget.size(); k1++) {
					if (vTarget[k1] == i)
						continue;
					//TODO for Non-manifold

					//face_faces[i][j].push_back(vTarget[k1]);
					switch (j)
					{
					case 0:
						face_faces[i].f0.push_back(vTarget[k1]);
						break;
					case 1:
						face_faces[i].f1.push_back(vTarget[k1]);
						break;
					case 2:
						face_faces[i].f2.push_back(vTarget[k1]);
						break;
					default:
						break;
					}

					//if (face_faces[i][j] != -1)
					//{
					//	face_faces[i][j] = vTarget[k1];
					//}
					//else
					//	face_faces[i][j] = vTarget[k1];

					//face_faces_nums[i][j] ++;
					//break;
				}
			}
		}
	}

	void TriMeshRepair::need_across_edge2()
	{
		if (!m_mesh->across_edge.empty())
			m_mesh->across_edge.clear();

		m_mesh->need_adjacentfaces();
		if (m_mesh->adjacentfaces.empty())
			return;

		//dprintf("Finding across-edge maps... ");

		int nf = m_mesh->faces.size();
		m_mesh->across_edge.resize(nf, trimesh::TriMesh::Face(-1, -1, -1));

#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < nf; i++) {
			for (int j = 0; j < 3; j++) {
				int v1 = m_mesh->faces[i][j];
				int v2 = m_mesh->faces[i][NEXT_MOD3(j)];
				const std::vector<int>& a1 = m_mesh->adjacentfaces[v1];
				for (size_t k1 = 0; k1 < a1.size(); k1++) {
					int other = a1[k1];
					if (other == i)
						continue;
					int v2_in_other = m_mesh->faces[other].indexof(v2);
					if (v2_in_other < 0)
						continue;
					if (m_mesh->faces[other][NEXT_MOD3(v2_in_other)] != v1)
						continue;
					m_mesh->across_edge[i][j] = other;
					break;
				}
			}
		}

		//dprintf("Done.\n");
	}

	void TriMeshRepair::fix_normal_directions()
	{
		int nv = m_mesh->vertices.size(), nf = m_mesh->faces.size();

		std::vector<char> norm_sw(m_mesh->faces.size(), 0);
		std::vector<int> indexFace;
		indexFace.clear();

		int facet_num = 0;
		int checked = 1;
		norm_sw[facet_num] = 1;
		int num_neighbors = 3 - ((m_mesh->across_edge[0][0] == -1) + (m_mesh->across_edge[0][1] == -1) + (m_mesh->across_edge[0][2] == -1));
		if (num_neighbors == 0)
		{
			//reverse_facet(0);
			norm_sw[0] = -1;
		}

		for (;;) {
			for (int j = 0; j < 3; ++j) {

				std::vector<int>& face_face = (j == 0 ? face_faces[facet_num].f0 : j == 1 ? face_faces[facet_num].f1 : face_faces[facet_num].f2);
				switch (norm_sw[facet_num]) {
				case 1:
					for (int k = 0; k < face_face.size(); k++)
					{
						if (face_face[k] > 0 && norm_sw[face_face[k]] == 0)
						{
							if (m_mesh->across_edge[facet_num][j] == -1)
							{
								norm_sw[face_face[k]] = -1;
							}
							else
							{
								norm_sw[face_face[k]] = 1;
							}
							indexFace.push_back(face_face[k]);
							checked++;
						}
					}
					break;
				case 0:
					break;
				case -1:
					for (int k = 0; k < face_face.size(); k++)
					{
						if (face_face[k] > 0 && norm_sw[face_face[k]] == 0)
						{
							if (m_mesh->across_edge[facet_num][j] == -1)
							{
								norm_sw[face_face[k]] = 1;
							}
							else
							{
								norm_sw[face_face[k]] = -1;
							}
							indexFace.push_back(face_face[k]);
							checked++;
						}
					}
					break;
				}
			}

			if (indexFace.size() > 0)
			{
				facet_num = indexFace.back();
				indexFace.pop_back();
				//checked++;
			}
			else
			{
				for (int i = 0; i < nf; i++) {
					if (norm_sw[i] == 0)
					{
						//TODO :first is wrong
						facet_num = i;
						norm_sw[facet_num] = 1;
						int num_neighbors = 3 - ((m_mesh->across_edge[facet_num][0] == -1) + (m_mesh->across_edge[facet_num][1] == -1) + (m_mesh->across_edge[facet_num][2] == -1));
						if (num_neighbors == 0)
						{
							//reverse_facet(0);
							norm_sw[facet_num] = -1;
						}

						checked++;
						break;
					}
				}

				//reverse_facet(facet_num);
			}
			if (checked >= m_mesh->faces.size())
				break;
		}

#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < nf; i++)
		{
			if (norm_sw[i] == -1)
			{
				reverse_facet(i, false);
			}
		}

		m_mesh->across_edge.clear();
		need_across_edge2();
		need_face_faces();
		m_mesh->need_normals();
	}

	void TriMeshRepair::remove_unconnected_facets()
	{
		int nf = m_mesh->faces.size();
		std::vector<char> del_facet(nf, 0);
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < nf; i++) {
			for (int j = 0; j < 3; j++) {
				if (m_mesh->neighbors[m_mesh->faces[i][j]].size() <= 2)
				{
					int k = (j + 1) % 3;
					std::vector<int>& face_face = (k == 0 ? face_faces[i].f0 : k == 1 ? face_faces[i].f1 : face_faces[i].f2);
					if (face_face.size() > 1)
					{
						del_facet[i] = 1;
					}
				}
				//neighbors
			}
		}

		std::vector<trimesh::TriMesh::Face> validFaces;
		for (int i = 0; i < m_mesh->faces.size(); ++i)
		{
			if (del_facet[i] == 0)
				validFaces.push_back(m_mesh->faces.at(i));
		}
		m_mesh->faces.swap(validFaces);
	}

	void TriMeshRepair::fix_volume()
	{
		float volume = get_volume();
		if (volume < 0.0) {
			for (uint32_t i = 0; i < m_mesh->faces.size(); ++i) {
				reverse_facet(i, false);
			}
		}
		m_mesh->need_normals();
	}

	void TriMeshRepair::reverse_facet(int facet_num, bool update)
	{
		int tmp_vertex = m_mesh->faces[facet_num][0];
		m_mesh->faces[facet_num][0] = m_mesh->faces[facet_num][1];
		m_mesh->faces[facet_num][1] = tmp_vertex;

		if (update)
		{
			m_mesh->across_edge.clear();
			need_across_edge2();
			need_face_faces();
		}

	}

	void TriMeshRepair::need_normalsFaces(bool simple_area_weighted)
	{
		normalsFaces.resize(m_mesh->faces.size());
		for (int i = 0; i < m_mesh->faces[i].size(); i++)
		{
			normalsFaces[i] = m_mesh->trinorm(i);
		}
	}

	void getErrorEdges(trimesh::TriMesh* mesh, int& errorEdges, int& errorNormals)
	{
		if (mesh == nullptr)
			return;

		errorEdges = 0;
		errorNormals = 0;

		mesh->need_across_edge();
		mesh->need_adjacentfaces();
		int nf = mesh->faces.size();

//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < nf; i++) {
			for (int j = 0; j < 3; j++) {
				int v1 = mesh->faces[i][j];
				int v2 = mesh->faces[i][NEXT_MOD3(j)];
				const std::vector<int>& a1 = mesh->adjacentfaces[v1];
				const std::vector<int>& a2 = mesh->adjacentfaces[v2];

				std::vector<int>vTarget;
				for (const int& n1 : a1)
					for (const int& n2 : a2)
						if (n1 == n2)
							vTarget.push_back(n1);

				if (vTarget.size() == 1)
				{
					errorEdges++;
				}
				else if (mesh->across_edge[i][j] == -1)
				{
					errorNormals++;
				}
			}
		}
		errorNormals > 0 ? (errorNormals + 6) / 6 : 0;
	}

	void TriMeshRepair::fix_holes(ccglobal::Tracer* tracer)
	{
		trimesh::TriMesh* result = m_meshWrapper->fillHolesWrapper(tracer);

		m_mesh->need_normals();
		need_normalsFaces();
	}
}