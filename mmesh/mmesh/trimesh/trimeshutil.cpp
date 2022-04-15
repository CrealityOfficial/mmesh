#include "mmesh/trimesh/trimeshutil.h"
#include "trimesh2/TriMesh.h"
#include "trimesh2/TriMesh_algo.h"

#include <unordered_map>
#include <assert.h>

#include "mmesh/create/ballcreator.h"
#include "ccglobal/tracer.h"
#include "ccglobal/spycc.h"

#include <ctime>

namespace mmesh
{
	void mergeTriMesh(trimesh::TriMesh* outMesh, const std::vector<trimesh::TriMesh*>& inMeshes, bool fanzhuan)
	{
		assert(outMesh);
		size_t totalVertexSize = outMesh->vertices.size();
        size_t totalUVSize = outMesh->cornerareas.size();
		size_t totalTriangleSize = outMesh->faces.size();
        
		size_t addVertexSize = 0;
		size_t addTriangleSize = 0;
        size_t addUVSize = 0;
        
		size_t meshSize = inMeshes.size();
		for (size_t i = 0; i < meshSize; ++i)
		{
			if (inMeshes.at(i))
			{
				addVertexSize += inMeshes.at(i)->vertices.size();
				addTriangleSize += inMeshes.at(i)->faces.size();
                addUVSize += inMeshes.at(i)->cornerareas.size();
			}
		}
		totalVertexSize += addVertexSize;
		totalTriangleSize += addTriangleSize;
        totalUVSize += addUVSize;

		if (addVertexSize > 0 && addTriangleSize > 0)
		{
			outMesh->vertices.reserve(totalVertexSize);
            outMesh->cornerareas.reserve(totalUVSize);
            outMesh->faces.reserve(totalTriangleSize);

            size_t startFaceIndex = outMesh->faces.size();
            size_t startVertexIndex = outMesh->vertices.size();;
            size_t startUVIndex = outMesh->cornerareas.size();
			for (size_t i = 0; i < meshSize; ++i)
			{
				trimesh::TriMesh* mesh = inMeshes.at(i);
				if (mesh)
				{
					int vertexNum = (int)mesh->vertices.size();
					int faceNum = (int)mesh->faces.size();
                    int uvNum = (int)mesh->cornerareas.size();
					if (vertexNum > 0 && faceNum > 0)
					{
						outMesh->vertices.insert(outMesh->vertices.end(), mesh->vertices.begin(), mesh->vertices.end());
                        outMesh->cornerareas.insert(outMesh->cornerareas.end(), mesh->cornerareas.begin(), mesh->cornerareas.end());
						outMesh->faces.insert(outMesh->faces.end(), mesh->faces.begin(), mesh->faces.end());

                        size_t endFaceIndex = startFaceIndex + faceNum;
						if (startVertexIndex > 0)
						{
							for (size_t ii = startFaceIndex; ii < endFaceIndex; ++ii)
							{
								trimesh::TriMesh::Face& face = outMesh->faces.at(ii);
								for (int j = 0; j < 3; ++j)
									face[j] += startVertexIndex;

								if (fanzhuan)
								{
									int t = face[1];
									face[1] = face[2];
									face[2] = t;
								}
							}
						}

						startFaceIndex += faceNum;
						startVertexIndex += vertexNum;
                        startUVIndex += uvNum;
                        
					}
				}
			}
		}
	}

	void mergeTriMesh_omp(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes)
	{
		size_t totalVertexSize = outMesh->vertices.size();
		size_t totalTriangleSize = outMesh->faces.size();
		size_t addVertexSize = 0;
		size_t addTriangleSize = 0;
		size_t meshSize = inMeshes.size();
		for (size_t i = 0; i < meshSize; ++i)
		{
			if (inMeshes.at(i))
			{
				addVertexSize += inMeshes.at(i)->vertices.size();
				addTriangleSize += inMeshes.at(i)->faces.size();
			}
		}
		totalVertexSize += addVertexSize;
		totalTriangleSize += addTriangleSize;

		if (addVertexSize > 0 && addTriangleSize > 0)
		{
			outMesh->vertices.reserve(totalVertexSize);
			outMesh->faces.reserve(totalTriangleSize);

			int startFaceIndex = outMesh->faces.size();
			int startVertexIndex = outMesh->vertices.size();;
			for (size_t i = 0; i < meshSize; ++i)
			{
				trimesh::TriMesh* mesh = inMeshes.at(i);
				if (mesh)
				{
					int vertexNum = (int)mesh->vertices.size();
					int faceNum = (int)mesh->faces.size();
					if (vertexNum > 0 && faceNum > 0)
					{
						outMesh->vertices.insert(outMesh->vertices.end(), mesh->vertices.begin(), mesh->vertices.end());
						outMesh->faces.insert(outMesh->faces.end(), mesh->faces.begin(), mesh->faces.end());

						int endFaceIndex = startFaceIndex + faceNum;
						if (startVertexIndex > 0)
						{
							for (int ii = startFaceIndex; ii < endFaceIndex; ++ii)
							{
								trimesh::TriMesh::Face& face = outMesh->faces.at(ii);
								for (int j = 0; j < 3; ++j)
									face[j] += startVertexIndex;
							}
						}

						startFaceIndex += faceNum;
						startVertexIndex += vertexNum;
					}
				}
			}
		}
	}

	void reverseTriMesh(trimesh::TriMesh* Mesh)
	{
		for (size_t i = 0; i < Mesh->faces.size(); i++)
		{
			int temp = Mesh->faces[i].at(1);
			Mesh->faces[i].at(1) = Mesh->faces[i].at(2);
			Mesh->faces[i].at(2) = temp;
		}
	}

	void mergeTrianglesTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes)
	{
		int totalVertexSize = 0;
		size_t meshSize = inMeshes.size();
		for (size_t i = 0; i < meshSize; ++i)
		{
			trimesh::TriMesh* mesh = inMeshes.at(i);
			if (mesh)
			{
				if (mesh->faces.size() == 0)
					totalVertexSize += (int)mesh->vertices.size();
				else
					totalVertexSize += mesh->vertices.size();
			}
		}

		if (totalVertexSize > 0)
		{
			outMesh->vertices.reserve(totalVertexSize);
			for (size_t i = 0; i < meshSize; ++i)
			{
				trimesh::TriMesh* mesh = inMeshes.at(i);
				if (mesh)
				{
					if (mesh->faces.size() == 0)
					{
						outMesh->vertices.insert(outMesh->vertices.end(), mesh->vertices.begin(), mesh->vertices.end());
					}
					else
					{
						size_t fnum = mesh->faces.size();
						if (fnum > 0)
						{
							for (size_t ii = 0; ii < fnum; ++ii)
							{
								trimesh::TriMesh::Face& face = mesh->faces.at(ii);
								for (int j = 0; j < 3; ++j)
								{
									int index = face[j];
									outMesh->vertices.push_back(mesh->vertices.at(index));
								}
							}
						}
					}
				}
			}
		}
	}

	trimesh::TriMesh* partMesh(const std::vector<int>& indices, trimesh::TriMesh* inMesh)
	{
		trimesh::TriMesh* outMesh = nullptr;
		if (inMesh && indices.size() > 0 && inMesh->vertices.size() > 0)
		{
			size_t vertexSize = inMesh->vertices.size();
			outMesh = new trimesh::TriMesh();
			outMesh->faces.resize(indices.size());
			outMesh->vertices.reserve(3 * indices.size());

			std::vector<int> flags(vertexSize, -1);
			int startIndex = 0;
			int k = 0;
			for (int i : indices)
			{
				trimesh::TriMesh::Face& f = inMesh->faces.at(i);
				trimesh::TriMesh::Face& of = outMesh->faces.at(k);
				for (int j = 0; j < 3; ++j)
				{
					int index = f[j];
					if (flags.at(index) < 0)
					{
						flags.at(index) = startIndex++;
						outMesh->vertices.push_back(inMesh->vertices.at(index));
					}

					of[j] = flags.at(index);
				}

				++k;
			}
		}

		return outMesh;
	}

	void dumplicateMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer, float ratio)
	{
		std::clock_t start = clock();
		if (!mesh)
			return;

		size_t vertexNum = mesh->vertices.size();

		struct hash_vec3
		{
			size_t operator()(const trimesh::vec3& v)const
			{
#if _WIN32
				return abs(v.x) * 10000.0f / 23.0f + abs(v.y) * 10000.0f / 19.0f + abs(v.z) * 10000.0f / 17.0f;
#else
				return fabs(v.x) * 10000.0f / 23.0f + fabs(v.y) * 10000.0f / 19.0f + abs(v.z) * 10000.0f / 17.0f;
#endif
			}

		};
		struct equal_vec3
		{
			bool operator()(const trimesh::vec3& v1, const trimesh::vec3& v2) const
			{
				return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
			}
		};
		typedef std::unordered_map<trimesh::vec3, int, hash_vec3, equal_vec3> unique_point;
		unique_point points((int)(vertexNum * ratio) + 1);

		typedef unique_point::iterator point_iterator;

		size_t faceNum = mesh->faces.size();

		if (vertexNum == 0 || faceNum == 0)
			return;

		trimesh::TriMesh* optimizeMesh = new trimesh::TriMesh();
		bool interuptted = false;

		std::vector<int> vertexMapper;
		vertexMapper.resize(vertexNum, -1);

		if(tracer)
			tracer->formatMessage("dumplicateMesh %d", (int)vertexNum);
		
		size_t pVertex = vertexNum / 20;
		if (pVertex == 0)
			pVertex = vertexNum;

		for (size_t i = 0; i < vertexNum; ++i)
		{
			trimesh::point p = mesh->vertices.at(i);
			point_iterator it = points.find(p);
			if (it != points.end())
			{
				int index = (*it).second;
				vertexMapper.at(i) = index;
			}
			else
			{
				int index = (int)points.size();
				points.insert(unique_point::value_type(p, index));

				vertexMapper.at(i) = index;
			}

			if (i % pVertex == 1)
			{
				if (tracer)
				{
					tracer->formatMessage("dumplicateMesh %i", (int)i);
					tracer->progress((float)i / (float)vertexNum);
					if (tracer->interrupt())
					{
						interuptted = true;
						break;
					}
				}
#if USE_SPYCC
				SESSION_TICK("dumplicateMesh")
#endif
			}
		}

		if (tracer)
			tracer->formatMessage("dumplicateMesh over %d", (int)points.size());

		if (interuptted)
		{
			delete optimizeMesh;
			return;
		}
		trimesh::TriMesh* omesh = optimizeMesh;
		omesh->vertices.resize(points.size());
		for (point_iterator it = points.begin(); it != points.end(); ++it)
		{
			omesh->vertices.at(it->second) = it->first;
		}

		omesh->faces = mesh->faces;
		for (size_t i = 0; i < faceNum; ++i)
		{
			trimesh::TriMesh::Face& of = omesh->faces.at(i);
			for (int j = 0; j < 3; ++j)
			{
				int index = of[j];
				of[j] = vertexMapper[index];
			}
		}

		mesh->vertices.swap(omesh->vertices);
		mesh->faces.swap(omesh->faces);
		mesh->need_bbox();

		delete omesh;

		clock_t end = clock();
		double endtime = (double)(end - start);
//		std::printf("total time1: %f\n", endtime);
	}

	void removeNorFaces(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
	{
		int nf = mesh->faces.size();
		std::vector<char> del_facet(nf, 0);

		if (tracer)
			tracer->formatMessage("before faces %d", (int)nf);
#if _WIN32
#if defined(_OPENMP)
		#pragma omp parallel for
#endif
#endif
		for (int i = 0; i < nf; i++) {
			trimesh::TriMesh::Face& facet = mesh->faces[i];
			if (mesh->vertices[facet[0]] == mesh->vertices[facet[1]] || mesh->vertices[facet[1]] == mesh->vertices[facet[2]] || mesh->vertices[facet[0]] == mesh->vertices[facet[2]])
			{
				del_facet[i] = 1;
			}
		}

		std::vector<trimesh::TriMesh::Face> validFaces;
		for (int i = 0; i < mesh->faces.size(); ++i)
		{
			if (del_facet[i] == 0)
				validFaces.push_back(mesh->faces.at(i));
		}
		mesh->faces.swap(validFaces);

		if (tracer)
			tracer->formatMessage("after faces %d", (int)mesh->faces.size());
	}


	void mergeTriMesh(trimesh::TriMesh* outMesh, std::vector<trimesh::TriMesh*>& inMeshes, const trimesh::fxform& globalMatrix, bool fanzhuan)
	{
		assert(outMesh);
		size_t totalVertexSize = outMesh->vertices.size();
		size_t totalTriangleSize = outMesh->faces.size();
		size_t addVertexSize = 0;
		size_t addTriangleSize = 0;
		size_t meshSize = inMeshes.size();
		for (size_t i = 0; i < meshSize; ++i)
		{
			addVertexSize += inMeshes.at(i)->vertices.size();
			addTriangleSize += inMeshes.at(i)->faces.size();
		}
		totalVertexSize += addVertexSize;
		totalTriangleSize += addTriangleSize;

		if (addVertexSize > 0 && addTriangleSize > 0)
		{
			outMesh->vertices.reserve(totalVertexSize);
			outMesh->faces.reserve(totalTriangleSize);

			int startFaceIndex = outMesh->faces.size();
			int startVertexIndex = outMesh->vertices.size();;
			for (size_t i = 0; i < meshSize; ++i)
			{
				trimesh::TriMesh* mesh = inMeshes.at(i);
				int vertexNum = (int)mesh->vertices.size();
				int faceNum = (int)mesh->faces.size();
				if (vertexNum > 0 && faceNum > 0)
				{
					for (int k = 0; k < vertexNum; k++)
					{
						trimesh::vec3 qPoint = mesh->vertices[k];

						qPoint = globalMatrix * qPoint;

						outMesh->vertices.emplace_back(qPoint);

						if (outMesh->bbox.min.x > qPoint.x) outMesh->bbox.min.x = qPoint.x;
						if (outMesh->bbox.min.y > qPoint.y) outMesh->bbox.min.y = qPoint.y;
						if (outMesh->bbox.min.z > qPoint.z) outMesh->bbox.min.z = qPoint.z;

						if (outMesh->bbox.max.x < qPoint.x) outMesh->bbox.max.x = qPoint.x;
						if (outMesh->bbox.max.y < qPoint.y) outMesh->bbox.max.y = qPoint.y;
						if (outMesh->bbox.max.z < qPoint.z) outMesh->bbox.max.z = qPoint.z;
					}

					//outMesh->vertices.insert(outMesh->vertices.end(), mesh->vertices.begin(), mesh->vertices.end());
					outMesh->faces.insert(outMesh->faces.end(), mesh->faces.begin(), mesh->faces.end());

					int endFaceIndex = startFaceIndex + faceNum;
					if (startVertexIndex > 0)
					{
						for (int ii = startFaceIndex; ii < endFaceIndex; ++ii)
						{
							trimesh::TriMesh::Face& face = outMesh->faces.at(ii);
							for (int j = 0; j < 3; ++j)
							{
								face[j] += startVertexIndex;
							}
							if (fanzhuan)
							{
								int t = face[1];
								face[1] = face[2];
								face[2] = t;
							}
						}
					}

					startFaceIndex += faceNum;
					startVertexIndex += vertexNum;
				}
			}
		}
	}

	trimesh::vec3 moveTrimesh2Center(trimesh::TriMesh* mesh, bool zZero)
	{
		trimesh::box3 b = mesh->bbox;

		trimesh::vec3 size = b.size() / 2.0f;
		size.x = 0.0f;
		size.y = 0.0f;
		if (!zZero)
			size.z = 0.0f;

		trimesh::vec3 offset = size - b.center();
		trimesh::trans(mesh, offset);

		return offset;
	}

	void moveMeshes2BoxCenter(std::vector<trimesh::TriMesh*> meshes, const trimesh::box3& box, bool zZero)
	{
		trimesh::box3 b;
		for (trimesh::TriMesh* mesh : meshes)
		{
			mesh->need_bbox();
			b += mesh->bbox;
		}

		trimesh::vec3 offset = box.center() - b.center();
		if (zZero)
		{
			offset.z = 0.0f - b.min.z;
		}
		for (trimesh::TriMesh* mesh : meshes)
			trimesh::trans(mesh, offset);
	}

	void convertUV2Azi(trimesh::TriMesh* mesh)
	{
		if (!mesh || (mesh->cornerareas.size() != mesh->vertices.size()))
			return;

		size_t size = mesh->cornerareas.size();
		for (size_t i = 0; i < size; ++i)
			mesh->cornerareas.at(i) = BallCreator::equ2azi(mesh->cornerareas.at(i));
	}

	void convertUV2Equ(trimesh::TriMesh* mesh)
	{
		if (!mesh || (mesh->cornerareas.size() != mesh->vertices.size()))
			return;

		size_t size = mesh->cornerareas.size();
		for (size_t i = 0; i < size; ++i)
			mesh->cornerareas.at(i) = BallCreator::azi2equ(mesh->cornerareas.at(i));
	}

	void flipZ(trimesh::TriMesh* mesh)
	{
		if (!mesh)
			return;

		mesh->need_bbox();
		trimesh::box3 b = mesh->bbox;
		trimesh::xform xf = trimesh::xform::trans(b.center()) 
			* trimesh::xform::rot(M_PIf, trimesh::vec3(1.0f, 0.0f, 0.0f))
			* trimesh::xform::trans(-b.center());
		trimesh::apply_xform(mesh, xf);
	}

	trimesh::fxform beltXForm(const trimesh::vec3& offset, float angle)
	{
		float theta = angle * M_PIf / 180.0f;

		trimesh::fxform xf0 = trimesh::fxform::trans(offset);

		trimesh::fxform xf1 = trimesh::fxform::identity();
		xf1(2, 2) = 1.0f / sinf(theta);
		xf1(1, 2) = -1.0f / tanf(theta);

		trimesh::fxform xf2 = trimesh::fxform::identity();
		xf2(2, 2) = 0.0f;
		xf2(1, 1) = 0.0f;
		xf2(2, 1) = -1.0f;
		xf2(1, 2) = 1.0f;

		trimesh::fxform xf3 = trimesh::fxform::trans(0.0f, 0.0f, 200.0f);

		trimesh::fxform xf = xf3 * xf2 * xf1 * xf0;
		//trimesh::fxform xf = xf3 * xf0;
		return xf;
	}

	void fillTriangleSoupFaceIndex(trimesh::TriMesh* mesh)
	{
		if (!mesh || mesh->faces.size() != 0)
			return;

		size_t size = mesh->vertices.size();
		if (size % 3 || size < 3)
			return;

		size /= 3;
		mesh->faces.resize(size);
		for (size_t i = 0; i < size; ++i)
		{
			trimesh::TriMesh::Face& face = mesh->faces.at(i);
			face[0] = (int)(3 * i);
			face[1] = (int)(3 * i + 1);
			face[2] = (int)(3 * i + 2);
		}
	}

	void loadTrimesh(std::fstream& in, trimesh::TriMesh& mesh)
	{
		loadVectorT(in, mesh.vertices);
		loadVectorT(in, mesh.faces);
	}

	void saveTrimesh(std::fstream& out, trimesh::TriMesh& mesh)
	{
		saveVectorT(out, mesh.vertices);
		saveVectorT(out, mesh.faces);
	}

	trimesh::dvec3 vec32dvec3(const trimesh::vec3& v)
	{
		return trimesh::dvec3(v.x, v.y, v.z);
	}
}
