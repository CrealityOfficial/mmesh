#include "mmesh/trimesh/trimeshutil.h"
#include "trimesh2/TriMesh.h"
#include "trimesh2/TriMesh_algo.h"
#include "trimesh2/quaternion.h"

#include <unordered_map>
#include <assert.h>

#include "mmesh/create/ballcreator.h"
#include "ccglobal/tracer.h"
#include "ccglobal/spycc.h"
#include "ccglobal/platform.h"
#include "ccglobal/log.h"
#include "mmesh/create/createcylinder.h"
#include "mmesh/trimesh/meshtopo.h"
#include "mmesh/util/mnode.h"
#include "mmesh/util/dumplicate.h"
#include <ctime>

namespace mmesh
{
	trimesh::TriMesh* LinePlusline2Model(std::vector<edge>& edges, float _radius)
	{
		trimesh::TriMesh* outMesh = new trimesh::TriMesh();
		std::vector<trimesh::TriMesh*> resuMerge;
		for (int i = 0; i < edges.size(); i++)
		{
			trimesh::vec3 centerPoint0(edges.at(i).p0);
			trimesh::vec3 centerPoint1(edges.at(i).p1);
			trimesh::vec3 vmiddle = (centerPoint0 + centerPoint1) / 2;
			trimesh::vec3 v0(centerPoint1 - centerPoint0);
			float len = trimesh::dist(centerPoint0, centerPoint1);
			//len += 1.0;
			trimesh::TriMesh* symesh  = createSoupCylinder(10, _radius, len, vmiddle, v0);
			resuMerge.emplace_back(symesh);
		}
		mmesh::mergeTriMesh(outMesh, resuMerge, false);
		return outMesh;
	}
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

	bool testNeedfitMesh(trimesh::TriMesh* mesh, float& scale)
	{
		if (!mesh)
			return false;

		mesh->need_bbox();
		trimesh::vec3 size = mesh->bbox.size();

		bool needScale = false;
		scale = 1.0f;
		if (size.max() > 1000.0f)
		{
			needScale = true;
			scale = 100.0f / size.max();
		}
		else if (size.min() < 1.0f && size.min() > 0.00001f)
		{
			needScale = true;
			scale = 100.0f / size.min();
		}

		return needScale;
	}

	bool dumplicateMeshExTest(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer, float ratio)	
	{
		return dumplicateMesh(mesh,tracer,ratio);
	}

	bool dumplicateMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer, float ratio)
	{
        return hashMesh<hash_func1>(mesh, tracer, ratio);
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

	void removeInvalidVertex(trimesh::TriMesh* mesh)
	{
		if(!mesh)
			return;

		int vnum = (int)mesh->vertices.size();
		int fnum = (int)mesh->faces.size();
		if(vnum == 0 || fnum == 0)
			return;

		int index = 0;
		std::vector<int> flags(vnum, -1);

		auto ff = [](const trimesh::vec3& v)->bool {
			for (int i = 0; i < 3; ++i)
			{
                if (std::isnan(v[i]))
					return true;

                if (!std::isnormal(v[i]))
				{
					if (v[i] != 0.0f)
						return true;
				}

				if (std::abs(v[i]) > 1e+10)
					return true;
			}
			return false;
		};
		for(int i = 0; i < vnum; ++i)
		{
			const trimesh::vec3& v = mesh->vertices.at(i);
			if (ff(v))
			{
#if _DEBUG
				printf("error vertex [%f %f %f]\n", v.x, v.y, v.z);
#endif
			}
			else
			{
				flags.at(i) = index;
				mesh->vertices.at(index) = v;
				++index;
			}
		}

		if (index != vnum)  //have invlid
		{
			if (index > 0)
				mesh->vertices.resize(index);
			else
				mesh->vertices.clear();

			int findex = 0;
			for (int i = 0; i < fnum; ++i)
			{
				trimesh::TriMesh::Face f = mesh->faces.at(i);
				if ((flags[f.x] >= 0) && (flags[f.y] >= 0) && (flags[f.z] >= 0))
				{
					f.x = flags[f.x];
					f.y = flags[f.y];
					f.z = flags[f.z];
					mesh->faces.at(findex) = f;
					++findex;
				}
			}

			if (findex > 0)
				mesh->faces.resize(findex);
			else
				mesh->faces.clear();
		}
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

	trimesh::vec3 moveToOriginal(trimesh::TriMesh* mesh)
	{
		const trimesh::box3& box = mesh->bbox;

		trimesh::vec3 offset = box.center();
		offset.z = box.min.z / 2.0f;

		trimesh::trans(mesh, -offset);
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

	trimesh::fxform beltXForm(const trimesh::vec3& offset, float angle, int beltType)
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

		if (1 == beltType)
		{
			trimesh::fxform xf3 = trimesh::fxform::trans(0.0f, 0.0f, 0.0f);

			trimesh::fxform xf = xf3 * xf2 * xf1 * xf0;
			//trimesh::fxform xf = xf3 * xf0;
			return xf;
		} 
		else
		{
			trimesh::fxform xf3 = trimesh::fxform::trans(0.0f, 0.0f, 200.0f);
			trimesh::fxform xf = xf3 * xf2 * xf1 * xf0;
			return xf;
		}

	}

	trimesh::fxform xformFromPlane(const trimesh::vec3& pos, const trimesh::vec3& normal)
	{
		trimesh::vec3 nn = trimesh::normalized(normal);
		trimesh::quaternion q = trimesh::quaternion::fromDirection(nn, trimesh::vec3(0.0f, 0.0f, 1.0f));
		trimesh::fxform m1 = mmesh::fromQuaterian(q);
		return m1 * trimesh::fxform::trans(pos);
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

	void indexTriangle2Soup(trimesh::TriMesh* mesh)
	{
		if (!mesh || mesh->faces.size() == 0)
			return;

		int size = (int)mesh->faces.size();
		std::vector<trimesh::vec3> positions(3 * size);
		for (int i = 0; i < size; ++i)
		{
			trimesh::TriMesh::Face& face = mesh->faces.at(i);
			for(int j = 0; j <3; ++j)
				positions.at(3 * i + j) = mesh->vertices.at(face[j]);
		}
		mesh->vertices.swap(positions);
		mesh->faces.clear();
		fillTriangleSoupFaceIndex(mesh);
		if (mesh->faceUVs.size() > 0 && mesh->UVs.size() > 0)
		{
			std::vector<trimesh::vec2> uvs(3 * size);
			for (int i = 0; i < size; ++i)
			{
				trimesh::TriMesh::Face& faceUV = mesh->faceUVs.at(i);
				for (int j = 0; j < 3; ++j)
				{
					uvs.at(3 * i + j) = mesh->UVs.at(faceUV[j]);
				}
				mesh->faceUVs.at(i) = mesh->faces.at(i);
			}
			mesh->UVs.swap(uvs);
		}
	}

	void meshMerge(TriMeshPointer& outMesh,const std::vector<TriMeshPointer>& inMeshes, const trimesh::fxform& globalMatrix, bool fanzhuan, ccglobal::Tracer* progressor)
	{
		std::vector<trimesh::TriMesh*> _inMeshes;
		for (auto mesh : inMeshes)
		{
			_inMeshes.push_back(mesh.get());
				
		}
		mergeTriMesh(outMesh.get(), _inMeshes, globalMatrix, fanzhuan);
	}

	std::vector < std::vector<TriMeshPointer>> meshSplit(const std::vector<TriMeshPointer>& meshes, ccglobal::Tracer* progressor)
	{
		std::vector < std::vector<TriMeshPointer>> outMeshes;
		outMeshes.reserve(meshes.size());

		for (auto mesh : meshes)
		{
			int originV = mesh->vertices.size();
			originV = originV >= 0 ? originV : 1;
			mmesh::dumplicateMesh(mesh.get(), nullptr);;

			if (!mesh || mesh->faces.size() <= 0)
			{
				continue;
			}

			if (progressor)
			{
				progressor->progress(0.1f);
			}

			mmesh::MeshTopo topo;
			topo.build(mesh.get());
			if (progressor)
			{
				progressor->progress(0.3f);

				if (progressor->interrupt())
				{
					return outMeshes;
				}
			}

			int faceNum = (int)mesh->faces.size();
			std::vector<bool> visitFlags(faceNum, false);

			std::vector<int> visitStack;
			std::vector<int> nextStack;

			std::vector<std::vector<int>> parts;
			parts.reserve(100);
			for (int faceID = 0; faceID < faceNum; ++faceID)
			{
				if (visitFlags.at(faceID) == false)
				{
					visitFlags.at(faceID) = true;
					visitStack.push_back(faceID);

					std::vector<int> facesChunk;
					facesChunk.push_back(faceID);
					while (!visitStack.empty())
					{
						int seedSize = (int)visitStack.size();
						for (int seedIndex = 0; seedIndex < seedSize; ++seedIndex)
						{
							int cFaceID = visitStack.at(seedIndex);
							trimesh::ivec3& oppoHalfs = topo.m_oppositeHalfEdges.at(cFaceID);
							for (int halfID = 0; halfID < 3; ++halfID)
							{
								int oppoHalf = oppoHalfs.at(halfID);
								if (oppoHalf >= 0)
								{
									int oppoFaceID = topo.faceid(oppoHalf);
									if (visitFlags.at(oppoFaceID) == false)
									{
										nextStack.push_back(oppoFaceID);
										facesChunk.push_back(oppoFaceID);
										visitFlags.at(oppoFaceID) = true;
									}
								}
							}
						}

						visitStack.swap(nextStack);
						nextStack.clear();
					}

					parts.push_back(std::vector<int>());
					parts.back().swap(facesChunk);
				}
				else
				{
					visitFlags.at(faceID) = true;
				}

				if ((faceID + 1) % 100 == 0)
				{
					if (progressor->interrupt())
					{
						return outMeshes;
					}
				}

			}

			std::vector<trimesh::TriMesh*> meshes;
			size_t partSize = parts.size();
			for (size_t i = 0; i < partSize; ++i)
			{
				if (parts.at(i).size() > 10)
				{
					trimesh::TriMesh* outMesh = mmesh::partMesh(parts.at(i), mesh.get());
					if (outMesh) meshes.push_back(outMesh);
				}
			}

			//merge small ones
			int tSize = (int)meshes.size();
			int maxCount = 0;
			for (int i = 0; i < tSize; ++i)
			{
				if (maxCount < (int)meshes.at(i)->vertices.size())
					maxCount = (int)meshes.at(i)->vertices.size();
			}

			//���Ըı� ��������150�� ������50��
			//int smallCount = (int)((float)maxCount * 0.05f);
			const int smallV = 150;
			const int samllF = 50;
			std::vector<trimesh::TriMesh*> allInOne;
			//std::vector<trimesh::TriMesh*> validMeshes;
			std::vector<TriMeshPointer> validMeshes;
			for (int i = 0; i < tSize; ++i)
			{
				//if ((int)meshes.at(i)->vertices.size() < smallCount)
				float ratio = meshes.at(i)->vertices.size() * 1.0f / originV;
				if ((int)meshes.at(i)->vertices.size() < smallV
					&& (int)meshes.at(i)->faces.size() < samllF
					&& ratio< 0.1f)
					allInOne.push_back(meshes.at(i));
				else
					validMeshes.push_back(TriMeshPointer(meshes.at(i)));
			}

			if (allInOne.size() > 0)
			{
				trimesh::TriMesh* newMesh = new trimesh::TriMesh();
				mmesh::mergeTriMesh(newMesh, allInOne);
				validMeshes.push_back(TriMeshPointer(newMesh));

				for (trimesh::TriMesh* m : allInOne)
					delete m;
				allInOne.clear();
			}
			outMeshes.push_back(validMeshes);
		}
		
		return outMeshes;
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
