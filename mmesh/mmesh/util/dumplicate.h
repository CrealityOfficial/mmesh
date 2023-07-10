#ifndef MMESH_MNODE_DUMPLICATE_1622032440408_H
#define MMESH_MNODE_DUMPLICATE_1622032440408_H
#include "trimesh2/TriMesh.h"
#include <unordered_map>
#include "ccglobal/tracer.h"
#include "trimesh2/TriMesh_algo.h"
#include "mmesh/trimesh/trimeshutil.h"
namespace mmesh
{

	void weldingMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer = nullptr);

    //δ�õ��Ķ�������Ч�Ķ����滻
    void removeNorVector2(trimesh::TriMesh* mesh);

    struct hash_vec3 {
        size_t operator()(const trimesh::vec3& v)const
        {
            return std::abs(v.x)* 10000.0f / 23.0f + std::abs(v.y) * 10000.0f / 19.0f + std::abs(v.z) * 10000.0f / 17.0f;
        }
    };

    struct hash_func1 {
        size_t operator()(const trimesh::vec3& v)const
        {
            return (int(v.x * 99971)) ^ (int(v.y * 99989)) ^ (int(v.z * 99991));
        }
    };

    struct hash_func2 {
        size_t operator()(const trimesh::vec3& v)const
        {
            return (int(v.x * 99971)) ^ (int(v.y * 99989) << 2) ^ (int(v.z * 99991) << 3);
        }
    };

    bool compareTriMesh(const trimesh::TriMesh& mesh1, const trimesh::TriMesh& mesh2);

	template<class T>
	bool hashMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
	{
        if (!mesh)
            return false;

        float sValue = 1.0f;
        bool needScale = mmesh::testNeedfitMesh(mesh, sValue);

        if (needScale)
            trimesh::apply_xform(mesh, trimesh::xform::scale(sValue));

        size_t vertexNum = mesh->vertices.size();

        struct equal_vec3 {
            bool operator()(const trimesh::vec3& v1, const trimesh::vec3& v2) const
            {
                return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
            }
        };
        typedef std::unordered_map<trimesh::vec3, int, T, equal_vec3> unique_point;
        unique_point points((int)(vertexNum * 0.3) + 1);

        
        size_t faceNum = mesh->faces.size();

        if (vertexNum == 0 || faceNum == 0)
            return false;

        trimesh::TriMesh * optimizeMesh = new trimesh::TriMesh();
        bool interuptted = false;

        std::vector<int> vertexMapper;
        vertexMapper.resize(vertexNum, -1);

        if (tracer)
            tracer->formatMessage("dumplicateMesh %d", (int)vertexNum);

        size_t pVertex = vertexNum / 20;
        if (pVertex == 0)
            pVertex = vertexNum;

        for (size_t i = 0; i < vertexNum; ++i) {
            trimesh::point p = mesh->vertices.at(i);
            auto it = points.find(p);
            if (it != points.end()) {
                int index = (*it).second;
                vertexMapper.at(i) = index;
            } else {
                int index = (int)points.size();
                points.insert(unique_point::value_type(p, index));

                vertexMapper.at(i) = index;
            }

            if (i % pVertex == 1) {
                if (tracer) {
                    tracer->formatMessage("dumplicateMesh %i", (int)i);
                    tracer->progress((float)i / (float)vertexNum);
                    if (tracer->interrupt()) {
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

        if (interuptted) {
            delete optimizeMesh;
            return false;
        }
        trimesh::TriMesh* omesh = optimizeMesh;
        omesh->vertices.resize(points.size());
        for (auto it = points.begin(); it != points.end(); ++it) {
            omesh->vertices.at(it->second) = it->first;
        }

        omesh->faces = mesh->faces;
        for (size_t i = 0; i < faceNum; ++i) {
            trimesh::TriMesh::Face& of = omesh->faces.at(i);
            for (int j = 0; j < 3; ++j) {
                int index = of[j];
                of[j] = vertexMapper[index];
            }
        }

        mesh->vertices.swap(omesh->vertices);
        mesh->faces.swap(omesh->faces);
        mesh->flags.clear();

        if (needScale)
            trimesh::apply_xform(mesh, trimesh::xform::scale(1.0f / sValue));

        mesh->clear_bbox();
        mesh->need_bbox();

        delete omesh;
        return true;
	}
}

#endif // MMESH_MNODE_DUMPLICATE_1622032440408_H