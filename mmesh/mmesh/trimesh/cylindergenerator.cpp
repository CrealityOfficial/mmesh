#include "cylindergenerator.h"
#include "mmesh/trimesh/quaternion.h"

namespace mmesh
{
	CylinderGenerator::CylinderGenerator(int n, float theta)
	{
        int hPart = n;
        if (hPart < 3)
            hPart = 3;

        theta *= M_PIf / 180.0f;
        float deltaTheta = M_PIf * 2.0f / (float)(hPart);
        std::vector<float> cosValue;
        std::vector<float> sinValue;
        for (int i = 0; i < hPart; ++i)
        {
            cosValue.push_back(cosf(deltaTheta * (float)i + theta));
            sinValue.push_back(sinf(deltaTheta * (float)i + theta));
        }

        for (int i = 0; i < hPart; ++i)
        {
            m_baseNormals.push_back(trimesh::vec3(cosValue[i], sinValue[i], 0.0f));
        }

        m_hPart = hPart;
	}

	CylinderGenerator::~CylinderGenerator()
	{

	}

    void CylinderGenerator::generate(const trimesh::vec3& start, const trimesh::vec3& end, float radius,
        std::vector<trimesh::vec3>& triangles, bool withoutBU)
    {
        trimesh::vec3 dir = end - start;
        trimesh::normalize(dir);
        trimesh::quaternion q = trimesh::quaternion::fromDirection(dir, trimesh::vec3(0.0f, 0.0f, 1.0f));

        int vertexNum = 2 * m_hPart;
        std::vector<trimesh::vec3> points(vertexNum);
        int faceNum = 4 * m_hPart - 4;

        int vertexIndex = 0;
        for (int i = 0; i < m_hPart; ++i)
        {
            trimesh::vec3 n = q * m_baseNormals[i];
            points.at(vertexIndex) = start + n * radius;
            ++vertexIndex;
            points.at(vertexIndex) = end + n * radius;
            ++vertexIndex;
        }

        triangles.reserve(3 * faceNum);

        auto fvindex = [](int layer, int index)->int {
            return layer + 2 * index;
        };

        for (int i = 0; i < m_hPart; ++i)
        {
            int v1 = fvindex(0, i);
            int v2 = fvindex(1, i);
            int v3 = fvindex(0, (i + 1) % m_hPart);
            int v4 = fvindex(1, (i + 1) % m_hPart);

            triangles.push_back(points.at(v1));
            triangles.push_back(points.at(v3));
            triangles.push_back(points.at(v2));
            triangles.push_back(points.at(v2));
            triangles.push_back(points.at(v3));
            triangles.push_back(points.at(v4));
        }

        if (!withoutBU)
        {
            for (int i = 1; i < m_hPart - 1; ++i)
            {
                triangles.push_back(points.at(0));
                triangles.push_back(points.at(fvindex(0, i + 1)));
                triangles.push_back(points.at(fvindex(0, i)));
            }

            for (int i = 1; i < m_hPart - 1; ++i)
            {
                triangles.push_back(points.at(1));
                triangles.push_back(points.at(fvindex(1, i)));
                triangles.push_back(points.at(fvindex(1, i + 1)));
            }
        }
    }
}