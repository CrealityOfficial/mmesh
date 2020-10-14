#include "triangleaabbnode.h"

namespace mmesh
{

    TriangleAABBNode::TriangleAABBNode()
        :m_left(nullptr)
        , m_right(nullptr)
    {
    }

    TriangleAABBNode::~TriangleAABBNode()
    {
    }

    const trimesh::box3& TriangleAABBNode::box()
    {
        return m_box;
    }

    void TriangleAABBNode::expand(int* first, int* beyond, size_t range, std::vector<trimesh::box3>& boxMap)
    {
        m_box = calBox(first, beyond, boxMap);

        // sort primitives along longest axis aabb
        sortPrimitive(first, beyond, boxMap);

        switch (range)
        {
        case 2:
            m_left = (void*)(*first);
            m_right = (void*)(*(++first));
            break;
        case 3:
            m_left = (void*)(*first);
            m_right = static_cast<TriangleAABBNode*>(this) + 1;
            right().expand(first + 1, beyond, 2, boxMap);
            break;
        default:
            const std::size_t newRange = range / 2;
            m_left = static_cast<TriangleAABBNode*>(this) + 1;
            m_right = static_cast<TriangleAABBNode*>(this) + newRange;
            left().expand(first, first + newRange, newRange, boxMap);
            right().expand(first + newRange, beyond, range - newRange, boxMap);
        }
    }

    TriangleAABBNode& TriangleAABBNode::left()
    {
        return *static_cast<TriangleAABBNode*>(m_left);
    }

    TriangleAABBNode& TriangleAABBNode::right()
    {
        return *static_cast<TriangleAABBNode*>(m_right);
    }

    int TriangleAABBNode::lData()
    {
        long l = (long)m_left;
        return (int)l;
    }

    int TriangleAABBNode::rData()
    {
        long r = (long)m_right;
        return (int)r;
    }

    trimesh::box3 TriangleAABBNode::calBox(int* first, int* beyond, std::vector<trimesh::box3>& boxMap)
    {
        trimesh::box3 box = boxMap.at(*first);
        for (++first; first != beyond; ++first)
        {
            box += boxMap.at(*first);
        }
        return box;
    }

    void TriangleAABBNode::sortPrimitive(int* first, int* beyond, std::vector<trimesh::box3>& boxMap)
    {
        int* middle = first + (beyond - first) / 2;

        trimesh::vec3 size = m_box.size();
        const double dx = size.x;
        const double dy = size.y;
        const double dz = size.z;

        int dir = 0;
        if (dx >= dy)
        {
            if (dx >= dz)
            {
                dir = 0;
            }
            else // dz>dx and dx>=dy
            {
                dir = 2;
            }
        }
        else // dy>dx
        {
            if (dy >= dz)
            {
                dir = 1;
            }
            else  // dz>dy and dy>dx
            {
                dir = 2;
            }
        }

        auto lessX = [&boxMap](int i, int j) ->bool {
            trimesh::vec3 ci = boxMap.at(i).center();
            trimesh::vec3 cj = boxMap.at(i).center();
            return ci.x < cj.x;
        };
        auto lessY = [&boxMap](int i, int j) ->bool {
            trimesh::vec3 ci = boxMap.at(i).center();
            trimesh::vec3 cj = boxMap.at(i).center();
            return ci.y < cj.y;
        };
        auto lessZ = [&boxMap](int i, int j) ->bool {
            trimesh::vec3 ci = boxMap.at(i).center();
            trimesh::vec3 cj = boxMap.at(i).center();
            return ci.z < cj.z;
        };
        switch (dir)
        {
        case 0: // sort along x
            std::nth_element(first, middle, beyond, lessX);
            break;
        case 1: // sort along y
            std::nth_element(first, middle, beyond, lessY);
            break;
        case 2: // sort along z
            std::nth_element(first, middle, beyond, lessZ);
            break;
        default:
            break;
        }
    }

    void TriangleAABBNode::traversal(trimesh::vec3& p, trimesh::vec3& n, size_t range, std::vector<int>& indices)
    {
        // Recursive traversal
        switch (range)
        {
        case 2:
            indices.push_back(lData());
            indices.push_back(rData());
            break;
        case 3:
            indices.push_back(lData());
            if (right().intersect(p, n))
            {
                right().traversal(p, n, range, indices);
            }
            break;
        default:
            if (left().intersect(p, n))
            {
                left().traversal(p, n, range / 2, indices);
                if (right().intersect(p, n))
                {
                    right().traversal(p, n, range - range / 2, indices);
                }
            }
            else if (right().intersect(p, n))
            {
                right().traversal(p, n, range - range / 2, indices);
            }
        }
    }

    void TriangleAABBNode::ztraversal(trimesh::vec3& p, size_t range, std::vector<int>& indices)
    {
        // Recursive traversal
        switch (range)
        {
        case 2:
            indices.push_back(lData());
            indices.push_back(rData());
            break;
        case 3:
            indices.push_back(lData());
            if (right().zintersect(p))
            {
                right().ztraversal(p, 2, indices);
            }
            break;
        default:
            if (left().zintersect(p))
            {
                left().ztraversal(p, range / 2, indices);
                if (right().zintersect(p))
                {
                    right().ztraversal(p, range - range / 2, indices);
                }
            }
            else if (right().zintersect(p))
            {
                right().ztraversal(p, range - range / 2, indices);
            }
        }
    }

    bool TriangleAABBNode::intersect(trimesh::vec3& p, trimesh::vec3& n)
    {
        return true;
    }

    bool TriangleAABBNode::zintersect(trimesh::vec3& p)
    {
        return p.x >= m_box.min.x && p.x <= m_box.max.x && p.y >= m_box.min.y && p.y <= m_box.max.y;
    }

}