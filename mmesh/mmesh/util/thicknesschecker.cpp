#include "thicknesschecker.h"


ThicknessChecker::ThicknessChecker()
{
}

ThicknessChecker::~ThicknessChecker()
{
    clearModel();
}

void ThicknessChecker::addModel(std::string uid, trimesh::TriMesh* modelMesh)
{
    if (m_mapofModelTree.find(uid) == m_mapofModelTree.end())
    {
        ModelAdjacentOctree* MAOTree = new ModelAdjacentOctree(modelMesh);
        m_mapofModelTree[uid] = MAOTree;
    }
}

void ThicknessChecker::removeModel(std::string uid)
{
    if (m_mapofModelTree[uid])
    {
        delete m_mapofModelTree[uid];
        m_mapofModelTree[uid] = nullptr;
    }

    m_mapofModelTree.erase(uid);
}

void ThicknessChecker::clearModel()
{
    for (auto itr = m_mapofModelTree.begin(); itr != m_mapofModelTree.end(); itr++)
    {
        delete itr->second;
    }

    m_mapofModelTree.clear();
}

bool ThicknessChecker::hasModel(std::string uid)
{
    return m_mapofModelTree.find(uid) != m_mapofModelTree.end();
}

int ThicknessChecker::getModelTreeDepth(std::string uid)
{
    auto itr = m_mapofModelTree.find(uid);
    if (itr != m_mapofModelTree.end())
        return itr->second->getTreeDepth();

    return -1;
}

bool ThicknessChecker::lineCollide(std::string uid, trimesh::dvec3 linePos, trimesh::dvec3 lineDir, std::vector<std::pair<int, trimesh::dvec3>>& intersectedFaceIDNPosArray)
{
    auto itr = m_mapofModelTree.find(uid);
    if (itr != m_mapofModelTree.end())
        return itr->second->lineCollide(linePos, lineDir, intersectedFaceIDNPosArray);

    return false;
}
