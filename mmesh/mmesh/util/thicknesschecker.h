#ifndef THICKNESSCHECKER_H
#define THICKNESSCHECKER_H

#include <map>

#include "adjacentoctree.h"


class ThicknessChecker
{
public:
	ThicknessChecker();
	~ThicknessChecker();

	void addModel(std::string uid, trimesh::TriMesh* modelMesh);
	void removeModel(std::string uid);
	void clearModel();
	bool hasModel(std::string uid);

	int getModelTreeDepth(std::string uid);

	bool lineCollide(std::string uid, trimesh::dvec3 linePos, trimesh::dvec3 lineDir, std::vector<std::pair<int, trimesh::dvec3>>& intersectedFaceIDNPosArray);

private:
	std::map<std::string, ModelAdjacentOctree*> m_mapofModelTree;
};

#endif // THICKNESSCHECKER_H