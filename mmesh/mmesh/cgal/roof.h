#ifndef FMESH_ROOF_1605318972342_H
#define FMESH_ROOF_1605318972342_H
#include "trimesh2/Vec.h"
#include <clipper/clipper.hpp>

//seperate
struct PolyPair
{
	bool clockwise;
	ClipperLib::PolyNode* outer;
	std::vector<ClipperLib::PolyNode*> inner;
};
void seperate1423(ClipperLib::PolyTree* polyTree, std::vector<PolyPair*>& polyPairs);
void seperate1234(ClipperLib::PolyTree* polyTree, std::vector<PolyPair*>& polyPairs);

namespace mmesh
{
	void buildRoofs(ClipperLib::PolyTree* polyTree, std::vector<std::vector<trimesh::vec3>*>& patches, double roofHeight, double thickness);

	void roofLine(ClipperLib::PolyTree* polyTree,
		ClipperLib::PolyTree* roof, ClipperLib::PolyTree* roofPoint, ClipperLib::Paths* roofFace, bool onePoly = false);

	void skeletonPoints(ClipperLib::PolyTree* polyTree, ClipperLib::Path* roofPoint);
}

#endif // FMESH_ROOF_1605318972342_H