#include "circurlar.h"

namespace mmesh
{
	void loopPolyTree(polyNodeFunc func, ClipperLib::PolyNode* polyNode)
	{
		if (!polyNode) return;

		polyNodeFunc cFunc = [&cFunc, &func](ClipperLib::PolyNode* node) {
			func(node);

			for (ClipperLib::PolyNode* n : node->Childs)
				cFunc(n);
		};

		cFunc(polyNode);
	}

	void level2PolyNode(polyNodeFunc func, ClipperLib::PolyNode* polyNode)
	{
		if (!polyNode) return;

		int index = 0;
		polyNodeFunc cFunc = [&cFunc, &func](ClipperLib::PolyNode* node) {
			func(node);

			for (ClipperLib::PolyNode* n : node->Childs)
				cFunc(n);
		};

		cFunc(polyNode);
	}

	void seperatePolyTree(ClipperLib::PolyTree* polyTree,
		std::vector<ClipperLib::Path*>& exterior, std::vector<ClipperLib::Path*>& interior)
	{
		polyNodeFunc func = [&func, &exterior, &interior](ClipperLib::PolyNode* node) {
			if (!node->IsHole())
				exterior.push_back(&node->Contour);
			if (node->IsHole() && node->Parent)
				interior.push_back(&node->Contour);
		};

		mmesh::loopPolyTree(func, polyTree);
	}
}