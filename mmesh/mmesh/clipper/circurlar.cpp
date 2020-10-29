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
}