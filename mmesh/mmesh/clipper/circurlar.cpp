#include "circurlar.h"

namespace mmesh
{
	void loopPolyTree(polyNodeFunc func, ClipperLib::PolyTree* polyTree)
	{
		if (!polyTree) return;

		polyNodeFunc cFunc = [&cFunc, &func](ClipperLib::PolyNode* node) {
			func(node);

			for (ClipperLib::PolyNode* n : node->Childs)
				cFunc(n);
		};

		cFunc(polyTree);
	}
}