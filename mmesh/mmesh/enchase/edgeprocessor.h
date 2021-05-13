#ifndef ENCHASE_EDGEPROCESSOR_1603162255993_H
#define ENCHASE_EDGEPROCESSOR_1603162255993_H
#include "enum.h"
#include "imagematrix.h"

namespace enchase
{
	void edgeProcess(MatrixF* matrix, EdgeType type);

	void zeroEdge(MatrixF* matrix);
	void interpolationEdge(MatrixF* matrix);
}

#endif // ENCHASE_EDGEPROCESSOR_1603162255993_H