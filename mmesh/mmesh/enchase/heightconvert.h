#ifndef ENCHASE_HEIGHTCONVERT_1600480088461_H
#define ENCHASE_HEIGHTCONVERT_1600480088461_H
#include "imagematrix.h"

namespace enchase
{
	void normalConvert(MatrixF* matrix, float base, float scale);
	void transparencyConvert(MatrixF* matrix, float base, float scale, float transmittance_1mm);
}

#endif // ENCHASE_HEIGHTCONVERT_1600480088461_H