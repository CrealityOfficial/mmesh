#include "heightconvert.h"
#include "imagematrix.h"
#include <vector>

namespace enchase
{
	void normalConvert(MatrixF* matrix, float base, float scale)
	{
		*matrix = scale * (*matrix);
		*matrix = base + (*matrix);
	}

	void transparencyConvert(MatrixF* matrix, float base, float scale, float transmittance_1mm)
	{
		double divisor = 1.0 / log(transmittance_1mm / 100.0);     // transmittance_1mm = 0.25 
		double min_luminance = pow((transmittance_1mm / 100.0), scale);// peak_height=13£¬base_height=3  

		// y = height = 0, 1, 2...118;  x = width = 0, 1, 2...937 
		for (int y = 0; y < matrix->height(); y++)
		{
			for (int x = 0; x < matrix->width(); x++)  //  x = width = 0, 1, 2...937 
			{
				double mapped_luminance = min_luminance + (1.0 - min_luminance) * (*matrix->ptr(y, x));
				*matrix->ptr(y, x) = base + divisor * log(mapped_luminance);
			}
		}
	}
}