#include "blur.h"

namespace enchase
{
	void blur(MatrixF* matrix, int blurNum)
	{
		MatrixF f(3, 3, 1, 1.0f / 9.0f);
		MatrixF* matrix2 = new MatrixF(*matrix);

		int i = 0;
		for (i = 0; i + 1 < blurNum; i += 2)
		{
			matrix->conv(f, MatrixF::PAD_EDGE, *matrix2);
			matrix2->conv(f, MatrixF::PAD_EDGE, *matrix);
		}
		if (i < blurNum)
		{
			matrix->conv(f, MatrixF::PAD_EDGE, *matrix2);
			*matrix = *matrix2;
		}

		delete matrix2;
	}

    void blur(MatrixU* matrix, int blurNum)
    {
        MatrixF f(3, 3, 1, 1.0f / 9.0f);
        MatrixU* matrix2 = new MatrixU(*matrix);

        int i = 0;
        for (i = 0; i + 1 < blurNum; i += 2)
        {
            matrix->conv<float>(f, MatrixU::PAD_EDGE, *matrix2);
            matrix2->conv<float>(f, MatrixU::PAD_EDGE, *matrix);
        }
        if (i < blurNum)
        {
            matrix->conv<float>(f, MatrixU::PAD_EDGE, *matrix2);
            *matrix = *matrix2;
        }

        delete matrix2;
    }
}