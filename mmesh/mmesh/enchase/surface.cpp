#include "surface.h"
#include "blur.h"
#include "heightconvert.h"
#include "edgeprocessor.h"

namespace enchase
{
	Surface::Surface()
		: useBlur(false)
		, blurTimes(9)
		, convertType(0)
		, baseHeight(0.0f)
		, maxHeight(2.2f)
		, invert(true)
		, useIndex(0)
		, edgeType(EdgeType::et_none)
	{

	}

	Surface::~Surface()
	{

	}

	void Surface::process(MatrixF* matrix)
	{
		edgeProcess(matrix, edgeType);

		if (invert)
		{
			printf("invert\n");
			*matrix = 1 - *matrix;
		}

		if (useBlur)
		{
			blur(matrix, blurTimes);
		}

		if (convertType == 1)
		{
			transparencyConvert(matrix, baseHeight, maxHeight, 25);
		}
		else
		{
			normalConvert(matrix, baseHeight, maxHeight);
		}
	}

	MatrixF* Surface::matrix()
	{
		MatrixF* m = produce();
		if(m) process(m);
		return m;
	}

	void Surface::setIndex(int index)
	{
		useIndex = index;
	}

	int Surface::index()
	{
		return useIndex;
	}
}