#ifndef ENCHASE_SURFACE_1601262551755_H
#define ENCHASE_SURFACE_1601262551755_H
#include <memory>
#include <string>
#include "mmesh/enchase/imagematrix.h"
#include "mmesh/enchase/enum.h"

namespace enchase
{
	class Surface
	{
	public:
		Surface();
		virtual ~Surface();

		MatrixF* matrix();
		virtual MatrixF* produce() = 0;
		void setIndex(int index);
		int index();
	protected:
		void process(MatrixF* matrix);
	public:
		int convertType;
		float baseHeight;
		float maxHeight;

		bool useBlur;
		int blurTimes;
		bool invert;
		EdgeType edgeType;

		int useIndex;
	};

	typedef std::shared_ptr<Surface> SurfacePtr;
}

#endif // ENCHASE_SURFACE_1601262551755_H