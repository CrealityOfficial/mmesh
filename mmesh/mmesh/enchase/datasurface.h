#ifndef ENCHASE_DATASURFACE_1608003995163_H
#define ENCHASE_DATASURFACE_1608003995163_H
#include "surface.h"
namespace enchase
{
	class DataSurface : public Surface
	{
	public:
		DataSurface();
		virtual ~DataSurface();

		void setData(unsigned char* data, int width, int height);
        
        void original(MatrixU* matrix);
        void oilPainting(MatrixU* matrix);
        void sketch(MatrixU* matrix);
        void sketch2(MatrixU* matrix, MatrixU* sketchArray0, MatrixU* sketchArray1);
	protected:
		MatrixF* produce() override;
        void stepData(MatrixU* matrix, int steps, bool smooth);
        
	protected:
		int m_width;
		int m_height;
		unsigned char* m_data;
	};
}

#endif // ENCHASE_DATASURFACE_1608003995163_H
