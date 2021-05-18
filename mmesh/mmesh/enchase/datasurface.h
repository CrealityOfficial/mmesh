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
	protected:
		MatrixF* produce() override;
	protected:
		int m_width;
		int m_height;
		unsigned char* m_data;
	};
}

#endif // ENCHASE_DATASURFACE_1608003995163_H