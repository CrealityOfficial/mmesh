#include "datasurface.h"

namespace enchase
{
	DataSurface::DataSurface()
		:m_width(0)
		, m_height(0)
		, m_data(nullptr)
	{

	}

	DataSurface::~DataSurface()
	{
		if (m_data)
		{
			delete [] m_data;
			m_data = nullptr;
		}
	}

	void DataSurface::setData(unsigned char* data, int width, int height)
	{
		m_width = width;
		m_height = height;

		if (m_width > 0 && m_height > 0)
		{
			m_data = new unsigned char[m_width * m_height];
			memcpy(m_data, data, m_width * m_height * sizeof(char));
		}
	}

	MatrixF* DataSurface::produce()
	{
		uint width = (uint)m_width;
		uint height = (uint)m_height;
		if (width == 0 || height == 0)
		{
			return nullptr;
		}

		MatrixF* imgMtx = new MatrixF(width, height, 1);
		float* imgData = imgMtx->ptr(0);

		for (int h = 0; h < height; h++)
		{
			for (int w = 0; w < width; w++)
			{
				unsigned char c = m_data[h * m_width + w];
				imgData[w] = (float)c / 255.0f;
			}
			imgData = imgMtx->ptr(h);
		}
		return imgMtx;
	}
}