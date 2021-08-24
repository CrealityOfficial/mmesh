#include "imageloader.h"
#include <string.h>
#include "optimizeenchaser.h"
#include "trimesh2/Vec.h"
#include "trimesh2/XForm.h"

namespace enchase
{
	ImageData::ImageData()
		:data(nullptr)
			, width(0)
			, height(0)
	{

	}

	ImageData::~ImageData()
	{
		release();
	}

	void ImageData::release()
	{
		if (data)
		{
			delete [] data;
			data = nullptr;
		}

		width = 0;
		height = 0;
	}

	void ImageData::allocate(int w, int h)
	{
		if (w > 0 && h > 0)
		{
			if (w * h > width * height)
			{
				release();
				data = new unsigned char[w * h];
				memset(data, 0, width * height);
			}
			width = w;
			height = h;
		}
	}

	void ImageData::resize(int w, int h)
	{
		if(!valid() || (width == w && height == h))
			return;

		unsigned char* newData = new unsigned char[w * h];

		float matrix[9] ={
				(float)width/(float)w, 0.0f, 0.0f,
				0.0f, (float)height/(float)h, 0.0f,
				0.0f, 0.0f, 1.0f
		};
		auto _transform = [](float* matrix, int& x, int& y){
			float _x = (float)x;
			float _y = (float)y;

			x = (int)(matrix[0] * _x + matrix[1] * _y + matrix[2]);
			y = (int)(matrix[3] * _x + matrix[4] * _y + matrix[5]);
		};


		enchase::Texture texRaw(width, height, data);
		for (int i = 0; i < w; ++i)
		{
			for (int j = 0; j < h; ++j)
			{
				int x = i;
				int y = j;
				_transform(matrix, x, y);

				int indexNew = j * w + i;
				unsigned char gray = 0;

				if (x >= 0 && x < width && y >= 0 && y < height)
				{
					unsigned char _gray = 0;
					if (texRaw.texcoordGet(x, y, _gray))
						gray = _gray;
				}

				newData[indexNew] = gray;
			}
		}

		std::swap(newData, data);
		width = w;
		height = h;

		delete [] newData;
	}

	void ImageData::gradient(unsigned char start, unsigned char end)
	{
		for(int j = 0; j < height; ++j)
		{
			unsigned char* d = data + j * width;
			float r = (float)j / (float)(height);
			unsigned char v = (unsigned char)((1.0f - r) * (float)start + r * (float)end);
			for(int i = 0; i < width; ++i)
				*(d + i) = v;
		}
	}

	bool ImageData::valid()
	{
		return width > 0 && height > 0 && data;
	}

	typedef struct BitmapFileHeader
	{
		unsigned short bfType;
		unsigned long bfSize;
		unsigned short bfReserved1;
		unsigned short bfReserved2;
		unsigned long bfOffBits;
	} BITMAPFILE;

	struct BitmapInfoHeader
	{
		unsigned long biSize;
		long biWidth;
		long biHeight;
		unsigned short biPlanes;
		unsigned short biBitCount;
		unsigned long biCompression;
		unsigned long biSizeImage;
		long biXPixPerMeter;
		long biYPixPerMeter;
		unsigned long biClrUsed;
		unsigned long biClrImporant;
	};

	struct RGBQUAD 
	{
		unsigned char rgbBlue;
		unsigned char rgbGreen;
		unsigned char rgbRed;
		unsigned char rgbReserved;
	};

	void loadBMP(ImageData& data, const std::string& fileName)
	{
		FILE* fp = fopen(fileName.c_str(), "rb");
		if (fp == 0)
			return;

		fseek(fp, 14, 0);

		//BITMAPINFOHEADER head;
		BitmapInfoHeader head;
		fread(&head, sizeof(BitmapInfoHeader), 1, fp);

		int W = head.biWidth;
		int H = std::abs(head.biHeight);
		data.allocate(W, H);

		bool flip = head.biHeight < 0;

		unsigned short biBitCount = head.biBitCount;

		int lineByte = (W * biBitCount / 8 + 3) / 4 * 4;
		int depth = biBitCount / 8;
		if (biBitCount == 8)
		{
			RGBQUAD pColorTable[256];
			fread(&pColorTable[0], sizeof(RGBQUAD), 256, fp);
		}

		unsigned char* pBmpBuf = new unsigned char[lineByte * H];
		fread(pBmpBuf, 1, lineByte * H, fp);
		if (W > 0 && H > 0)
		{
			for (int i = 0; i < W; ++i)
			{
				for (int j = 0; j < H; ++j)
				{
					unsigned char* s = pBmpBuf + j * lineByte + i * depth;
					unsigned char* d = data.data + (flip ? j : (H - 1 - j)) * W + i;

					if (biBitCount == 24)
					{
						int r = *s;
						int g = *(s + 1);
						int b = *(s + 2);

						*d = (unsigned char)((11 * r + 16 * g + 5 * b) / 32);
					}
					else
					{
						*d = *s;
					}
				}
			}
		}

		delete[]pBmpBuf;
		fclose(fp);
	}

	void loadImage(ImageData& data, const std::string& fileName)
	{
		size_t pos = fileName.find_last_of(".");
		if (pos != std::string::npos)
		{
			std::string ext = std::string(fileName.begin() + pos + 1, fileName.end());
			if (!strcmp(ext.c_str(), "bmp"))
			{
				loadBMP(data, fileName);
			}
		}
	}
}