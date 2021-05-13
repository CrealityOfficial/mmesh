#include "imageloader.h"
#include <string.h>

namespace enchase
{
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

		data.width = head.biWidth;
		data.height = std::abs(head.biHeight);
		bool flip = head.biHeight < 0;

		unsigned short biBitCount = head.biBitCount;

		int lineByte = (data.width * biBitCount / 8 + 3) / 4 * 4;
		int depth = biBitCount / 8;
		if (biBitCount == 8)
		{
			RGBQUAD pColorTable[256];
			fread(&pColorTable[0], sizeof(RGBQUAD), 256, fp);
		}

		unsigned char* pBmpBuf = new unsigned char[lineByte * data.height];
		fread(pBmpBuf, 1, lineByte * data.height, fp);
		if (data.width > 0 && data.height > 0)
		{
			data.data = new unsigned char[data.width * data.height];
			for (int i = 0; i < data.width; ++i)
			{
				for (int j = 0; j < data.height; ++j)
				{
					unsigned char* s = pBmpBuf + j * lineByte + i * depth;
					unsigned char* d = data.data + (flip ? j : (data.height - 1 - j)) * data.width + i;

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
		data.width = 0;
		data.height = 0;
		data.data = nullptr;

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