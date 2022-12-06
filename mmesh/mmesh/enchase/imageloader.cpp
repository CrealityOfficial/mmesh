#include "imageloader.h"
#include <string.h>
#include <float.h>
#include "optimizeenchaser.h"
#include "trimesh2/Vec.h"
#include "trimesh2/XForm.h"

#include <functional>
#include <limits.h>
#define FLT_MIN 1.175494351e-38F 
#define FLT_MAX 3.402823466e+38F
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

	ImageData::ImageData(const ImageData& imageData)
		:data(nullptr)
		, width(0)
		, height(0)
	{
		if (imageData.data && imageData.width && imageData.height)
		{
			allocate(imageData.width, imageData.height);
			memcpy(data, imageData.data, imageData.width * imageData.height);
		}
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
				memset(data, 0, w * h);
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
//			for(int i = 0; i < width; ++i)
//				*(d + i) = v;
            memset(d, v, width);
		}
	}

	bool ImageData::valid()
	{
		return width > 0 && height > 0 && data;
	}

	void ImageData::fromFlipY(const ImageData& src)
	{
		allocate(src.width, src.height);
		for (int i = 0; i < src.height; ++i)
		{
			unsigned char* cs = src.data + i * src.width;
			unsigned char* cd = data + (src.height - 1 - i) * src.width;
			memcpy(cd, cs, src.width);
		}
	}

	void ImageData::rotate(double radians, int channels, bool releaseFlag, int maxWidth, int maxHeight)
	{
		typedef std::pair<float, float> vec2f;
		std::function<vec2f(vec2f, float)> rotateFunc = [](vec2f origin, float rotRadians)->vec2f
		{
			return vec2f(origin.first * cosf(rotRadians) - origin.second * sinf(rotRadians), origin.first * sinf(rotRadians) + origin.second * cosf(rotRadians));
		};

		double halfWidth = width / 2.0;
		double halfHeight = height / 2.0;
		std::vector<vec2f> edgePos = { vec2f(-halfWidth , halfHeight), vec2f(halfWidth, halfHeight), vec2f(-halfWidth, -halfHeight), vec2f(halfWidth, -halfHeight) };
		float maxX = -FLT_MAX, minX = FLT_MAX, maxY = -FLT_MAX, minY = FLT_MAX;
		for (int i = 0; i < edgePos.size(); i++)
		{
			vec2f newPos = rotateFunc(edgePos[i], radians);
			if (newPos.first < minX)
				minX = newPos.first;
			if (newPos.first > maxX)
				maxX = newPos.first;

			if (newPos.second < minY)
				minY = newPos.second;
			if (newPos.second > maxY)
				maxY = newPos.second;
		}

		float centerX = (maxX - minX) / 2.0;
		float centerY = (maxY - minY) / 2.0;
		int rotateWidth = maxX - minX;
		int newWidth = rotateWidth > maxWidth ? maxWidth : rotateWidth;
		int rotateHeight = maxY - minY;
		int newHeight = rotateHeight > maxHeight ? maxHeight : rotateHeight;
		unsigned char* newData = new unsigned char[newWidth * newHeight * channels];
		memset(newData, 0, newWidth * newHeight * channels);

		float iOffset = (rotateWidth - newWidth) / 2.0;
		float jOffset = (rotateHeight - newHeight) / 2.0;

		enchase::Texture texRaw(width, height, data);
		for (int i = 0; i < newWidth; ++i)
		{
			for (int j = 0; j < newHeight; ++j)
			{
				float realI = i + iOffset;
				float realJ = j + jOffset;
				vec2f originPos = rotateFunc(vec2f(realI - centerX, centerY - realJ), -radians);

				float x = (originPos.first + halfWidth) / width;
				float y = (halfHeight - originPos.second) / height;

				int indexNew = j * newWidth * channels + i * channels;
				unsigned char gray = 0;

				if (x >= 0 && x < 1.0 && y >= 0 && y < 1.0)
				{
					unsigned char _gray = 0;
					if (texRaw.texcoordGet(x, y, _gray))
						gray = _gray;
				}

				for (int k = 0; k < channels; k++)
				{
					newData[indexNew] = gray;
					indexNew++;
				}
			}
		}

		if (releaseFlag)
			release();

		data = newData;
		width = newWidth;
		height = newHeight;
		rotRadians = radians;
	}

	void ImageData::clone(int w, int h, unsigned char* srcdata)
	{
		allocate(w, h);
		memcpy(data, srcdata, w*h);
	}

	void ImageData::cloneFrom(const ImageData& src)
	{
		if (src.width > 0 && src.height > 0)
		{
			allocate(src.width, src.height);
			memcpy(data, src.data, src.width * src.height);
		}
	}

	void ImageData::extendChannels(int targetChannels)
	{
		int newWidth = width;
		int newHeight = height;
		unsigned char* newData = new unsigned char[newWidth * newHeight * targetChannels];
		memset(newData, 0, newWidth * newHeight * targetChannels);

		for (int i = 0; i < newWidth; ++i)
		{
			for (int j = 0; j < newHeight; ++j)
			{
				int indexNew = j * newWidth * targetChannels + i * targetChannels;
				unsigned char gray = data[j * width + i];

				for (int k = 0; k < targetChannels; k++)
				{
					newData[indexNew] = gray;
					indexNew++;
				}
			}
		}

		release();

		data = newData;
		width = newWidth;
		height = newHeight;
	}

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

	void fillImageData(ImageData& raw, ImageData* alpha, int width, int height, unsigned char* data, int type, bool flipY)
	{
		raw.allocate(width, height);
		if(alpha)
			alpha->allocate(width, height);

		int pixel = (type == 3 || type == 4) ? 2 : 4;

		if (type == 0 || type == 6)
			pixel = 1;

		typedef std::function<void(unsigned char*, unsigned char&, unsigned char&)> pixelFunc;
		pixelFunc fpixel3 = [](unsigned char* data, unsigned char& gray, unsigned char& alpha) {
			unsigned short* sdata = (unsigned short*)data;
			unsigned char R = (*sdata) & 0x1f;
			unsigned char G = (*sdata >> 5) & 0x3f;
			unsigned char B = (*sdata >> 11) & 0x1f;
			gray = (unsigned char)(((float)R + (float)G + (float)B) / 3.0f);
			alpha = 255;
		};
		pixelFunc fpixel4 = [](unsigned char* data, unsigned char& gray, unsigned char& alpha) {
			unsigned short* sdata = (unsigned short*)data;
			unsigned char A = (*sdata) & 0xf;
			unsigned char R = (*sdata >> 4) & 0xf;
			unsigned char G = (*sdata >> 8) & 0xf;
			unsigned char B = (*sdata >> 12) & 0xf;
			gray = (unsigned char)(((float)R + (float)G + (float)B) / 3.0f);
			alpha = A;
		};
		pixelFunc fpixel5 = [](unsigned char* pdata, unsigned char& gray, unsigned char& alpha) {
			float v = 0.0f;
			for (int k = 0; k < 3; ++k)
				v += (float) * (pdata + k);
			gray = (unsigned char)(v / 3.0f);

			alpha = *(pdata + 3);
		};

		pixelFunc fpixel0 = [](unsigned char* pdata, unsigned char& gray, unsigned char& alpha) {
			gray = *pdata;
			alpha = 255;
		};

		pixelFunc fpixel = (type == 3) ? fpixel3 : ((type == 4) ? fpixel4 : ((type == 0) ? fpixel0 : fpixel5));
		for (int i = 0; i < width; ++i)
		{
			for (int j = 0; j < height; ++j)
			{
				int index = j * width + i;
				int rindex = (height - 1 - j) * width + i;
				if(!flipY)
					rindex = index;
				unsigned char* pdata = data + rindex * pixel;
				unsigned char a = 0;
				fpixel(pdata, raw.data[index], a);

				if (alpha)
					alpha->data[index] = a;
			}
		}
	}
}
