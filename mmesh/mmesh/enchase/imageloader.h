#ifndef ENCHASE_IMAGELOADER_1608004787715_H
#define ENCHASE_IMAGELOADER_1608004787715_H
#include <string>

namespace enchase
{
	class ImageData
	{
	public:
		unsigned char* data;
		int width;
		int height;

		ImageData();
		ImageData(const ImageData& data);
		~ImageData();
		
		void release();
		void allocate(int w, int h);
		void resize(int w, int h);
		void gradient(unsigned char start, unsigned char end);
		bool valid();

		inline void blend(int x, int y, unsigned char value, unsigned char alpha)
		{
			if (alpha == 0 || x < 0 || x >= width || y < 0 || y >= height)
				return;

			unsigned char* dest = data + y * width + x;
			if (alpha == 255)
			{
				*dest = value;
				return;
			}

			unsigned char d = *dest;
			if (d == 255)
				return;

			unsigned char v = (unsigned char)((float)alpha * (float)value / 255.0f);
			if (255 - d < v)
				* dest = 255;
			else
				*dest = d + v;
		}
	};

	class GrayImage
	{
	public:
		GrayImage(){}
		~GrayImage() {}

		ImageData raw;
		ImageData alpha;
	};

	//type = 3 R5G6B5 , type = 4 R4G4B4A4 , type = 5 R8G8B8A8
	void fillImageData(ImageData& raw, ImageData* alpha, int width, int height, unsigned char* data, int type = 5, bool flipY = true);

	void loadImage(ImageData& data, const std::string& fileName);
	void loadImage_freeImage(ImageData& data, const std::string& fileName);
	void loadImage_freeImage(ImageData& data, const std::string& extension, int fd);
	void writeImage_freeImage(unsigned char* data, int width, int height, const std::string& fileName);  //rgba
}

#endif // ENCHASE_IMAGELOADER_1608004787715_H