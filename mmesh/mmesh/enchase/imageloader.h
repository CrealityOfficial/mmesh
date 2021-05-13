#ifndef ENCHASE_IMAGELOADER_1608004787715_H
#define ENCHASE_IMAGELOADER_1608004787715_H
#include <string>

class LogCallback;
namespace enchase
{
	struct ImageData
	{
		unsigned char* data;
		int width;
		int height;
	};

	void loadImage(ImageData& data, const std::string& fileName);
	void loadImage_freeImage(ImageData& data, const std::string& fileName, LogCallback* callback);
}

#endif // ENCHASE_IMAGELOADER_1608004787715_H