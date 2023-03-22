#include "imageloader.h"
#include "FreeImage.h"
#include "Utilities.h"

#if HAVE_FREEIMAGE
#include "FreeImage.h"

/** Generic image loader
	@param lpszPathName Pointer to the full file name
	@param flag Optional load flag constant
	@return Returns the loaded dib if successful, returns NULL otherwise
*/
FIBITMAP* GenericLoader(const char* lpszPathName, int flag) {
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;

	// check the file signature and deduce its format
	// (the second argument is currently not used by FreeImage)
	fif = FreeImage_GetFileType(lpszPathName, 0);
	if (fif == FIF_UNKNOWN) {
		// no signature ?
		// try to guess the file format from the file extension
		fif = FreeImage_GetFIFFromFilename(lpszPathName);
	}
	// check that the plugin has reading capabilities ...
	if ((fif != FIF_UNKNOWN) && FreeImage_FIFSupportsReading(fif)) {
		// ok, let's load the file
		FIBITMAP* dib = FreeImage_Load(fif, lpszPathName, flag);
		// unless a bad file format, we are done !
		return dib;
	}
	return NULL;
}

FIBITMAP* GenericLoader(FREE_IMAGE_FORMAT fif, int flag, int fd) 
{
	// check that the plugin has reading capabilities ...
	if ((fif != FIF_UNKNOWN) && FreeImage_FIFSupportsReading(fif)) {
		// ok, let's load the file
		FIBITMAP* dib = FreeImage_LoadFD(fif, fd, flag);
		// unless a bad file format, we are done !
		return dib;
	}
	return NULL;
}

/** Generic image writer
	@param dib Pointer to the dib to be saved
	@param lpszPathName Pointer to the full file name
	@param flag Optional save flag constant
	@return Returns true if successful, returns false otherwise
*/
bool GenericWriter(FIBITMAP* dib, const char* lpszPathName, int flag) {
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	BOOL bSuccess = FALSE;

	if (dib) {
		// try to guess the file format from the file extension
		fif = FreeImage_GetFIFFromFilename(lpszPathName);
		if (fif != FIF_UNKNOWN) {
			// check that the plugin has sufficient writing and export capabilities ...
			WORD bpp = FreeImage_GetBPP(dib);
			if (FreeImage_FIFSupportsWriting(fif) && FreeImage_FIFSupportsExportBPP(fif, bpp)) {
				// ok, we can save the file
				bSuccess = FreeImage_Save(fif, dib, lpszPathName, flag);
				// unless an abnormal bug, we are done !
			}
		}
	}
	return (bSuccess == TRUE) ? true : false;
}

// ----------------------------------------------------------
LogCallback* tCallback = nullptr;
/**
	FreeImage error handler
	@param fif Format / Plugin responsible for the error
	@param message Error message
*/
void FreeImageErrorHandler(FREE_IMAGE_FORMAT fif, const char* message) {
	printf("\n*** ");
	if (fif != FIF_UNKNOWN) {
		printf("%s Format\n", FreeImage_GetFormatFromFIF(fif));
	}
	printf("%s ", message);
	printf(" ***\n");
}

#endif 

namespace enchase
{
    void convert(ImageData& data, FIBITMAP* dib, LogCallback* callback)
    {
        const int nBitCounts = 8;
        int nBpp = FreeImage_GetBPP(dib);
        BYTE* imgData = FreeImage_GetBits(dib);
        int width = FreeImage_GetWidth(dib);
        int height = FreeImage_GetHeight(dib);

        //if (callback)
        //{
        //	callback->log("width height :%d x %d, nBpp : %d ", width, height, nBpp);
        //}
        //CXLogInfo("width height :%d x %d, nBpp : %d ", width, height, nBpp);

        int rowSize = (((width * nBpp) + 31) >> 5) << 2;
        if (imgData && width > 0 && height > 0)
        {
            data.width = width;
            data.height = height;
            data.data = new unsigned char[width * height];

            BYTE nIntensity;
            float fIntensity;
            int j, k;
            bool little = FreeImage_IsLittleEndian();
            switch (nBpp)
            {
            case 64: //64位图（带alpha通道）转换为8位灰度图
                for (j = 0; j < height; j++)
                {
                    for (k = 0; k < width; k++)
                    {
                        if (!little)
                        {
                            fIntensity = (float)(0.33333 * (256 * imgData[j * rowSize + 8 * k] + imgData[j * rowSize + 8 * k + 1]) +
                                0.33333 * (256 * imgData[j * rowSize + 8 * k + 2] + imgData[j * rowSize + 8 * k + 3]) +
                                0.33333 * (256 * imgData[j * rowSize + 8 * k + 4] + imgData[j * rowSize + 8 * k + 5])) * 0.00390f;
                        }
                        else
                        {
                            fIntensity = (float)(0.33333 * (256 * imgData[j * rowSize + 8 * k + 1] + imgData[j * rowSize + 8 * k]) +
                                0.33333 * (256 * imgData[j * rowSize + 8 * k + 3] + imgData[j * rowSize + 8 * k + 2]) +
                                0.33333 * (256 * imgData[j * rowSize + 8 * k + 5] + imgData[j * rowSize + 8 * k + 4])) * 0.00390f;
                        }


                        *(data.data + j * width + k) = (BYTE)fIntensity;
                    }
                }
                break;
            case 32: //32位图（带alpha通道）转换为8位灰度图
                for (j = 0; j < height; j++)
                {
                    for (k = 0; k < width; k++)
                    {
                        nIntensity = (BYTE)(0.33333 * imgData[j * rowSize + 4 * k] +
                            0.33333 * imgData[j * rowSize + 4 * k + 1] + 0.33333 * imgData[j * rowSize + 4 * k + 2]);

                        *(data.data + j * width + k) = nIntensity;
                    }
                }
                break;
            case 24: //24位图像转为8位灰度图
                for (j = 0; j < height; j++)
                {
                    for (k = 0; k < width; k++)
                    {
                        nIntensity = (BYTE)(0.33333 * imgData[j * rowSize + 3 * k] +
                            0.33333 * imgData[j * rowSize + 3 * k + 1] + 0.33333 * imgData[j * rowSize + 3 * k + 2]);

                        *(data.data + j * width + k) = nIntensity;
                    }
                }
                break;
            case 8: //8位伪彩色转为8位灰度图
                for (j = 0; j < height; j++)
                {
                    for (k = 0; k < width; k++)
                    {
                        FreeImage_GetPixelIndex(dib, k, j, &nIntensity);
                        RGBQUAD* ptrRGB = FreeImage_GetPalette(dib);
                        nIntensity = (BYTE)(0.299 * ptrRGB[nIntensity].rgbRed +
                            0.587 * ptrRGB[nIntensity].rgbGreen + 0.114 * ptrRGB[nIntensity].rgbBlue);

                        *(data.data + j * width + k) = nIntensity;
                    }
                }
            }
        }
    }
#if HAVE_FREEIMAGE
	void convert2Gray(ImageData& data, FIBITMAP* dib)
	{
		const int nBitCounts = 8;
		int nBpp = FreeImage_GetBPP(dib);
		BYTE* imgData = FreeImage_GetBits(dib);
		int width = FreeImage_GetWidth(dib);
		int height = FreeImage_GetHeight(dib);

		int rowSize = (((width * nBpp) + 31) >> 5) << 2;
		if (imgData && width > 0 && height > 0)
		{
			data.allocate(width, height);

			BYTE nIntensity;
			int j, k;
			switch (nBpp)
			{
			case 32: //32位图锟斤拷锟斤拷alpha通锟斤拷锟斤拷转锟斤拷为8位锟揭讹拷图
				for (j = 0; j < height; j++)
				{
					for (k = 0; k < width; k++)
					{
						nIntensity = (BYTE)(0.33333 * imgData[j * rowSize + 4 * k] +
							0.33333 * imgData[j * rowSize +4 * k + 1] + 0.33333 * imgData[j * rowSize + 4 * k + 2]);
						
						*(data.data + j * width + k) = nIntensity;
					}
				}
				break;
			case 24: //24位图锟斤拷转为8位锟揭讹拷图
				for (j = 0; j < height; j++)
				{
					for (k = 0; k < width; k++)
					{
						nIntensity = (BYTE)(0.33333 * imgData[j * rowSize + 3 * k] +
							0.33333 * imgData[j * rowSize + 3 * k + 1] + 0.33333 * imgData[j * rowSize + 3 * k + 2]);

						*(data.data + j * width + k) = nIntensity;
					}
				}
				break;
			case 8: //8位伪锟斤拷色转为8位锟揭讹拷图
				for (j = 0; j < height; j++)
				{
					for (k = 0; k < width; k++)
					{
						FreeImage_GetPixelIndex(dib, k, j, &nIntensity);
						RGBQUAD* ptrRGB = FreeImage_GetPalette(dib);
						nIntensity = (BYTE)(0.299 * ptrRGB[nIntensity].rgbRed +
							0.587 * ptrRGB[nIntensity].rgbGreen + 0.114 * ptrRGB[nIntensity].rgbBlue);

						*(data.data + j * width + k) = nIntensity;
					}
				}
			}
		}
	}
	bool convertBaseFormat(ImageData& data, FIBITMAP* dib)
	{
		bool ret = true;
		switch (data.format)
		{
		case ImageDataFormat::FORMAT_RGBA_8888:
		{
			//! 获取数据指针
			FIBITMAP* newdib = FreeImage_ConvertTo32Bits(dib);

			BYTE* pixels = (BYTE*)FreeImage_GetBits(newdib);
			int     width = FreeImage_GetWidth(newdib);
			int     height = FreeImage_GetHeight(newdib);
			data.allocate(width * 4, height);
			data.format = ImageDataFormat::FORMAT_RGBA_8888;
			for (int i = 0; i < width * height * 4; i += 4)
			{
				data.data[i] = pixels[i + 0];//bgr->rgb
				data.data[i + 1] = pixels[i + 1];
				data.data[i + 2] = pixels[i+2];//bgr->rgb
				data.data[i + 3] = pixels[i + 3] > 0 ? pixels[i + 3] : 255;
			}
			FreeImage_Unload(newdib);
		}
		break;
        case ImageDataFormat::FORMAT_RGBA_888:
        {
            //! 获取数据指针
            FIBITMAP* newdib = FreeImage_ConvertTo24Bits(dib);

            BYTE* pixels = (BYTE*)FreeImage_GetBits(newdib);
            int     width = FreeImage_GetWidth(newdib);
            int     height = FreeImage_GetHeight(newdib);
            data.allocate(width * 3, height);
            data.format = ImageDataFormat::FORMAT_RGBA_888;
            for (int i = 0; i < width * height * 3; i += 3)
            {
                data.data[i] = pixels[i + 2];//bgr->rgb
                data.data[i + 1] = pixels[i + 1];
                data.data[i + 2] = pixels[i];//bgr->rgb
                //data.data[i + 3] = pixels[i + 3] > 0 ? pixels[i + 3] : 255;
            }
            FreeImage_Unload(newdib);
        }
        break;
		default:
		{
			convert2Gray(data, dib);
		}
		}
		return ret;

	}

#endif
	void freeImage_Deinitialise()
	{
#if HAVE_FREEIMAGE
        FreeImage_DeInitialise();
#endif
    }


	void freeImage_Initialise()
	{
#if HAVE_FREEIMAGE
		FreeImage_Initialise(FALSE);
#endif
	}
	void loadImage_freeImage(ImageData& data, const std::string& fileName)
	{
#if HAVE_FREEIMAGE
		FreeImage_SetOutputMessage(FreeImageErrorHandler);

		FIBITMAP* dib = GenericLoader(fileName.c_str(), 0);
		if (dib != NULL) {

			convertBaseFormat(data, dib);
			// free the dib
			FreeImage_Unload(dib);
		}else
			data.release();
#endif
	}

	void loadImage_freeImage(ImageData& data, const std::string& extension, int fd)
	{
#if HAVE_FREEIMAGE
		FreeImage_SetOutputMessage(FreeImageErrorHandler);

		FREE_IMAGE_FORMAT format = FIF_UNKNOWN;
		if (extension == "jpg" || extension == "jpeg")
			format = FIF_JPEG;
		else if (extension == "png")
			format = FIF_PNG;

		FIBITMAP* dib = GenericLoader(format, 0, fd);
		if (dib != NULL) {

			convertBaseFormat(data, dib);
			// free the dib
			FreeImage_Unload(dib);
		}else
			data.release();
#endif
	}

	void writeImage_freeImage(unsigned char* data, int width, int height, const std::string& fileName)
	{
#if HAVE_FREEIMAGE
		FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;

		// check the file signature and deduce its format
		// (the second argument is currently not used by FreeImage)
		fif = FreeImage_GetFileType(fileName.c_str(), 0);
		if (fif == FIF_UNKNOWN) {
			// no signature ?
			// try to guess the file format from the file extension
			fif = FreeImage_GetFIFFromFilename(fileName.c_str());
		}
		// check that the plugin has reading capabilities ...
		if ((fif != FIF_UNKNOWN) && FreeImage_FIFSupportsWriting(fif)) {
			FIBITMAP* dib = FreeImage_ConvertFromRawBits(data, width, height, 4 * width, 32, 0, 0, 0);;
			FreeImage_Save(fif, dib, fileName.c_str(), 0);
			FreeImage_Unload(dib);
		}
#endif
	}
	ImageData* constructNewFreeImage(std::vector<ImageData*> imagedata, ImageDataFormat format, std::vector<std::pair<ImageData::point, ImageData::point>> &offset)
	{
		enchase::ImageData* dataret = nullptr;
		#if HAVE_FREEIMAGE
		//int H = FreeImage_GetHeight(dib);

		//得到图像宽度int W = FreeImage_GetWidth(dib);

		//得到图像像素 BYTE* data = FreeImage_GetBits(dib);

		//得到图像位深 int bpp = FreeImage_GetBpp(dib);

		//得到x, y像素 RGBQUAD color; FreeImage_GetPixelColor(dib, x, y, &color);

		//写入x, y像素 FreeImage_SetPixelColor(dib, x, y, color);

		//开辟新图像 FIBITMAP* re = FreeImage_Allocate(W, H, bpp);

		//拷贝exif信息 FreeImage_CloneMetadata(dib, re);
		int widthMax = 0;
		int heightMax = 0;
		int widthTotal = 0;
		int heightTotal = 0;
		int widthoffset = 0;
		int heightoffset = 0;
		int bytesPerPixel = 4;//FORMAT_RGBA_8888
		int bpp = 32;
		switch (format)
		{
		case FORMAT_RGBA_8888:
			bytesPerPixel = 4;
			bpp = 32;
			break;
			//case FORMAT_RGB_565:
			//case FORMAT_RGBA_4444:
			//	bytesPerPixel = 2;
			//	break;
		default://other format is not test
		{
			printf("other format is not test");
			return nullptr;

		}
		}
		for (auto dataPtr: imagedata)
		{

			if (dataPtr == nullptr)
			{
				continue;//should be not empty
			}
			if (dataPtr->format != format)//channels must be same
			{
				return nullptr;
			}

			widthMax = widthMax > dataPtr->width?widthMax:dataPtr->width;
			heightMax = heightMax > dataPtr->height? heightMax :dataPtr->height;
			widthTotal += dataPtr->width/ bytesPerPixel;
			heightTotal += dataPtr->height;
		}
		widthMax = widthMax/ bytesPerPixel;
		heightMax = heightMax;
		FIBITMAP* dibptr =FreeImage_Allocate(widthMax, heightTotal, bpp);
		if (dibptr == nullptr)
		{
			return nullptr;
		}

		offset.resize(imagedata.size());
		int offsetIndex=0;
		for (int index=0;index<imagedata.size();index++)
		{
			auto &dataPtr = imagedata[index];
			if (dataPtr == nullptr)
			{
				widthoffset = 0;
				ImageData::point startpos = { widthoffset,heightoffset };
				ImageData::point endpos = { widthoffset ,heightoffset };
				offset[offsetIndex++] = std::make_pair(startpos, endpos);
				continue;
			}
			for (int indexW = 0; indexW < dataPtr->width/ bytesPerPixel; indexW++)
			{
				for (int indexH = 0; indexH < dataPtr->height; indexH++)
				{
					RGBQUAD rgb;
					rgb.rgbRed =   dataPtr->data[dataPtr->width * indexH + indexW * bytesPerPixel];
					rgb.rgbGreen = dataPtr->data[dataPtr->width * indexH + indexW * bytesPerPixel +1];
					rgb.rgbBlue =  dataPtr->data[dataPtr->width * indexH + indexW * bytesPerPixel +2];
					FreeImage_SetPixelColor(dibptr, widthoffset + indexW, heightoffset + indexH, &rgb);
				}

			}
			ImageData::point startpos = { widthoffset ,heightoffset };
			heightoffset += dataPtr->height;
			widthoffset = dataPtr->width / bytesPerPixel;
			ImageData::point endpos = { widthoffset ,heightoffset };
			offset[offsetIndex++] = std::make_pair(startpos, endpos);
			widthoffset = 0;
				
		}
		//FreeImage_Save(FREE_IMAGE_FORMAT::FIF_BMP, dibptr, "test.bmp", 0);
			if(dibptr)
			{
				dataret = new enchase::ImageData;
				dataret->format = format;
				convertBaseFormat(*dataret, dibptr);
				FreeImage_Unload(dibptr);

			}
			#endif
			return dataret;
	}
	ImageData* scaleFreeImage(ImageData* imagedata, float scaleX, float scaleY)
	{
		enchase::ImageData* dataret = nullptr;
		#ifdef HAVE_FREEIMAGE
		int bytesPerPixel = 4;//FORMAT_RGBA_8888
		int bpp = 32;
		switch (imagedata->format)
		{
		case FORMAT_RGBA_8888:
			bytesPerPixel = 4;
			bpp = 32;
			break;
			//case FORMAT_RGB_565:
			//case FORMAT_RGBA_4444:
			//	bytesPerPixel = 2;
			//	break;
        case FORMAT_RGBA_888:
            bytesPerPixel = 3;
            bpp = 24;
            break;
		default://other format is not test
		{
			printf("other format is not test");
			return nullptr;

		}
		}

		FIBITMAP* dibOrignal = FreeImage_Allocate(imagedata->width / bytesPerPixel, imagedata->height, bpp);
		for (int indexW = 0; indexW < imagedata->width / bytesPerPixel; indexW++)
		{
			for (int indexH = 0; indexH < imagedata->height; indexH++)
			{
				RGBQUAD rgb;
				rgb.rgbRed = imagedata->data[imagedata->width * indexH + indexW * bytesPerPixel];
				rgb.rgbGreen = imagedata->data[imagedata->width * indexH + indexW * bytesPerPixel + 1];
				rgb.rgbBlue = imagedata->data[imagedata->width * indexH + indexW * bytesPerPixel + 2];
				FreeImage_SetPixelColor(dibOrignal, indexW, indexH, &rgb);
			}

		}
		int newW = imagedata->width * scaleX / bytesPerPixel;
		int newH = imagedata->height * scaleY;
		FIBITMAP* newdib = FreeImage_Rescale(dibOrignal, newW, newH);
		FreeImage_Unload(dibOrignal);
		dataret = new enchase::ImageData;
		dataret->format = imagedata->format;
		convertBaseFormat(*dataret, newdib);
		FreeImage_Unload(newdib);
		#endif
		return dataret;

	}

    bool loadImage_freeImage565(unsigned char* data, const std::string& fileName, int width, int height)
    {
        if (!data || fileName.empty() || width <= 0 || height <= 0) return false;
        try {
            unsigned char* p = data;
            FIBITMAP* dib = GenericLoader(fileName.c_str(), 0);
            if (!dib)
            {
                //CXLogError("loadImage_freeImage565 GenericLoader failed maybe format is wrong!");
                return false;
            }
            FIBITMAP* resdib = FreeImage_Rescale(dib, width, height);
            if (!resdib)
            {
                ///CXLogError("loadImage_freeImage565 FreeImage_Rescale failed maybe format is wrong!");
                return false;
            }
            FIBITMAP* dib565 = FreeImage_ConvertTo16Bits565(resdib);
            if (!dib565)
            {
                //CXLogError("loadImage_freeImage565 FreeImage_ConvertTo16Bits565 failed maybe format is wrong!");
                return false;
            }
            int errLineNum = 0;
            for (int h = 0; h < height; ++h)
            {
                unsigned char* line = (unsigned char*)FreeImage_GetScanLine(dib565, height - h - 1);
                if (!line)
                {
                    errLineNum++;
                    continue;
                }
                memcpy(p, line, width * 2);
                p += width * 2;
            }
            if (errLineNum > 0)
            {
                //CXLogError("loadImage_freeImage565 failed! errLineNum = %d",errLineNum);
            }
            FreeImage_Unload(dib);
            FreeImage_Unload(resdib);
            FreeImage_Unload(dib565);
        }
        catch (...)
        {
            //CXLogCritical("loadImage_freeImage565 exception!");
            return false;
        }
        return true;
    }

    bool loadImage_freeImage888(unsigned char* data, const std::string& fileName, int width, int height)
    {
        if (!data) return false;
        try {
            unsigned char* p = data;
            FIBITMAP* dib = GenericLoader(fileName.c_str(), 0);
            FIBITMAP* resdib = FreeImage_Rescale(dib, width, height);
            FIBITMAP* dib24 = FreeImage_ConvertTo24Bits(resdib);

            for (int h = 0; h < height; ++h)
            {
                unsigned char* line = (unsigned char*)FreeImage_GetScanLine(dib24, height - h - 1);
                memcpy(p, line, width * 3);
                p += width * 3;
            }

            //FIBITMAP* bitmap = FreeImage_ConvertFromRawBits(data, width, height, width * 3,  24,  0, 0, 0, FALSE);

            //FreeImage_Save(FIF_PNG, bitmap, "d:/sss1.png");

            FreeImage_Unload(dib);
            FreeImage_Unload(resdib);
            FreeImage_Unload(dib24);
        }
        catch (...)
        {
            return false;
        }
        return true;
    }

    void loadImage_freeImage(ImageData & data, const std::string & fileName, LogCallback * callback)
    {
        data.data = nullptr;
        data.width = 0;
        data.height = 0;

        FreeImage_SetOutputMessage(FreeImageErrorHandler);

        FIBITMAP* dib = GenericLoader(fileName.c_str(), 0);
        if (dib != NULL) {

            convert(data, dib, callback);

            //if (callback)
            //{
            //	callback->log("converted--> width height :%d x %d", data.width, data.height);
            //}

            // free the dib
            ///CXLogInfo("converted--> width height :%d x %d", data.width, data.height);
            FreeImage_Unload(dib);
        }
    }

    void convert8888to565(unsigned char* source, unsigned char* dest, int width, int height)
    {
        if (!source || !dest)
            return;

        for (int rows = 0; rows < height; rows++) {
            unsigned short* new_bits = (unsigned short*)(dest + rows * width * 2);
            for (int cols = 0; cols < width; cols++) {
                new_bits[cols] = RGB565(source[FI_RGBA_BLUE], source[FI_RGBA_GREEN], source[FI_RGBA_RED]);
                source += 4;
            }
        }
        return;
    }
}
