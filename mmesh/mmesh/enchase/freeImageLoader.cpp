#include "imageloader.h"

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

#if HAVE_FREEIMAGE
	void convert(ImageData& data, FIBITMAP* dib)
	{
		const int nBitCounts = 8;
		int nBpp = FreeImage_GetBPP(dib);
		BYTE* imgData = FreeImage_GetBits(dib);
		int width = FreeImage_GetWidth(dib);
		int height = FreeImage_GetHeight(dib);

		int rowSize = (((width * nBpp) + 31) >> 5) << 2;
		if (imgData && width > 0 && height > 0)
		{
			data.width = width;
			data.height = height;
			data.data = new unsigned char[width * height];

			BYTE nIntensity;
			int j, k;
			switch (nBpp)
			{
			case 32: //32λͼ����alphaͨ����ת��Ϊ8λ�Ҷ�ͼ
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
			case 24: //24λͼ��תΪ8λ�Ҷ�ͼ
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
			case 8: //8λα��ɫתΪ8λ�Ҷ�ͼ
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

#endif

	void loadImage_freeImage(ImageData& data, const std::string& fileName)
	{
		data.data = nullptr;
		data.width = 0;
		data.height = 0;

#if HAVE_FREEIMAGE
		FreeImage_SetOutputMessage(FreeImageErrorHandler);

		FIBITMAP* dib = GenericLoader(fileName.c_str(), 0);
		if (dib != NULL) {

			convert(data, dib);
			// free the dib
			FreeImage_Unload(dib);
		}
#endif
	}

	void loadImage_freeImage(ImageData& data, const std::string& extension, int fd)
	{
		data.data = nullptr;
		data.width = 0;
		data.height = 0;

#if HAVE_FREEIMAGE
		FreeImage_SetOutputMessage(FreeImageErrorHandler);

		FIBITMAP* dib = GenericLoader(FIF_JPEG, 0, fd);
		if (dib != NULL) {

			convert(data, dib);
			// free the dib
			FreeImage_Unload(dib);
		}
#endif
	}
}