#ifndef MMESH_PHOTO3D_1607995563690_H
#define MMESH_PHOTO3D_1607995563690_H
#include <string>
#include "imagematrix.h"

namespace trimesh
{
	class TriMesh;
}

namespace enchase
{
	class Surface;
	class Photo3DParam
	{
	public:
		Photo3DParam();
		~Photo3DParam();

		float baseThickness;
		float maxThickness;

		bool useBlur;
		int blurTimes;

		bool invert;
		int useIndex;
		int maxPixel; //���߷ֱ���

		float realWidth;  //ʵ��������Ƭģ�͵ĳߴ磬ͬʱ�ᱣ������ͼƬ�Ŀ�߱�
	};

	class Photo3D
	{
	public:
		Photo3D();
		~Photo3D();

		void setLogger(const std::string& logger);
		void setSource(const std::string& imageName);   //����֧��bmp, png, jpg
		void setSource(const std::string& extension, int fd);
		void setSource(unsigned char* data, int width, int height);   //�Ҷ�ֵ
        void setSerialNumber(unsigned char* data, int width, int height);
        void setSerialNumber(enchase::MatrixF *serial);
        
		bool generate(const std::string& stlFile, const Photo3DParam& param, int& errorCode);
		trimesh::TriMesh* generate(const Photo3DParam& param, int& errorCode);
	protected:
		enchase::Surface* m_surface;
        enchase::MatrixF* m_serialNumber;
	};
}

#endif // CXSS_PHOTO3D_1607995563690_H
