#ifndef CXSS_PHOTO3D_1607995563690_H
#define CXSS_PHOTO3D_1607995563690_H
#include <string>

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

		bool generate(const std::string& stlFile, const Photo3DParam& param, std::string* error);
		trimesh::TriMesh* generate(const Photo3DParam& param, std::string* error);
	protected:
		enchase::Surface* m_surface;
	};
}

#endif // CXSS_PHOTO3D_1607995563690_H