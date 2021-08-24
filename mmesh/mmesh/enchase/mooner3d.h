#ifndef ENCHASE_MOONER3D_1621324751120_H
#define ENCHASE_MOONER3D_1621324751120_H
#include <string>

namespace trimesh
{
	class TriMesh;
}

namespace enchase
{
	class Surface;
	class Moon3DParam
	{
	public:
		Moon3DParam();
		~Moon3DParam();

		float baseThickness;
		float maxThickness;

		bool useBlur;
		int blurTimes;

		bool invert;

		float radius;
		float shellThickness;
		float clipHeight;

		bool onlyCreateOutter;
	};

	class Moon3D
	{
	public:
		Moon3D();
		~Moon3D();

		void setSource(const std::string& imageName);   //现在支持bmp, png, jpg
		void setSource(const std::string& extension, int fd);
		void setSource(unsigned char* data, int width, int height);   //灰度值

		bool generate(const std::string& stlFile, const Moon3DParam& param);
		trimesh::TriMesh* generate(const Moon3DParam& param);
	protected:
		enchase::Surface* m_surface;
	};
}

#endif // ENCHASE_MOONER3D_1621324751120_H