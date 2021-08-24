#include "photo3d.h"
#include "default.h"
#include "mapper.h"
#include "enchaser.h"
#include "imagematrixfsource.h"
#include "surface.h"
#include "datasurface.h"
#include "imageloader.h"

#include <memory>

namespace enchase
{
	Photo3DParam::Photo3DParam()
	{
		useBlur = true;
		blurTimes = 9;

		baseThickness = 0.0f;
		maxThickness = 2.2f;
		invert = true;
		useIndex = 0;

		maxPixel = 600;
		realWidth = 140.0f;
	}

	Photo3DParam::~Photo3DParam()
	{

	}

	Photo3D::Photo3D()
		:m_surface(nullptr)
	{
	}

	Photo3D::~Photo3D()
	{
		if (m_surface)
		{
			delete m_surface;
			m_surface = nullptr;
		}
	}

	void Photo3D::setLogger(const std::string& logger)
	{
	}

	void Photo3D::setSource(const std::string& extension, int fd)
	{
		enchase::ImageData image;

		//enchase::loadImage_freeImage(image, extension, fd);

		//if (image.width == 0 || image.height == 0 || image.data == nullptr)
		//	return;

		//setSource(image.data, image.width, image.height);
		//if (image.data)
		//{
		//	delete[] image.data;
		//}
	}

	void Photo3D::setSource(const std::string& imageName)
	{
		enchase::ImageData image;

		//enchase::loadImage_freeImage(image, imageName);

		//if(image.width == 0 || image.height == 0 || image.data == nullptr)
		//	return;

		//setSource(image.data, image.width, image.height);
		//if (image.data)
		//{
		//	delete [] image.data;
		//}
	}

	void Photo3D::setSource(unsigned char* data, int width, int height)
	{
		if (m_surface)
		{
			delete m_surface;
			m_surface = nullptr;
		}

		enchase::DataSurface* surface = new enchase::DataSurface();
		surface->setData(data, width, height);
		m_surface = surface;
	}

	bool Photo3D::generate(const std::string& stlFile, const Photo3DParam& param, std::string* error)
	{		
		std::unique_ptr<trimesh::TriMesh> destPtr(generate(param, error));
		if (destPtr)
		{
			destPtr->write(stlFile);
			return true;
		}

		if (error)
			*error = "enchase error.";

		return false;
	}

	trimesh::TriMesh* Photo3D::generate(const Photo3DParam& param, std::string* error)
	{
		enchase::MatrixF* matrix = nullptr;
		if (m_surface)
		{
			m_surface->baseHeight = param.baseThickness;
			m_surface->maxHeight = param.maxThickness;
			m_surface->invert = param.invert;
			m_surface->useBlur = param.useBlur;
			m_surface->blurTimes = param.blurTimes;

			matrix = m_surface->matrix();
		}

		if (!matrix)
		{
			return nullptr;
		}

		float whRatio = (float)matrix->width() / (float)matrix->height();
		int width = param.maxPixel;
		int height = param.maxPixel;
		if (whRatio > 1.0f)
			height = (int)((float)width / whRatio);
		else
			width = (int)((float)height * whRatio);

		float pixel = param.realWidth / (float)width;
		trimesh::TriMesh* mesh = enchase::defaultPlane(width, height, pixel);

		enchase::Mapper mapper;
		enchase::Enchaser enchaser;
		enchaser.setSource(mesh);

		enchase::defaultCoord(width, height, mapper.allTextureGroup());

		enchase::MatrixFSource* source = new enchase::MatrixFSource(matrix);
		mapper.setSource(source);
		enchaser.enchaseCache(&mapper, 0);

		trimesh::TriMesh* generated = enchaser.takeCurrent();
		delete mesh;

		return generated;
	}
}