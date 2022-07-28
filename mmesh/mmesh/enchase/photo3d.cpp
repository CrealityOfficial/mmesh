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
		blurTimes = 3;

		baseThickness = 0.0f;
		maxThickness = 2.2f;
		invert = false;
		useIndex = 0;

		maxPixel = 1000;
		realWidth = 140.0f;
	}

	Photo3DParam::~Photo3DParam()
	{

	}

	Photo3D::Photo3D()
        : m_surface(nullptr),
          m_serialNumber(nullptr)
	{
	}

	Photo3D::~Photo3D()
	{
		if (m_surface)
		{
			delete m_surface;
			m_surface = nullptr;
		}
        
        if (m_serialNumber) {
            delete m_serialNumber;
            m_serialNumber = nullptr;
        }
	}

	void Photo3D::setLogger(const std::string& logger)
	{
	}

	void Photo3D::setSource(const std::string& extension, int fd)
	{
		enchase::ImageData image;

		enchase::loadImage_freeImage(image, extension, fd);

		if (image.width == 0 || image.height == 0 || image.data == nullptr)
			return;

		setSource(image.data, image.width, image.height);
		if (image.data)
		{
			delete[] image.data;
		}
	}

	void Photo3D::setSource(const std::string& imageName)
	{
		enchase::ImageData image;

		enchase::loadImage_freeImage(image, imageName);

		if(image.width == 0 || image.height == 0 || image.data == nullptr)
			return;

		setSource(image.data, image.width, image.height);
		if (image.data)
		{
			delete [] image.data;
            image.data = nullptr;
		}
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
    
    void Photo3D::setSerialNumber(unsigned char* data, int width, int height)
    {
        enchase::MatrixF* imgMtx = new enchase::MatrixF(width, height, 1);
        float* imgData = imgMtx->ptr(0);

        for (int h = 0; h < height; h++)
        {
            imgData = imgMtx->ptr(h);
            for (int w = 0; w < width; w++)
            {
                unsigned char c = data[h * width + w];
                imgData[w] = (float)c / 255.0f;
            }
        }

        m_serialNumber = imgMtx;
    }

    void Photo3D::setSerialNumber(enchase::MatrixF *serial)
    {
        if (m_serialNumber) {
            delete m_serialNumber;
            m_serialNumber = nullptr;
        }
        if (serial) {
            enchase::MatrixF *newMat = new enchase::MatrixF(*serial);
            m_serialNumber = newMat;
        }
    }

	bool Photo3D::generate(const std::string& stlFile, const Photo3DParam& param, int& errorCode)
	{		
		std::unique_ptr<trimesh::TriMesh> destPtr(generate(param, errorCode));
		if (destPtr)
		{
			destPtr->write(stlFile);
			return true;
		}

		return false;
	}

	trimesh::TriMesh* Photo3D::generate(const Photo3DParam& param, int& errorCode)
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
			errorCode = 1;
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
        enchase::Enchaser enchaser;
        enchaser.setSource(mesh);
        
		enchase::Mapper mapper;
		enchase::defaultCoord(width, height, mapper.allTextureGroup());

		enchase::MatrixFSource* source = new enchase::MatrixFSource(matrix);
		mapper.setSource(source);
        
        if (m_serialNumber) {
            enchase::MatrixF* newSerial = new enchase::MatrixF(*m_serialNumber);
            enchase::MatrixFSource s(newSerial);
            enchaser.enchaseCache(&mapper, 0, 0.037, 0.037*whRatio, 2.2, &s);
        } else {
            enchaser.enchaseCache(&mapper, 0, 0.037, 0.037*whRatio, 2.2);
        }

		trimesh::TriMesh* generated = enchaser.takeCurrent();
		delete mesh;

		return generated;
	}
}
