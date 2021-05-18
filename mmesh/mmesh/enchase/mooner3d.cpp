#include "mooner3d.h"

#include "mapper.h"
#include "enchaser.h"
#include "imagematrixfsource.h"
#include "surface.h"
#include "datasurface.h"
#include "imageloader.h"

#include "mmesh/enchase/trimeshgenerator.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "mmesh/trimesh/split.h"
#include <memory>

namespace enchase
{
	Moon3DParam::Moon3DParam()
	{
		useBlur = true;
		blurTimes = 9;

		baseThickness = 0.35f;
		maxThickness = 2.2f;
		invert = false;

		radius = 10.0f;
		shellThickness = 1.0f;
		clipHeight = 4.0f;
	}

	Moon3DParam::~Moon3DParam()
	{

	}

	Moon3D::Moon3D()
		:m_surface(nullptr)
	{
	}

	Moon3D::~Moon3D()
	{
		if (m_surface)
		{
			delete m_surface;
			m_surface = nullptr;
		}
	}

	void Moon3D::setSource(const std::string& extension, int fd)
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

	void Moon3D::setSource(const std::string& imageName)
	{
		enchase::ImageData image;

		enchase::loadImage_freeImage(image, imageName);

		if (image.width == 0 || image.height == 0 || image.data == nullptr)
			return;

		setSource(image.data, image.width, image.height);
		if (image.data)
		{
			delete[] image.data;
		}
	}

	void Moon3D::setSource(unsigned char* data, int width, int height)
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

	bool Moon3D::generate(const std::string& stlFile, const Moon3DParam& param)
	{
		std::unique_ptr<trimesh::TriMesh> destPtr(generate(param));
		if (destPtr)
		{
			destPtr->write(stlFile);
			return true;
		}

		return false;
	}

	trimesh::TriMesh* Moon3D::generate(const Moon3DParam& param)
	{
		if (!m_surface)
			return nullptr;

		m_surface->baseHeight = param.baseThickness;
		m_surface->maxHeight = param.maxThickness;
		m_surface->invert = param.invert;
		m_surface->useBlur = param.useBlur;
		m_surface->blurTimes = param.blurTimes;

		trimesh::TriMesh* mesh = nullptr;
		trimesh::TriMesh* meshOut = enchaseGenerate(mesh, SurfacePtr(m_surface));
		m_surface = nullptr;
		delete mesh;

		trimesh::TriMesh* meshIn = nullptr;

		std::vector<trimesh::TriMesh*> meshes;
		meshes.push_back(mesh);
		meshes.push_back(meshIn);

		trimesh::TriMesh* mergedMesh = new trimesh::TriMesh();
		mmesh::mergeTriMesh(mergedMesh, meshes);
		delete meshIn;
		delete meshOut;

		trimesh::TriMesh* meshUpper = nullptr;
		trimesh::TriMesh* meshLower = nullptr;
		if (!mmesh::split(mergedMesh, param.clipHeight, trimesh::vec3(0.0f, 0.0f, 1.0f), &meshUpper, &meshLower))
			meshUpper = mergedMesh;
		
		delete mergedMesh;
		delete meshLower;
		return meshUpper;
	}
}