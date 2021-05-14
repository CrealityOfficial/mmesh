#include "source.h"

namespace enchase
{
	Source::Source()
		: m_baseHeight(0.0f)
		, m_pixel2height(1.0f)
	{

	}

	Source::~Source()
	{

	}

	void Source::setBaseHeight(float baseHeight)
	{
		m_baseHeight = baseHeight;
	}

	void Source::setPixel2Height(float pixel2Height)
	{
		m_pixel2height = pixel2Height;
	}
}