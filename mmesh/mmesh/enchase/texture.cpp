#include "texture.h"

namespace enchase
{
	int PixTexture::idGenerator = 0;
	PixTexture::PixTexture()
		: id(idGenerator)
		, m_state(TextureState::pts_normal)
		, order(0)
	{
		idGenerator++;
	}

	PixTexture::~PixTexture()
	{

	}

	bool PixTexture::visibility()
	{
		return m_state == TextureState::pts_normal;
	}

	TextureState PixTexture::state()
	{
		return m_state;
	}

	void PixTexture::setState(TextureState state)
	{
		m_state = state;
		onStateChanged();
		//m_node->setNodeMask(m_state == TextureState::pts_normal ? 0xFFFFFFFF : 0);
	}

	const ImageData& PixTexture::raw() const
	{
		return textureImage.raw;
	}

	const ImageData& PixTexture::alpha() const
	{
		return textureImage.alpha;
	}

	int PixTexture::width()
	{
		return textureImage.raw.width;
	}

	int PixTexture::height()
	{
		return textureImage.raw.height;
	}

	void PixTexture::initializeGradient(int width, int height, unsigned char start, unsigned char end)
	{
		textureImage.raw.allocate(width, height);
		textureImage.raw.gradient(start, end);
		notifyUpdate(false);
	}

	void PixTexture::fillData(int width, int height, unsigned char* data, int type, bool fillAlpha, bool update)
	{
		enchase::fillImageData(textureImage.raw, fillAlpha ? &textureImage.alpha : nullptr, width, height, data, type);

		if (update)
			notifyUpdate(fillAlpha);
	}

	void PixTexture::resize(int width, int height, bool alpha, bool update)
	{
		textureImage.raw.resize(width, height);
		if (alpha)
			textureImage.alpha.resize(width, height);

		if(update)
			notifyUpdate(alpha);
	}

	void PixTexture::notifyUpdate(bool alpha)
	{
		onDataChanged(textureImage.raw, alpha ? &textureImage.alpha : nullptr);
	}

	int PixTexture::ID() const
	{
		return id;
	}

	int PixTexture::Order()
	{
		return order;
	}

	void PixTexture::setOrder(int _order)
	{
		order = _order;
	}
}