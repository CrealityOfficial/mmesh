#ifndef MMESH_TEXTURE_1665380482664_H
#define MMESH_TEXTURE_1665380482664_H
#include "mmesh/enchase/imageloader.h"

namespace enchase
{
	enum class TextureState
	{
		pts_normal,
		pts_edit,
		pts_hide
	};

	class PixTexture
	{
	public:
		PixTexture();
		virtual ~PixTexture();

		bool visibility();
		TextureState state();
		void setState(TextureState state);

		const ImageData& raw() const;
		const ImageData& alpha() const;

		int width();
		int height();

		void initializeGradient(int width, int height, unsigned char start, unsigned char end);
		//type = 0 Gray8 type = 3 R5G6B5 , type = 4 R4G4B4A4 , type = 5 R8G8B8A8
		void fillData(int width, int height, unsigned char* data, int type, bool fillAlpha = false, bool update = true);
		void resize(int width, int height, bool alpha = false, bool update = true);

		void notifyUpdate(bool alpha = true);

		int ID() const;

		int Order();
		void setOrder(int _order);
	protected:
		virtual void onStateChanged() {}
		virtual void onDataChanged(const ImageData& data, ImageData* alpha) {}
		virtual void setBump(bool bump) {}
	protected:
		enchase::GrayImage textureImage;

		const int id;
		static int idGenerator;
		int order;

		TextureState m_state;
	};
}

#endif // MMESH_TEXTURE_1665380482664_H