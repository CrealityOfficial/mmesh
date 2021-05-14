#ifndef ENCHASE_SOURCE_1600262114220_H
#define ENCHASE_SOURCE_1600262114220_H

namespace enchase
{
	class Source
	{
	public:
		Source();
		virtual ~Source();

		virtual bool tex(float& t, float x, float y) = 0;
		virtual int width() = 0;
		virtual int height() = 0;

		void setBaseHeight(float baseHeight);
		void setPixel2Height(float pixel2Height);
	protected:
		float m_baseHeight;
		float m_pixel2height;
	};
}

#endif // ENCHASE_SOURCE_1600262114220_H