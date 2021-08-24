#ifndef ENCHASE_OPTIMIZEENCHASER_1623637204192_H
#define ENCHASE_OPTIMIZEENCHASER_1623637204192_H

namespace enchase
{
	struct EnchaseParam
	{
		float D;
		int blur;
		EnchaseParam()
			:D(0.02f)
			,blur(0)
		{

		}
	};

	class Texture
	{
	public:
		Texture(int width, int height, unsigned char* texture);
		~Texture();

		bool texcoordGet(float s, float t, unsigned char& v) const;
		bool texcoordGet(int s, int t, unsigned char& v) const;
	protected:
		int m_width;
		int m_height;
		unsigned char* m_pixel;
		bool m_valid;
	};

	void enchasePosition(int vertexNum, float* position, float* UV,
		int uvStride, int width, int height, unsigned char* texture, const EnchaseParam& param);
}

#endif // ENCHASE_OPTIMIZEENCHASER_1623637204192_H