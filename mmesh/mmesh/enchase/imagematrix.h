#ifndef ENCHASE_IMAGEMATRIX_1600480088459_H
#define ENCHASE_IMAGEMATRIX_1600480088459_H
#include <stdlib.h>
#include <stdint.h>
#include <memory>
#include <math.h>
#include <string.h>

namespace enchase
{
#ifndef uchar
	typedef unsigned char uchar;
#endif

#ifndef uint
	typedef unsigned int uint;
#endif

	template<typename T>
	class Matrix
	{
	public:
		enum PADDING
		{
			PAD_ZERO,     // 0 填充
			PAD_EDGE,     // 使用边缘填充
			PAD_REFLECT,  // 反射填充
		};

	protected:
		uint m_width;
		uint m_height;
		uint m_depth;

		T* m_data;

	protected:
		uint m_lineWidth;


	public:
		Matrix() : m_width(0), m_height(0), m_depth(0), m_data(nullptr) {}

		Matrix(uint w, uint h, uint depth = 1) : m_width(w), m_height(h), m_depth(depth)
		{
			m_lineWidth = m_depth * m_width;
			m_data = (T*)malloc(m_height * m_lineWidth * sizeof(T));
			memset(m_data, 0, m_width * m_height * m_depth * sizeof(T));
		}

		Matrix(uint w, uint h, uint depth, T data) : m_width(w), m_height(h), m_depth(depth)
		{
			m_lineWidth = m_depth * m_width;
			m_data = (T*)malloc(m_height * m_lineWidth * sizeof(T));
			for (uint i = 0; i < m_height * m_width * m_depth; i++)
			{
				m_data[i] = data;
			}
		}

		Matrix(const Matrix& matrix)
		{
			m_width = matrix.width();
			m_height = matrix.height();
			m_depth = matrix.depth();

			m_lineWidth = m_depth * m_width;

			m_data = (T*)malloc(m_height * m_lineWidth * sizeof(T));
			memcpy(m_data, matrix.ptr(), m_width * m_height * m_depth * sizeof(T));
		}

		virtual ~Matrix()
		{
			if (m_data != nullptr)
			{
				free(m_data);
				m_data = nullptr;
			}
		}

		void resize(uint w, uint h, uint depth)
		{
			m_width = w;
			m_height = h;
			m_depth = depth;
			m_lineWidth = m_depth * m_width;

			m_data = (T*)realloc(m_data, m_height * m_lineWidth * sizeof(T));
		}

		inline uint width() const
		{
			return m_width;
		}

		inline uint height() const
		{
			return m_height;
		}

		inline uint depth() const
		{
			return m_depth;
		}

		T* ptr(uint row = 0)
		{
			return m_data + row * m_lineWidth;
		}

		const T* ptr(uint row = 0) const
		{
			return m_data + row * m_lineWidth;
		}

		T* ptr(uint row, uint col)
		{
			return m_data + row * m_lineWidth + col * m_depth;
		}

		const T* ptr(uint row, uint col) const
		{
			return m_data + row * m_lineWidth + col * m_depth;
		}

		template<typename _Tp> _Tp& at(uint row, uint col)
		{
			return *((_Tp*)(m_data + row * m_lineWidth + col * m_depth));
		}

		template<typename _Tp> const _Tp& at(uint row, uint col) const
		{
			return *((const _Tp*)(m_data + row * m_lineWidth + col * m_depth));
		}

		Matrix& operator = (const Matrix& matrix)
		{
			if (this == &matrix)
			{
				return *this;
			}

			if (m_data)
			{
				free(m_data);
				m_data = nullptr;
			}

			m_width = matrix.width();
			m_height = matrix.height();
			m_depth = matrix.depth();

			m_lineWidth = m_depth * m_width;

			m_data = (T*)malloc(m_height * m_lineWidth * sizeof(T));   // 
			memcpy(m_data, matrix.ptr(), m_width * m_height * m_depth * sizeof(T));

			return *this;
		}

		Matrix& operator = (T v)
		{
			for (int i = 0; i < m_height * m_width * m_depth; i++)
			{
				m_data[i] = v;
			}
			return *this;
		}

		T& operator [](uint pos)
		{
			return m_data[pos];
		}

		friend Matrix operator - (T v, Matrix& f)  // 重载 - 
		{
			Matrix m(f);
			T* mdata = m.ptr();
			T* fdata = f.ptr();
			for (uint i = 0; i < f.m_height * f.m_width * f.m_depth; i++)
			{
				*mdata++ = v - *fdata++;
			}
			return m;
		}

		friend Matrix operator - (Matrix& f, T v)  // 重载 - 
		{
			Matrix m(f);
			T* mdata = m.ptr();
			T* fdata = f.ptr();
			for (int i = 0; i < f.m_height * f.m_width * f.m_depth; i++)
			{
				*mdata++ = *fdata++ - v;
			}
			return m;
		}

		friend Matrix operator + (T v, Matrix& f)  // 重载 + ,矩阵在 右边
		{
			Matrix m(f);
			T* mdata = m.ptr();
			T* fdata = f.ptr();
			for (uint i = 0; i < f.m_height * f.m_width * f.m_depth; i++)
			{
				*mdata++ = v + *fdata++;
			}
			return m;
		}

		friend Matrix operator + (Matrix& f, T v)  // 重载 + ,矩阵在 左边
		{
			Matrix m(f);
			T* mdata = m.ptr();
			T* fdata = f.ptr();
			for (int i = 0; i < f.m_height * f.m_width * f.m_depth; i++)
			{
				*mdata++ = *fdata++ + v;
			}
			return m;
		}

		friend Matrix operator * (T v, Matrix& f)  // 重载 *  
		{
			Matrix m(f);
			T* mdata = m.ptr();
			T* fdata = f.ptr();
			for (uint i = 0; i < f.m_height * f.m_width * f.m_depth; i++)
			{
				*mdata++ = v * *fdata++;
			}
			return m;
		}

		friend Matrix operator * (Matrix& f, T v)  // 重载 *  
		{
			Matrix m(f);
			T* mdata = m.ptr();
			T* fdata = f.ptr();
			for (int i = 0; i < f.m_height * f.m_width * f.m_depth; i++)
			{
				*mdata++ = *fdata++ * v;
			}
			return m;
		}

		friend Matrix operator / (T v, Matrix& f)  // 重载 *  
		{
			Matrix m(f);
			T* mdata = m.ptr();
			T* fdata = f.ptr();
			for (int i = 0; i < f.m_height * f.m_width * f.m_depth; i++)
			{
				*mdata++ = v / *fdata++;
			}
			return m;
		}

		friend Matrix operator / (Matrix& f, T v)  // 重载 *  
		{
			Matrix m(f);
			T* mdata = m.ptr();
			T* fdata = f.ptr();
			for (int i = 0; i < f.m_height * f.m_width * f.m_depth; i++)
			{
				*mdata++ = *fdata++ / v;
			}
			return m;
		}

		// 根据纹理坐标获取值，数据存储在 v 中
		// s 和 t 的分布范围是 [0, 1]
		// 采用双线性差值
		bool texcoordGet(float s, float t, T& v) const
		{
			//t = 1.0 - t;
			if (m_depth != 1)
			{
				return false;
			}
			if (s < 0 || s > 1 || t < 0 || t > 1)
			{
				return false;
			}
			float tv = t * (m_height - 1);
			float sv = s * (m_width - 1);

			int y = round(tv);
			int x = round(sv);

			float xper = abs(x - sv);
			float yper = abs(y - tv);

			// 需要双线性插值
			if (yper >= 0.01)    
			{
				T* data1 = m_data + y * m_lineWidth + x;
				T* data2 = ((tv - y) > 0) ? data1 + m_lineWidth : data1 - m_lineWidth;

				T v1 = *data1;
				T v2 = *data2;

				// 第一次线性插值
				if (xper >= 0.01)   // 在 0.01 范围内，则不进行插值
				{
					bool b = (sv - x) >= 0;

					T _t = b ? *(data1 + 1) : *(data1 - 1);
					v1 = (T)(v1 * (1 - xper) + _t * xper);

					_t = b ? *(data2 + 1) : *(data2 - 1);
					v2 = (T)(v2 * (1 - xper) + _t * xper);
				}

				v = (T)(v1 * (1 - yper) + v2 * yper);   // 第二次线性插值
			}
			else    // 只需要一次线性插值
			{
				if (xper >= 0.01)   // 在 0.01 范围内，则不进行插值
				{
					T* data1 = m_data + y * m_lineWidth + x;
					T v1 = *data1;

					bool b = (sv - x) >= 0;
					T _t = b ? *(data1 + 1) : *(data1 - 1);
					v = (T)(v1 * (1 - xper) + _t * xper);
				}
				else
				{
					v = m_data[y * m_lineWidth + x];
				}
			}
			return true;
		}

		// 使用 convm 对自身进行卷积，结果保存到 outm 中
		int conv(const Matrix& convm, PADDING pad, Matrix& outm) const
		{
			if (m_depth != 1 || convm.m_depth != 1)
			{
				return -1;
			}

			int i, j, i1, j1, pos1, pos2;
			int h1, w1, h2, w2, ch, cw;
			double summ;

			//变量初始化
			h1 = m_height;
			w1 = m_width;

			h2 = convm.m_height;    // 传递 图像矩阵数据 convm
			w2 = convm.m_width;
			//printf("h2=%d,w2=%d ", h2, w2);    // h2 = 3,w2 = 3   

			//矩阵 f2 的中心
			ch = (h2 - 1) / 2;
			cw = (w2 - 1) / 2;
			//printf("ch=%d,cw=%d ", ch, cw);   // ch = 1,cw = 1 


			for (i = 0; i < h1; i++)
			{
				for (j = 0; j < w1; j++)
				{
					summ = 0;
					for (i1 = 0; i1 < h2; i1++)
					{
						for (j1 = 0; j1 < w2; j1++)
						{
							//计算对应的 f1 坐标
							pos1 = i + i1 - ch;
							pos2 = j + j1 - cw;

							T d;
							bool set_zero = false;

							if (pos1 < 0)
							{
								switch (pad)
								{
								case PAD_ZERO:
									d = (T)(0);
									set_zero = true;
									break;
								case PAD_EDGE:
									pos1 = 0;
									break;
								case PAD_REFLECT:
									pos1 = -1 - pos1;
									break;
								}
							}
							else if (pos1 >= h1)
							{
								switch (pad)
								{
								case PAD_ZERO:
									d = (T)(0);
									set_zero = true;
									break;
								case PAD_EDGE:
									pos1 = h1 - 1;
									break;
								case PAD_REFLECT:
									pos1 = (h1 - 1) - (pos1 - h1);
									break;
								}
							}

							if (pos2 < 0)
							{
								switch (pad)
								{
								case PAD_ZERO:
									d = (T)(0);
									set_zero = true;
									break;
								case PAD_EDGE:
									pos2 = 0;
									break;
								case PAD_REFLECT:
									pos2 = -1 - pos2;
									break;
								}
							}
							else if (pos2 >= w1)
							{
								switch (pad)
								{
								case PAD_ZERO:
									d = (T)(0);
									set_zero = true;
									break;
								case PAD_EDGE:
									pos2 = w1 - 1;
									break;
								case PAD_REFLECT:
									pos2 = (w1 - 1) - (pos2 - w1);
									break;
								}
							}

							if (!set_zero)   // m_data = new T[m_width * m_height * m_depth];    
							{
								summ += m_data[pos1 * w1 + pos2] * convm.m_data[i1 * w2 + j1];

							}

						}
					}
					outm.m_data[i * w1 + j] = summ;
				}
			}

			return 0;
		}


	};


	typedef Matrix<uchar> MatrixU;
	typedef Matrix<float> MatrixF;   // 模板是T，换成float , MatrixF(名称) 相当于Matrix 的参数 
}

#endif // ENCHASE_IMAGEMATRIX_1600480088459_H