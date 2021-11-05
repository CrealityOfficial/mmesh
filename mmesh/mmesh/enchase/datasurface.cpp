#include "datasurface.h"
#include "blur.h"
#include <math.h>

#if _WIN32
#include <algorithm>
#endif

namespace enchase
{
	DataSurface::DataSurface()
		:m_width(0)
		, m_height(0)
		, m_data(nullptr)
	{

	}

	DataSurface::~DataSurface()
	{
		if (m_data)
		{
			delete [] m_data;
			m_data = nullptr;
		}
	}

	void DataSurface::setData(unsigned char* data, int width, int height)
	{
		m_width = width;
		m_height = height;

		if (m_width > 0 && m_height > 0)
		{
			m_data = new unsigned char[m_width * m_height];
			memcpy(m_data, data, m_width * m_height * sizeof(char));
		}
	}

	MatrixF* DataSurface::produce()
	{
		uint width = (uint)m_width;
		uint height = (uint)m_height;
		if (width == 0 || height == 0)
		{
			return nullptr;
		}

		MatrixF* imgMtx = new MatrixF(width, height, 1);
		float* imgData = imgMtx->ptr(0);

		for (int h = 0; h < height; h++)
		{
            imgData = imgMtx->ptr(h);
			for (int w = 0; w < width; w++)
			{
				unsigned char c = m_data[h * m_width + w];
				imgData[w] = (float)c / 255.0f;
			}
		}
		return imgMtx;
	}
    
    void DataSurface::original(MatrixU* matrix)
    {
        //‘≠Õº
        enchase::blur(matrix, 3);
    }

    void DataSurface::oilPainting(MatrixU* matrix)
    {
        //”Õª≠
//        stepData(matrix, 10, false);
//        enchase::blur(matrix, 2);
#if 1
        MatrixU* src = new MatrixU(*matrix);
        int width = src->width();
        int height = src->height();
        
        auto texture = [&src, width, height](int v, int u) ->unsigned char{
            
            int safeU = std::min(u, width-1);
            safeU = std::max(0, safeU);
            
            int safeV = std::min(v, height-1);
            safeV = std::max(0, safeV);
            
            return *src->ptr(safeV, safeU);
        };
        
        int radius = 7;
        for (int h = 0; h < matrix->height(); h++) {
            for (int w = 0; w < matrix->width(); w++) {
                
                float n = float((radius + 1) * (radius + 1));
                int i; int j;
                
                int m0 = 0; int m1 = 0; int m2 = 0; int m3 = 0;
                int s0 = 0; int s1 = 0; int s2 = 0; int s3 = 0;
                int c = 0;
                unsigned char color = 0;
                
                for (j = -radius; j <= 0; ++j)  {
                    for (i = -radius; i <= 0; ++i)  {
                      c = texture(h + j, w + i);
                      m0 += c;
                      s0 += c * c;
                    }
                }

                for (j = -radius; j <= 0; ++j)  {
                    for (i = 0; i <= radius; ++i)  {
                      c = texture(h + j, w + i);
                      m1 += c;
                      s1 += c * c;
                    }
                }

                for (j = 0; j <= radius; ++j)  {
                    for (i = 0; i <= radius; ++i)  {
                      c = texture(h + j, w + i);
                      m2 += c;
                      s2 += c * c;
                    }
                }

                for (j = 0; j <= radius; ++j)  {
                    for (i = -radius; i <= 0; ++i)  {
                      c = texture(h + j, w + i);
                      m3 += c;
                      s3 += c * c;
                    }
                }

#if _WIN32
                float min_sigma2 = FLT_MAX;
#else
                float min_sigma2 = MAXFLOAT;
#endif
                m0 /= n;
                s0 = abs(s0 / n - m0 * m0);

                float sigma2 = s0;
                if (sigma2 < min_sigma2) {
                    min_sigma2 = sigma2;
                    color = m0;
                }

                m1 /= n;
                s1 = abs(s1 / n - m1 * m1);

                sigma2 = s1;
                if (sigma2 < min_sigma2) {
                    min_sigma2 = sigma2;
                    color = m1;
                }

                m2 /= n;
                s2 = abs(s2 / n - m2 * m2);

                sigma2 = s2;
                if (sigma2 < min_sigma2) {
                    min_sigma2 = sigma2;
                    color = m2;
                }

                m3 /= n;
                s3 = abs(s3 / n - m3 * m3);

                sigma2 = s3;
                if (sigma2 < min_sigma2) {
                    min_sigma2 = sigma2;
                    color = m3;
                }
                
                *matrix->ptr(h, w) = color;
                
            }
        }
#else
        
        MatrixU* src = new MatrixU(*matrix);
        int width = src->width();
        int height = src->height();
        
        auto texture = [&src, width, height](int v, int u) ->unsigned char{
            
            int safeU = std::min(u, width-1);
            safeU = std::max(0, safeU);
            
            int safeV = std::min(v, height-1);
            safeV = std::max(0, safeV);
            
            return *src->ptr(safeV, safeU);
        };
        
        for (int h = 0; h < matrix->height(); h++) {
            for (int w = 0; w < matrix->width(); w++) {
                
                int array[5] = {0};
                
                for (int i=-3; i<=3; i++) {
                    for (int j=-3; j<=3; j++) {
                        int r = texture(h + i, w + j);
                        int c = 4 * r / 255;
                        array[c] += 1;
                    }
                }
                
                int currentMax = array[0];
                int l = 0;

                for (int k=1; k<=4; k++) {
                    if (currentMax < array[k]) {
                        currentMax = array[k];
                        l = k;
                    }
                }

                int sum = 0;
                for (int i=-3; i<=3; i++) {
                    for (int j=-3; j<=3; j++) {
                        unsigned char r = texture(h + i, w + j);
                        int c = 4 * r / 255;
                        if (c == l) {
                            sum += r;
                        }
                    }
                }
                
                unsigned char x = sum / currentMax;
                *matrix->ptr(h, w) = x;
                
            }
        }
#endif
        
    }

    void DataSurface::sketch(MatrixU* matrix)
    {
        //https://blog.csdn.net/u010468553/article/details/79392171
        //Àÿ√Ë
        MatrixU* src = new MatrixU(*matrix);
        
        //∑¥œ‡
        *src = 0xFF - (*src);
        
        enchase::blur(src, 3);
        
        //—’…´ºıµ≠
        for (int i = 0; i < matrix->height(); i++) {
            
            for (int j = 0; j < matrix->width(); j++) {
                unsigned char base_color = *matrix->ptr(i, j);
                unsigned char final_color = *src->ptr(i, j);
                unsigned char den = (0xFF - base_color);
                if (den == 0x0) {
                    den = 0x1;
                }
                unsigned char c = std::min(base_color * 4 / 5 + base_color * final_color/den * 5 / 4, 0xFF);
                *matrix->ptr(i, j) = c;
            }
        }
        delete src;
    }

    void DataSurface::stepData(MatrixU* matrix, int steps, bool smooth)
    {
        unsigned char map[256] = {0};
        
        int range = 256 / steps;
        
        for (int i=0; i<256; i++) {
            
            int stepIndex = i / range;
            
            if (smooth) {
                
                float r = i - (stepIndex * range);  //r = [0, range]
            
//                float x = (((float)r / (float)range) - 0.5) * M_PI;     //x = [-pi/2, pi/2]
//                float y = (sin(x) + 1.0) / 2.0;                        //y = [0, 1]
                
                float x = ((float)r / (float)range) * M_PI;  //x = [0, pi]
                float y = 1.0 - cos(x);                     //y = [0, 1]
                
//                float x = (float)r / (float)range;
//                float y = x * x * (3 - 2 * x);
                
                int c = y * range + (stepIndex * range);
                c = std::min(c, 255);
                map[i] = (unsigned char)c;

                printf("c = %d, maped = %d\n", i, c);
                
            } else {
                unsigned char c = stepIndex * range;
                map[i] = (unsigned char)c;
                printf("c = %d, maped = %d\n", i, c);
            }
        }
        
        
        MatrixU* src = new MatrixU(*matrix);
        
        int width = matrix->width();
        int height = matrix->height();
        
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                
                unsigned char c = *src->ptr(i, j);
                *matrix->ptr(i, j) = map[c];
                
            }
        }
        
        delete src;
    }
    
}
