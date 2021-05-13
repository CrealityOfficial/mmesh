#include "edgeprocessor.h"
#include <vector>

namespace enchase
{
	void edgeProcess(MatrixF* matrix, EdgeType type)
	{
		switch (type)
		{
		case enchase::EdgeType::et_none:
			break;
		case enchase::EdgeType::et_zero:
			zeroEdge(matrix);
			break;
		case enchase::EdgeType::et_interpolation_x:
			interpolationEdge(matrix);
			break;
		default:
			break;
		}
	}

	void zeroEdge(MatrixF* matrix)
	{
		int w = matrix->width();
		int h = matrix->height();

		auto f = [&matrix](int w, int h) {
			matrix->at<float>(h, w) = 0.0f;
		};
		for (int i = 0; i < w; ++i)
		{
			int h0 = 0;
			int he = h - 1;
			f(i, h0);
			f(i, he);
		}

		for (int i = 0; i < h; ++i)
		{
			int w0 = 0;
			int we = w - 1;
			f(w0, i);
			f(we, i);
		}
	}

	void interpolationEdge(MatrixF* matrix)
	{
		printf("edge interpolation.\n");

		int stride = 20;

		int w = matrix->width();
		int h = matrix->height();
		if (w < 4 || h < 0)
			return;

		std::vector<int> samples;
		std::vector<std::vector<float>> tmps;
		tmps.resize(2 * stride);
		samples.resize(2 * stride);
		for (int i = 0; i < stride; ++i)
		{
			samples.at(i) = i;
			tmps.at(i).resize(h);
			samples.at(2 * stride - 1 - i) = w - 1 - i;
			tmps.at(2 * stride - 1 - i).resize(h);
		}

		auto interplation = [&w, &h, &matrix](int _w, int _h)->float {
			int s = 1;
			float t = 0.0f;
			t = matrix->at<float>(_h, _w);
			for (int i = 0; i < s; ++i)
			{
				t += matrix->at<float>(_h, (w - _w - i - 1) % w);
				t += matrix->at<float>(_h, (_w + i + 1) % w);
			}

			return (t) / (float)(2 * s + 1);
		};

		for (int i = 0; i < 2 * stride; ++i)
		{
			for (int j = 0; j < h; ++j)
			{
				int _w = samples.at(i);
				int _h = j;
				tmps.at(i).at(j) = interplation(_w, _h);
			}
		}

		//assign
		auto assign = [&matrix](int index, std::vector<float>& col)
		{
			int size = col.size();
			for (int i = 0; i < size; ++i)
				matrix->at<float>(i, index) = col.at(i);
		};

		for (int i = 0; i < stride * 2; ++i)
		{
			assign(samples.at(i), tmps.at(i));
		}
	}
}