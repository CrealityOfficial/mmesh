#include "imagematrixfsource.h"

namespace enchase
{
	MatrixFSource::MatrixFSource(MatrixF* matrix)
		:m_matrix(matrix)
	{

	}

	MatrixFSource::~MatrixFSource()
	{
		if(m_matrix)
			delete m_matrix;
	}

	bool MatrixFSource::tex(float& t, float x, float y)
	{
		bool b = m_matrix->texcoordGet(x, y, t);
		return b;
	}

	int MatrixFSource::width()
	{
		return m_matrix->width();
	}

	int MatrixFSource::height()
	{
		return m_matrix->height();
	}

	MatrixF* MatrixFSource::matrix()
	{
		return m_matrix;
	}
}