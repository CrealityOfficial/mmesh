#ifndef ENCHASE_IMAGEMATRIXFSOURCE_1600481539855_H
#define ENCHASE_IMAGEMATRIXFSOURCE_1600481539855_H
#include "source.h"
#include "imagematrix.h"

namespace enchase
{
	class MatrixFSource : public Source
	{
	public:
		MatrixFSource(MatrixF* matrix);
		virtual ~MatrixFSource();

		bool tex(float& t, float x, float y) override;
		int width() override;
		int height() override;
		MatrixF* matrix();
	protected:
		MatrixF* m_matrix;
	};
}

#endif // ENCHASE_IMAGEMATRIXFSOURCE_1600481539855_H