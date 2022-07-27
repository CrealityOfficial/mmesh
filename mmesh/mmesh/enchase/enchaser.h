#ifndef ENCHASE_ENCHASER_1600262114250_H
#define ENCHASE_ENCHASER_1600262114250_H
#include "trimesh2/TriMesh.h"
#include <memory>

namespace enchase
{
    class Source;
	class Mapper;
	class Enchaser
	{
	public:
		Enchaser();
		~Enchaser();

		trimesh::TriMesh* source();
		trimesh::TriMesh* enchase(Mapper* mapper, int index);
		void setSource(trimesh::TriMesh* source);
		void write(const std::string& stlFile);

		void enchaseCache(Mapper* mapper, int index);
        void enchaseCache(Mapper* mapper, int index, float horizontalMargin, float verticalMargin, float maxThickness);
        void enchaseCache(Mapper* mapper, int index, float horizontalMargin, float verticalMargin, float maxThickness, Source* serial);
        
        
		trimesh::TriMesh* takeCurrent();
	protected:
		trimesh::TriMesh* m_source;

		std::unique_ptr<trimesh::TriMesh> m_current;
	};
}

#endif // ENCHASE_ENCHASER_1600262114250_H
