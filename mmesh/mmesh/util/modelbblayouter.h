#ifndef MMESH_MODELLAYOUTER_1589275563011_H
#define MMESH_MODELLAYOUTER_1589275563011_H

#include <vector>
#include <functional>

#include "trimesh2/Box.h"

//#include "qtusercore/module/progressor.h"

namespace mmesh
{
	class ModelBBLayouter
	{
	public:
		ModelBBLayouter();
		virtual ~ModelBBLayouter();

		/* layout all models BoundingBox in the workspace using a Bottom-Left Algorithm 
		*   param:
		*   workspaceBox:                              total box in which the model BBs to be arranged
		*   modelIndices:                                model index array, used to get BB and update model position
		*   getModelBoxFunc:                       function used to get model BB
		*                                                             input parameter: model index
		*   modelPositionUpdateFunc:        function used to update model position during layout_all
		*                                                             input parameter: model index and new BB center
		*/ 
		static void layout_all(trimesh::box3 workspaceBox, std::vector<int> modelIndices, 
			std::function<trimesh::box3(int)> getModelBoxFunc, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc/*, qtuser_core::Progressor* progressor*/);


	private:
		/* first, search for an appropriate position inside the workspace, if cannot find such a position, search outside the workspace then.*/
		static void layout_by_search(int vector_index, std::vector<int>& modelIndices, std::vector<int>& model_flags, trimesh::box3& workspaceBox, float step_size, 
			std::function<trimesh::box3(int)> getModelBoxFunc, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc);

		static void EnlargeBox(trimesh::box3& box, float d);
		static void TranslateBox(trimesh::box3& box, trimesh::vec3 offset);
	};
}
#endif // MMESH_MODELLAYOUTER_1589275563011_H
