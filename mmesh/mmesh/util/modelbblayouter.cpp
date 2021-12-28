#include "modelbblayouter.h"


namespace mmesh
{
	ModelBBLayouter::ModelBBLayouter()
	{
	}

	ModelBBLayouter::~ModelBBLayouter()
	{
	}

	void ModelBBLayouter::EnlargeBox(trimesh::box3& box, float d)
	{
		box.min += trimesh::vec3(-d, -d, -d);
		box.max += trimesh::vec3(d, d, d);
	};

	void ModelBBLayouter::TranslateBox(trimesh::box3& box, trimesh::vec3 offset)
	{
		if (box.valid)
		{
			box.min += offset;
			box.max += offset;
		}
	}

	void ModelBBLayouter::layout_all(trimesh::box3 workspaceBox, std::vector<int> modelIndices,
		std::function<trimesh::box3(int)> getModelBoxFunc, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc/*, qtuser_core::Progressor* progressor*/)
	{
		/* sort models by size */
		std::sort(modelIndices.begin(), modelIndices.end(), [&getModelBoxFunc](int a, int b)
			{
				trimesh::box3 box1 = getModelBoxFunc(a);
				trimesh::box3 box2 = getModelBoxFunc(b);
				//if (box1.size().x > box2.size().x) { return true; }
				if (box1.size().y > box2.size().y) { return true; }
				//if (box1.size().z > box2.size().z) { return true; }
				return false;
			});

		auto IsBoundBoxHit = [](trimesh::vec3 pos1, trimesh::box3 box1, trimesh::vec3 pos2, trimesh::box3 box2, double safety_distance)
		{
			TranslateBox(box1, pos1 - box1.center());
			TranslateBox(box2, pos2 - box2.center());
			trimesh::box3 box1_copy(box1.min, box1.max);
			EnlargeBox(box1_copy, safety_distance);
			return box1_copy.intersects(box2);
		};

		trimesh::box3 basebox = workspaceBox;            // workspace w.r.t. wcs
		basebox.min.z = 0.0;                                      // ignore z cuz just arrange on xy plane
		basebox.max.z = 0.0;                                     // ignore z cuz just arrange on xy plane
		trimesh::box3 totalbox;                               // box contains all models
		std::vector<int> model_flags;                       // 0:not settled, 1:settled inside, 2:settled outside
		std::vector<trimesh::vec3> positions;                   // position of boxes w.r.t. wcs
		float step_size = 5.0f;
		float model_safety_distance = 2.0;
		float boundary_safety_distance = basebox.size().x * 0.3;
		trimesh::vec3 old_pos, new_pos, stepx(-step_size, 0.0f, 0.0f), stepy(0.0f, -step_size, 0.0f), total_offset;
		bool is_hit = false;
		bool is_last_stop = false;
		bool is_init_pos = true;
		int direction = 0;

		for (int i = 0; i < modelIndices.size(); i++)
		{
			int modelIndex = modelIndices[i];

			trimesh::box3 gbox = getModelBoxFunc(modelIndex);               // model box w.r.t. model frame (DO NOT use local box cuz is not scaled)
			trimesh::vec3 box_size = gbox.size() * 0.5f;

			model_flags.push_back(0);
			positions.push_back(trimesh::vec3());

			old_pos[0] = basebox.max.x - boundary_safety_distance - box_size.x;
			old_pos[1] = basebox.max.y - boundary_safety_distance - box_size.y;
			old_pos[2] = gbox.center().z;                                                                // keep the same z as gbox.center
			new_pos = old_pos;
			is_last_stop = false;
			is_init_pos = true;

			// sliding from right-top to left-bottom
			while (true)
			{
				is_hit = false;
				if (new_pos.x < basebox.min.x + boundary_safety_distance + box_size.x ||
					new_pos.y < basebox.min.y + boundary_safety_distance + box_size.y)
				{
					is_hit = true;
				}
				else
				{
					for (size_t j = 0; j < i; j++)
					{
						int modelIndexJ = modelIndices[j];
						if (model_flags[j] && IsBoundBoxHit(new_pos, gbox, positions[j], getModelBoxFunc(modelIndexJ), model_safety_distance))
						{
							is_hit = true;
							break;
						}
					}
				}

				if (is_hit)
				{
					if (is_init_pos)
					{
						break;
					}
					if (is_last_stop)
					{
						model_flags[i] = 1;
						positions[i] = old_pos;
						break;
					}
					is_last_stop = true;
					direction++;
				}
				else
				{
					old_pos = new_pos;
					is_last_stop = false;
					is_init_pos = false;
				}
				new_pos = old_pos + ((direction % 2) ? stepx : stepy);
			}

			if (0 == model_flags[i])
			{
				if (boundary_safety_distance - 2.0f > 0.0f)
				{
					if (boundary_safety_distance - 2.0f * step_size - 2.0f > 0.0f)
					{
						boundary_safety_distance -= 2.0f * step_size;
					}
					else
					{
						boundary_safety_distance = 2.0f;
					}
					totalbox.clear();
					model_flags.clear();
					positions.clear();
					direction = 0;
					i = -1;
				}
				continue;  // if we cannot layout this model, adjust safety distance and try again
			}

			// expand totalbox
			if (!totalbox.valid)
			{
				totalbox = gbox;
				totalbox.min.z = 0.0;                                      // ignore z cuz just arrange on xy plane
				totalbox.max.z = 0.0;                                     // ignore z cuz just arrange on xy plane
				TranslateBox(totalbox, positions[i] - totalbox.center());
			}
			else
			{
				if (totalbox.min.x > positions[i].x - box_size.x)
				{
					totalbox.min[0] = positions[i].x - box_size.x;
				}
				if (totalbox.max.x < positions[i].x + box_size.x)
				{
					totalbox.max[0] = positions[i].x + box_size.x;
				}
				if (totalbox.min.y > positions[i].y - box_size.y)
				{
					totalbox.min[1] = positions[i].y - box_size.y;
				}
				if (totalbox.max.y < positions[i].y + box_size.y)
				{
					totalbox.max[1] = positions[i].y + box_size.y;
				}
			}
		}

		// settle models that can be settled inside
		total_offset = (basebox.size() - totalbox.size()) * 0.5 - trimesh::vec3(boundary_safety_distance, boundary_safety_distance, 0.0f);
		for (size_t i = 0; i < modelIndices.size(); i++)
		{
			if (0 == model_flags[i])
			{
				continue;
			}

			trimesh::vec3 newBoxCenter = total_offset + positions[i];
			int modelIndexI = modelIndices[i];
			modelPositionUpdateFunc(modelIndexI, newBoxCenter);
		}

		// settle models that cannot be settled inside
		for (size_t i = 0; i < modelIndices.size(); i++)
		{
			if (1 == model_flags[i])
			{
				continue;
			}
			ModelBBLayouter::layout_by_search(i, modelIndices, model_flags, workspaceBox, step_size, getModelBoxFunc, modelPositionUpdateFunc);
		}

		//checkModelRange();
	}


	void ModelBBLayouter::layout_by_search(int vector_index, std::vector<int>& modelIndices, std::vector<int>& model_flags, trimesh::box3& workspaceBox, float step_size,
		std::function<trimesh::box3(int)> getModelBoxFunc, std::function<void(int, trimesh::vec3)> modelPositionUpdateFunc)
	{
		if (vector_index >= modelIndices.size() || vector_index >= model_flags.size() || 1 == model_flags[vector_index])
		{
			return;
		}

		int modelIndex = modelIndices[vector_index];
		trimesh::box3 basebox = workspaceBox;
		trimesh::box3 gbox = getModelBoxFunc(modelIndex);
		trimesh::vec3 box_size = gbox.size() * 0.5f;
		trimesh::vec3 position(basebox.min.x + box_size.x, basebox.min.y + box_size.y, gbox.center().z);     // keep the same z as gbox.center
		bool is_hit = false;

		// search inside (from left-bottom to right-top)
		EnlargeBox(basebox, -step_size);
		basebox.min.z = 0.0;                                      // ignore z cuz just arrange on xy plane
		basebox.max.z = 0.0;                                     // ignore z cuz just arrange on xy plane
		while (true)
		{
			is_hit = false;

			if (position.x > basebox.max.x - box_size.x || position.y > basebox.max.y - box_size.y)
			{
				break;
			}

			for (size_t i = 0; i < modelIndices.size(); i++)
			{
				if (vector_index == i || 1 != model_flags[i])
				{
					continue;
				}
				TranslateBox(gbox, position - gbox.center());
				
				int modelIndexI = modelIndices[i];
				if (gbox.intersects(getModelBoxFunc(modelIndexI)))
				{
					is_hit = true;
					break;
				}
			}

			if (!is_hit)
			{
				trimesh::vec3 newBoxCenter = position;
				modelPositionUpdateFunc(modelIndex, newBoxCenter);
				model_flags[vector_index] = 1;
				break;
			}

			if (position.x + step_size > basebox.max.x - box_size.x)
			{
				position[0] = (basebox.min.x + box_size.x);  // left
				position[1] = (position.y + step_size);
			}
			else
			{
				position[0] = (position.x + step_size);
			}
		}

		if (model_flags[vector_index])  // settled inside
		{
			return;
		}

		/* search outside
		*              <-    6
		*        |     <-    2 |
		*      --|-------------|--
		*  v7 v3 |  Workspace  | 1^ 5^
		*      --|-------------|--
		*        |   0 ->      |
		*            4 ->
		*         ...
		*/
		//basebox = workspaceBox;
		EnlargeBox(basebox, 2.0f * step_size);
		basebox.min.z = 0.0;                                      // ignore z cuz just arrange on xy plane
		basebox.max.z = 0.0;                                     // ignore z cuz just arrange on xy plane
		position[0] = basebox.min.x - step_size + box_size.x;
		position[1] = basebox.min.y - step_size - box_size.y;
		int direction = 0;
		trimesh::vec3 outside_distance = box_size;
		while (true)
		{
			is_hit = false;

			for (size_t i = 0; i < modelIndices.size(); i++)
			{
				if (vector_index == i || 2 != model_flags[i])
				{
					continue;
				}
				TranslateBox(gbox, position - gbox.center());

				int modelIndexI = modelIndices[i];
				if (gbox.intersects(getModelBoxFunc(modelIndexI)))
				{
					is_hit = true;
					break;
				}
			}

			if (!is_hit)
			{
				trimesh::vec3 newBoxCenter = position;
				modelPositionUpdateFunc(modelIndex, newBoxCenter);
				model_flags[vector_index] = 2;
				break;
			}

			switch (direction % 4)
			{
			case 0:
				if (position.x > basebox.max.x + outside_distance.x)
				{
					position[1] = position.y + step_size;
					direction++;
				}
				else
				{
					position[0] = position.x + step_size;
				}
				break;
			case 1:
				if (position.y > basebox.max.y + outside_distance.y)
				{
					position[0] = position.x - step_size;
					direction++;
				}
				else
				{
					position[1] = position.y + step_size;
				}
				break;
			case 2:
				if (position.x < basebox.min.x - outside_distance.x)
				{
					position[1] = position.y - step_size;
					direction++;
				}
				else
				{
					position[0] = position.x - step_size;
				}
				break;
			case 3:
				if (position.y < basebox.min.y - outside_distance.y)
				{
					position[1] = position.y - step_size;
					outside_distance += trimesh::vec3(step_size, step_size, 0.0);
					direction++;
				}
				else
				{
					position[1] = position.y - step_size;
				}
				break;
			default:
				break;
			}
		}
	}
}
