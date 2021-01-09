#include "mmesh/trimesh/contour.h"
#include <list>
#include <assert.h>

#define FACE(x, y , z) trimesh::TriMesh::Face(x, y, z)

namespace mmesh
{
	void zoff(trimesh::vec3* svertex, trimesh::vec3* dvertex, size_t size, float z)
	{
		trimesh::vec3 offz = trimesh::vec3(0.0f, 0.0f, z);
		for (size_t i = 0; i < size; ++i)
		{
			*(dvertex + i) = *(svertex + i) + offz;
		}
	}

	void fillLoop(int sindex1, int sindex2, int size, trimesh::TriMesh::Face* faces)
	{
		if (size < 2) return;

		int findex = 0;
		for (int i = 0; i < size; ++i)
		{
			int i1 = sindex1 + i;
			int i3 = sindex2 + i;
			int i2 = i1 + 1;
			int i4 = i3 + 1;

			if (i == size - 1)
			{
				i2 = sindex1;
				i4 = sindex2;
			}

			*(faces + findex++) = FACE(i1, i2, i4);
			*(faces + findex++) = FACE(i1, i4, i3);
		}
	}

	void sortBoxes(const trimesh::box3& box, const std::vector<trimesh::box3>& boxes, float gap,
		std::vector<trimesh::vec3>& newPosition)
	{
		trimesh::box3 _box = box;
		_box.min += trimesh::vec3(gap, gap, 0.0f);
		_box.max += trimesh::vec3(-gap, -gap, 0.0f);

		std::vector<trimesh::box3> _boxes;
		for (const trimesh::box3& b : boxes)
		{
			trimesh::box3 _b = b;
			_b.min += trimesh::vec3(-gap, -gap, 0.0f);
			_b.max += trimesh::vec3(gap, gap, 0.0f);
			_boxes.push_back(_b);
		}

		sortBoxes(_box, _boxes, newPosition);
	}

	void sortBoxes(const trimesh::box3& box, const std::vector<trimesh::box3>& boxes,
		std::vector<trimesh::vec3>& newPosition)
	{
		int size = (int)boxes.size();
		if (size > 0)
		{
			newPosition.resize(size);
			std::vector<int> indices;
			for (int i = 0; i < size; ++i)
				indices.push_back(i);

			//sort
			std::sort(indices.begin(), indices.end(), [&boxes](int i, int j)->bool {
				return boxes.at(i).size().y > boxes.at(j).size().y;
				});

			//insert
			std::vector<std::list<trimesh::box3>> spaces;
			spaces.reserve(100);
			spaces.resize(1);
			spaces.at(0).push_back(box);

			int index = 1;
			auto containFunc = [](const trimesh::box& box, const trimesh::box& b)->bool {
				return (box.size().x >= b.size().x) && (box.size().y >= b.size().y);
			};
			auto findPosInList = [&containFunc](std::list<trimesh::box3>& lists, const trimesh::box3& b,
				trimesh::vec3& pos)->bool {
				if (lists.size() == 1 && !containFunc(lists.front(), b))
				{
					pos = lists.front().min + b.size() / 2.0f;
					lists.clear();
					return true;
				}

				std::list<trimesh::box3>::iterator it = lists.begin();
				for (; it != lists.end(); ++it)
				{
					if (containFunc(*it, b))
						break;
				}

				if (it != lists.end())
				{//find
					trimesh::box3 rbox = *it;
					trimesh::box3 b1 = rbox;
					b1.min.x += b.size().x;
					b1.max.y = b1.min.y + b.size().y;
					trimesh::box3 b2 = rbox;
					b2.min.y += b.size().y;
					lists.erase(it);
					lists.push_back(b1);
					lists.push_back(b2);
					pos = rbox.min + b.size() / 2.0f;
					return true;
				}
				return false;
			};

			auto findPos = [&index, &box, &findPosInList, &spaces](const trimesh::box3& b)->trimesh::vec3 {
				bool findOne = false;
				trimesh::vec3 newPos;
				for (size_t i = 0; i < spaces.size(); ++i)
				{
					findOne = findPosInList(spaces.at(i), b, newPos);
					if (findOne)
						break;
				}

				if (findOne)
					return newPos;

				std::list<trimesh::box3> newSpace;
				trimesh::box3 newBox = box;
				newBox.min += trimesh::vec3(index * box.size().x, 0.0f, 0.0f);
				newBox.max += trimesh::vec3(index * box.size().x, 0.0f, 0.0f);
				newSpace.push_back(newBox);

				findOne = findPosInList(newSpace, b, newPos);
				assert(findOne);
				return newPos;
			};

			for (int i = 0; i < size; ++i)
			{
				int iindex = indices.at(i);
				const trimesh::box& b = boxes.at(iindex);
				trimesh::vec3 pos = findPos(b);
				pos.z = 0.0f;
				newPosition.at(iindex) = pos;
			}
		}
	}
}