#include "dumplicate.h"
#include "ccglobal/tracer.h"

#include <unordered_map>
#include <thread>

#include "ccglobal/spycc.h"

#include <ctime>

namespace mmesh
{

	struct accumulate_block {
		void operator () (std::vector<trimesh::point>::iterator vStart, std::vector<trimesh::point>::iterator vEnd, std::vector<int>& result) {
			result.clear();
			auto iter = vStart;
			while (iter != vEnd)
			{
				result.push_back(abs(iter->x) * 10000 / 23 + abs(iter->y) * 10000 / 19 + abs(iter->z) * 10000 / 17);
				iter++;
			}
		}
	};


	void multithreadCalc(std::vector<trimesh::point>& vertices, std::vector<int>& result) {
		const long length = distance(vertices.begin(), vertices.end());
		if (length == 0) {
			return;
		}
		unsigned long const min_per_thread = 25;
		unsigned long const max_threads = (length + min_per_thread - 1) / min_per_thread;
		unsigned long const hardware_threads = std::thread::hardware_concurrency();
		unsigned long const num_threads = std::min(hardware_threads != 0 ? hardware_threads : 2, max_threads);
		unsigned long const block_size = length / num_threads;
		std::vector<std::thread> threads(num_threads - 1);
		std::vector<std::vector<int>> results(num_threads);
		std::vector<trimesh::point>::iterator block_start = vertices.begin();
		for (int i = 0; i < (num_threads - 1); i++) {
			auto block_end = block_start;
			advance(block_end, block_size);
			threads[i] = std::thread(accumulate_block(), block_start, block_end, std::ref(results[i]));
			block_start = block_end;//更新序列位置  
		}
		accumulate_block mainAccu;
		mainAccu(block_start, vertices.end(), results[num_threads - 1]);
		std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));//等待子线程完成  

		for (size_t i = 0; i < results.size(); i++)
		{
			result.insert(result.end(), results[i].begin(), results[i].end());
		}
		//return std::accumulate(results.begin(), results.end(), init);

	}

	void qDumplicateMesh(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
	{
		std::clock_t start = clock();

		//ANALYSIS_START("start DumplicateMesh");
			//trimesh::TriMesh* mesh = m_meshLoadInfo->m_mesh;
			size_t vertexNum = mesh->vertices.size();

			typedef std::unordered_map<int, int> unique_point;
			unique_point points(vertexNum * 3 / 10 + 1);

			typedef unique_point::iterator point_iterator;

			size_t faceNum = mesh->faces.size();

			if (vertexNum == 0 || faceNum == 0)
				return;

			trimesh::TriMesh* m_optimizedMesh = new trimesh::TriMesh();


			std::vector<int> vertexMapper;
			vertexMapper.resize(vertexNum, -1);

			bool interupt = false;

			//ANALYSIS_TICK("start multithreadCalc");
			//std::vector<int> vertexTo(vertexNum, -1);
			//for (size_t i = 0; i < vertexNum; ++i)
			//{
			//	vertexTo[i] = abs(mesh->vertices.at(i).x) * 10000 / 23 + abs(mesh->vertices.at(i).y) * 10000 / 19 + abs(mesh->vertices.at(i).z) * 10000 / 17;
			//}
			std::vector<int> vertexTo;
			multithreadCalc(mesh->vertices, vertexTo);
			//ANALYSIS_TICK("end multithreadCalc");

			//ANALYSIS_TICK("start vertexNum");
			for (size_t i = 0; i < vertexNum; ++i)
			{
				int p = vertexTo.at(i);
				point_iterator it = points.find(p);
				if (it != points.end())
				{
					int index = (*it).second;
					vertexMapper.at(i) = index;
				}
				else
				{
					int index = (int)points.size();
					points.insert(unique_point::value_type(p, index));

					vertexMapper.at(i) = index;
				}
			}

			//ANALYSIS_TICK("end vertexNum");

			trimesh::TriMesh* omesh = m_optimizedMesh;
			omesh->vertices.resize(points.size());
			for (point_iterator it = points.begin(); it != points.end(); ++it)
			{
				omesh->vertices.at(it->second) = it->first;
			}

			//ANALYSIS_TICK("start faceNum");
			omesh->faces.swap(mesh->faces);
			for (size_t i = 0; i < faceNum; ++i)
			{
				trimesh::TriMesh::Face& of = omesh->faces.at(i);
				for (int j = 0; j < 3; ++j)
				{
					int index = of[j];
					of[j] = vertexMapper[index];
				}
			}
			//ANALYSIS_TICK("end faceNum");
			mesh->vertices.swap(omesh->vertices);
			mesh->faces.swap(omesh->faces);
			mesh->need_bbox();

			//ANALYSIS_TICK("end");
			//ANALYSIS_DUMP("dumplicate.csv");
			delete omesh;

			clock_t end = clock();
			double endtime = (double)(end - start);
			std::printf("total time: %f\n", endtime);
	}
}