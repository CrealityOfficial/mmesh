#include "split.h"


namespace mmesh
{

	//����Ͷ�Ӧ�Ķ������ӵ�mesh ��
	 void addFaceVertices(trimesh::TriMesh* mesh, const trimesh::vec3& v1, const trimesh::vec3& v2, const trimesh::vec3& v3)
	 {
		int index = (int)mesh->vertices.size();
		mesh->vertices.push_back(v1);
		mesh->vertices.push_back(v2);
		mesh->vertices.push_back(v3);
		mesh->faces.push_back(trimesh::TriMesh::Face(index, index + 1, index + 2));
	};




	void fcollid(std::vector<trimesh::vec3>& lines,trimesh::TriMesh* inputMesh, std::vector<float>& distances,
										trimesh::TriMesh* meshUP, trimesh::TriMesh* meshDown,
										int Point0Index, int Point1Index, int Point2Index)
	{
		trimesh::vec3& Point0 = inputMesh->vertices.at(Point0Index);
		trimesh::vec3& Point1 = inputMesh->vertices.at(Point1Index);
		trimesh::vec3& Point2 = inputMesh->vertices.at(Point2Index);
		float dv = distances.at(Point0Index);//
		float d1 = distances.at(Point1Index);
		float d2 = distances.at(Point2Index);
		if (d1 == 0.0f && d2 == 0.0f)//�߽��ж� d1,d2λ���и���ʱ
		{
			if (dv >= 0.0f)//dv�����и����
			{
				addFaceVertices(meshUP, Point0, Point1, Point2);//�ָ������������ӵ���Ӧ��mesh
				lines.push_back(Point1);
				lines.push_back(Point2);
			}
			else//dvС���и��樌
			{
				addFaceVertices(meshDown, Point0, Point1, Point2);
				lines.push_back(Point2);
				lines.push_back(Point1);
			}
		}
		else if (d1 == 0.0f && dv * d2 >= 0.0f)
		{
			if (dv >= 0.0f)//d1λ���и�����dv d2λ���и�������
				addFaceVertices(meshUP, Point0, Point1, Point2);
			else//d1λ���и�����dv d2λ���и�������
				addFaceVertices(meshDown, Point0, Point1, Point2);
		}
		else if (d2 == 0.0f && dv * d1 >= 0.0f)
		{
			if (dv >= 0.0f)//d2λ���и�����dv d1λ���и�������
				addFaceVertices(meshUP, Point0, Point1, Point2);
			else//d2λ���и�����dv d1λ���и�������
				addFaceVertices(meshDown, Point0, Point1, Point2);
		}
		else//�ų����еĵ㴦���и��������� �����и����
		{
			//c1 c2 �����汻�и�����ɵĵ�
			trimesh::vec3 c1 = (dv / (dv - d1)) * Point1 - (d1 / (dv - d1)) * Point0;
			trimesh::vec3 c2 = (dv / (dv - d2)) * Point2 - (d2 / (dv - d2)) * Point0;

#ifdef _DEBUG
			if (c1.x < inputMesh->bbox.min.x || c1.x > inputMesh->bbox.max.x || c1.y < inputMesh->bbox.min.y || c1.y > inputMesh->bbox.max.y
				|| c2.x < inputMesh->bbox.min.x || c2.x > inputMesh->bbox.max.x || c2.y < inputMesh->bbox.min.y || c2.y > inputMesh->bbox.max.y)
			{
				printf("error");
			}
#endif
			if (dv > 0.0f)//������
			{
				addFaceVertices(meshUP, Point0, c1, c2);
				addFaceVertices(meshDown, c2, c1, Point2);
				addFaceVertices(meshDown, c1, Point1, Point2);

				lines.push_back(c1);
				lines.push_back(c2);
			}
			else if (dv < 0.0f)//������ 
			{
				addFaceVertices(meshDown, Point0, c1, c2);
				addFaceVertices(meshUP, c2, c1, Point2);
				addFaceVertices(meshUP, c1, Point1, Point2);

				lines.push_back(c2);
				lines.push_back(c1);
			}
			else//dv ==0,��ʱ����Ҫ����lines
			{
				if (d1 > 0.0f)
				{
					addFaceVertices(meshUP, Point0, Point1, Point2);
				}
				else if (d1 < 0.0f)
				{
					addFaceVertices(meshDown, Point0, Point1, Point2);
				}
			}
		}
	}

	//���ص����������û��aPoint�������ӵ�aPoint��Ȼ�󷵻�����
	int generateIndex(unique_point& points, const trimesh::vec3& aPoint)
	{
		int index = -1;
		point_iterator it = points.find(aPoint);
		if (it != points.end())
		{
			index = (*it).second;
		}
		else
		{
			index = (int)points.size();
			points.insert(unique_point::value_type(aPoint, index));
		}
		return index;
	}

	int check(std::vector<bool>& used)
	{
		int index = -1;
		size_t size = used.size();
		for (size_t i = 0; i < size; ++i)
		{
			if (!used.at(i))
			{
				index = (int)i;
				break;
			}
		}
		return index;
	}


	void lines2polygon(std::vector<trimesh::vec3>& lines, std::vector<std::vector<int>>& polygons, std::vector<trimesh::vec3>& uniPoints)
	{
		size_t segsize = lines.size() / 2;//�߶�����
		std::vector<segment> segments(segsize);//segments �߶εĵ�����
		unique_point points;// firtst ������ second ������
		for (size_t i = 0; i < segsize; ++i)
		{
			trimesh::vec3 v1 = lines.at(2 * i);//2 * i==0 2 4
			trimesh::vec3 v2 = lines.at(2 * i + 1);//2 * i + 1==1 3 5
			segments.at(i).start = generateIndex(points,v1);
			segments.at(i).end = generateIndex(points, v2);
		}

		std::vector<trimesh::vec3> vecpoints(points.size());
		for (auto it = points.begin(); it != points.end(); ++it)
		{
			vecpoints.at((*it).second) = (*it).first;//vecpoints ��������˳��洢������
		}
		std::vector<segment*> segmap(points.size(), nullptr);
		for (segment& s : segments)
		{
			segmap.at(s.start) = &s;//segmap �� segments��start ��������� 
		}
		std::vector<bool> used(points.size(), false);
		std::vector<IndexPolygon> indexPolygons;
		int index = check(used);
		while (index >= 0)//���ζ����е��߶ν�������
		{
			used.at(index) = true;
			segment* seg = segmap.at(index);
			if (seg)
			{
				int s = seg->start;
				int e = seg->end;

				bool find = false;
				for (IndexPolygon& polygon : indexPolygons)
				{
					if (s == polygon.end)//����ͷ��
					{
						polygon.polygon.push_back(e);
						polygon.end = e;
						find = true;
					}
					else if (e == polygon.start)//����β��
					{
						polygon.polygon.push_front(s);
						polygon.start = s;
						find = true;
					}

					if (find)
						break;
				}

				if (!find)//û�з������ӵĵ㣬����������
				{
					IndexPolygon Ipolygon;
					Ipolygon.polygon.push_back(s);
					Ipolygon.polygon.push_back(e);
					Ipolygon.start = s;
					Ipolygon.end = e;
					indexPolygons.emplace_back(Ipolygon);
				}
			}
			index = check(used);
		}

		std::map<int, IndexPolygon*> IndexPolygonMap;
		for (size_t i = 0; i < indexPolygons.size(); ++i)
		{
			IndexPolygon& p1 = indexPolygons.at(i);
			if (!p1.closed())
				IndexPolygonMap.insert(std::pair<int, IndexPolygon*>(p1.start, &p1));
		}

		//combime ���ɱպ�����
		for (size_t i = 0; i < indexPolygons.size(); ++i)
		{
			IndexPolygon& p1 = indexPolygons.at(i);

			if (p1.polygon.size() == 0 || p1.closed())
				continue;

			auto it = IndexPolygonMap.find(p1.end);
			while (it != IndexPolygonMap.end())
			{

				IndexPolygon& p2 = *(*it).second;
				if (p2.polygon.size() == 0)
					break;

				bool merged = false;
				if (p1.start == p2.end)//p1��������ʼ�����p2������end�㣬��ϲ�2��������p1
				{
					p1.start = p2.start;
					for (auto iter = p2.polygon.rbegin(); iter != p2.polygon.rend(); ++iter)
					{
						if ((*iter) != p1.polygon.front()) p1.polygon.push_front((*iter));
					}
					merged = true;
				}
				else if (p1.end == p2.start)//p1������end�����p2��������ʼ�㣬��ϲ�2��������p1
				{
					p1.end = p2.end;
					for (auto iter = p2.polygon.begin(); iter != p2.polygon.end(); ++iter)
					{
						if ((*iter) != p1.polygon.back()) p1.polygon.push_back((*iter));
					}
					merged = true;
				}

				if (merged)
				{
					p2.polygon.clear();
				}
				else
					break;

				it = IndexPolygonMap.find(p1.end);
			}
		}


		size_t polygonSize = indexPolygons.size();
		if (polygonSize > 0)
		{
			polygons.reserve(polygonSize);
			for (size_t i = 0; i < polygonSize; ++i)
			{
				std::vector<int> polygon;
				IndexPolygon& ipolygon = indexPolygons.at(i);
				for (int iindex : ipolygon.polygon)
				{
					polygon.push_back(iindex);
				}

				if (polygon.size() > 0)
				{
					polygons.emplace_back(polygon);//polygons ���յõ�����������
				}
			}
		}
		uniPoints.swap(vecpoints);//uniPoints ���յõ�������
	}

	bool split(trimesh::TriMesh* inputMesh, float z, const trimesh::vec3& normal,
		trimesh::TriMesh** mesh1, trimesh::TriMesh** mesh2)
	{
		size_t vertex_size = inputMesh->vertices.size();
		if (vertex_size == 0)
			return false;

		trimesh::vec3 pos(0, 0, z);//�и���뷨��normal ��ͬ�����и���
		std::vector<float> distances;
		distances.resize(vertex_size);
#define min_value 1e-4
		bool allPositive = true;
		bool allNegtive = true;
		for (int i = 0; i < vertex_size; ++i)
		{
			trimesh::vec3 d = inputMesh->vertices.at(i) - pos;//����ģ�Ͷ���vertices���и�������
			distances.at(i) = normal.dot(d);//����d�뷨�ߵĵ�������������õ�λ���и������棬���������и�������

			if (distances.at(i) < -min_value)
				allPositive = false;
			if (distances.at(i) > min_value)
				allNegtive = false;
		}
		if (allPositive || allNegtive)//ȫ�����и�������������棬���˳�
			return false;

		std::vector<trimesh::TriMesh::Face> faceUp;//�и�������
		std::vector<trimesh::TriMesh::Face> faceDown; //�и�������
		std::vector<trimesh::TriMesh::Face> collideFaces;//�и���
		for (trimesh::TriMesh::Face& f : inputMesh->faces)
		{
			float Point0 = distances.at(f.x);//��Face��3������
			float Point1 = distances.at(f.y);
			float Point2 = distances.at(f.z);
			if (Point0 > 0.0f && Point1 > 0.0f && Point2 > 0.0f)
			{
				faceUp.push_back(f);//3���㶼����0�����λ���и�������
			}
			else if (Point0 < 0.0f && Point1 < 0.0f && Point2 < 0.0f)
			{
				faceDown.push_back(f);//3���㶼С��0�����λ���и�������
			}
			else
				collideFaces.push_back(f);//������汻�и���������������
		}
		trimesh::TriMesh* meshUP = new trimesh::TriMesh();
		trimesh::TriMesh* meshDown = new trimesh::TriMesh();
		*mesh1 = meshUP;
		*mesh2 = meshDown;

		FaceGenerateMesh(meshUP,inputMesh,faceUp);
		FaceGenerateMesh(meshDown, inputMesh, faceDown);

		std::vector<trimesh::vec3> lines;
		int faceNum = (int)inputMesh->faces.size();
		for (int i = 0; i < faceNum; ++i)
		{
			trimesh::TriMesh::Face& f = inputMesh->faces.at(i);
			//�߶δ���0��ʾ�߶εĵ�ֱ�λ���������У��߶�<=0 ��ʾ�߶�λ�ĵ����������������
			float segment0 = distances.at(f.x) * distances.at(f.y);
			float segment1 = distances.at(f.y) * distances.at(f.z);
			float segment2 = distances.at(f.x) * distances.at(f.z);
			if (segment0 == 0.0f && segment1 == 0.0f && segment2 == 0.0f)
				continue;

			if (segment0 >= 0.0f && (segment1 <= 0.0f || segment2 <= 0.0f))//f.z Ϊ����
			{
				fcollid(lines, inputMesh, distances,meshUP,meshDown,f.z, f.x, f.y);
			}
			else if (segment0 < 0.0f)
			{
				if (segment1 <= 0.0f)//f.y Ϊ����
				{
					fcollid(lines, inputMesh, distances, meshUP, meshDown, f.y, f.z, f.x);
				}
				else if (segment2 <= 0.0f)//f.x Ϊ����
				{
					fcollid(lines, inputMesh, distances, meshUP, meshDown, f.x, f.y, f.z);
				}
			}

		}

		//�ָ����lines�߶����ɱպ�������polygons����������points��������
		std::vector<std::vector<int>> polygons;
		std::vector<trimesh::vec3> points;
		lines2polygon(lines, polygons, points);

		std::vector<trimesh::dvec2> polygons2;
		polygons2.reserve(points.size());
		trimesh::vec3 zn = trimesh::vec3(0.0f, 0.0f, 1.0f);
		trimesh::vec3 axis = normal TRICROSS zn;
		float angle = trimesh::vv_angle(axis, zn);
		trimesh::xform r = trimesh::xform::rot((double)angle, axis);
		for (size_t i = 0; i < points.size(); ++i)
		{
			trimesh::vec3 v = points.at(i);
			trimesh::dvec3 dv = trimesh::dvec3(v.x, v.y, v.z);
			trimesh::dvec3 p = r * dv;
			polygons2.push_back(trimesh::dvec2(p.x, p.y));
		}


		//�պ��������ǻ����� faces
		std::vector<trimesh::TriMesh::Face> faces;
		mmesh::PolygonStack pstack;
		pstack.generates(polygons, polygons2, faces, 0);


		//�������ɵ�faces ���ӵ�meshUP��meshDown����2��mesh��
		bool fillHole = true;
		if (fillHole)
		{
			int start1 = (int)meshUP->vertices.size();
			int start2 = (int)meshDown->vertices.size();

			meshUP->vertices.insert(meshUP->vertices.end(), points.begin(), points.end());
			meshDown->vertices.insert(meshDown->vertices.end(), points.begin(), points.end());
			for (trimesh::TriMesh::Face& fs : faces)
			{
				trimesh::TriMesh::Face f1 = fs;
				f1 += trimesh::ivec3(start1, start1, start1);
				int t = f1[2];
				f1[2] = f1[1];
				f1[1] = t;
				meshUP->faces.push_back(f1);

				trimesh::TriMesh::Face f2 = fs;
				f2 += trimesh::ivec3(start2, start2, start2);
				meshDown->faces.push_back(f2);
			}
		}
		return true;
	}

	//face �� vetices ������mesh
	void FaceGenerateMesh(trimesh::TriMesh* newMesh, trimesh::TriMesh* inputMesh, std::vector<trimesh::TriMesh::Face>& inputface)
	{
		newMesh->faces.swap(inputface);//������
		int index = 0;
		for (trimesh::TriMesh::Face& f : newMesh->faces)
		{
			newMesh->vertices.push_back(inputMesh->vertices.at(f.x));//�������Ӧ�� 3������
			newMesh->vertices.push_back(inputMesh->vertices.at(f.y));
			newMesh->vertices.push_back(inputMesh->vertices.at(f.z));
			
			f.x = index++;//newMesh��������
			f.y = index++;
			f.z = index++;
		}
	}

}