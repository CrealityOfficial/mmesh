// UNION_INTERSECTION.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#if 0

#include "find_union_intersect.h"
#ifdef USE_CGAL
namespace UNION_FIND_FUNC
{
	
	using namespace std;
	setNode* Make_Set(face_descriptor k)
	{
		setNode* x = new setNode(k);
		x->parent = x;
		return x;
	}

	setNode* Find_Set(setNode* x)
	{
		if (x != x->parent)
			x->parent = Find_Set(x->parent);
		return x->parent;
	}
	void Link(setNode* x, setNode* y)
	{
		if (x->rank > y->rank)
			y->parent = x;
		else
		{
			x->parent = y;
			if (x->rank == y->rank)
				y->rank = y->rank + 1;
		}
	}
	void Set_Union(setNode* x, setNode* y)
	{
		Link(Find_Set(x), Find_Set(y));
		setNode* z = Find_Set(x);
	}
	void forestSet_Create(Set forestSet[], vector<face_descriptor> vertex)
	{
		for (int i = 0; i < vertex.size(); i++)
		{
			face_descriptor index = vertex[i];                  //eg.a->97,b->98,...
			forestSet[index.idx()].root = Make_Set(vertex[i]);
		}
	}
	void Compute_conComponents(Set forestSet[], vector<edge>edgeArray)
	{//Compute the component forest
		for (int i = 0; i < edgeArray.size(); i++)
		{
			setNode* set_u = forestSet[edgeArray[i].u.idx()].root;
			setNode* set_v = forestSet[edgeArray[i].v.idx()].root;
			if (Find_Set(set_u) != Find_Set(set_v))
				Set_Union(set_u, set_v);
		}
	}

	void Print_conComponents(Set forestSet[], vector<face_descriptor> vertex)
	{//classify the forest and print the connect components and the representative
		vector<face_descriptor> representative;
		for (int i = 0; i < vertex.size(); i++) {
			setNode* t;
			t = Find_Set(forestSet[vertex[i].idx()].root);

			if (std::find(representative.begin(), representative.end(), t->key) == representative.end())     //the char t is not in representative 
				representative.push_back(t->key);
		}
		cout << "The representative of the forest:" << representative.size() << endl ;
		cout << "  value=="<<endl;
		for (int i = 0; i < representative.size(); i++)
		{
			cout << representative[i] << endl;
		}
		cout << endl;
		if (representative.size() > 0)
		{
			vector<face_descriptor>* ComponentsVec = new vector<face_descriptor>[representative.size()];
			for (int i = 0; i < vertex.size(); i++)
			{
				setNode* temp;
				temp = Find_Set(forestSet[vertex[i].idx()].root);
				vector<face_descriptor>::const_iterator index = std::find(representative.begin(), representative.end(), temp->key);
				int indexvalu = index - representative.begin();
				if (index < representative.end())
				{

					ComponentsVec[index - representative.begin()].emplace_back(vertex[i]);
				}
			}

			for (int i = 0; i < representative.size(); i++)
			{
				cout << "The connect component " << i + 1 << " is:";
				for (int j = 0; j < ComponentsVec[i].size(); j++)
					cout << ComponentsVec[i].at(j).idx() << "   ";
				cout << endl;
			}
		}
	}
	int main()
	{
		//vector<int>vertex;
		//vertex.emplace_back(1);
		//vertex.emplace_back(2);
		//vertex.emplace_back(3);
		//vertex.emplace_back(4);
		//vertex.emplace_back(5);
		//vertex.emplace_back(6);
		//vertex.emplace_back(7);
		//vertex.emplace_back(8);
		//vertex.emplace_back(9);
		//vertex.emplace_back(10);
		//vector<edge> edgeArray;
		//edgeArray.emplace_back(edge(1, 5));
		//edgeArray.emplace_back(edge(6, 7));
		//edgeArray.emplace_back(edge(7, 9));
		//edgeArray.emplace_back(edge(8, 10));
		//edgeArray.emplace_back(edge(2, 4));
		//int vNum = sizeof(vertex) / sizeof(int);
		//int eNum = sizeof(edgeArray) / sizeof(edge);
		//Set* forestSet = new Set[vertex.size()];

		//forestSet_Create(forestSet, vertex);           //Create forest set
		//Compute_conComponents(forestSet, edgeArray);   //Computing the component forest
		//Print_conComponents(forestSet, vertex);

		return 0;
	}
}
#endif

#endif