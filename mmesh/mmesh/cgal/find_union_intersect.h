// UNION_INTERSECTION.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//


#include<iostream>
#include<string>
#include<cstdlib>
#include<vector>
#ifdef USE_CGAL
#include "compute_normals_sm.h"

#if 0

namespace UNION_FIND_FUNC
{
	using namespace std;
	using namespace ComputeNormalsSM;
	typedef struct setNode {
		face_descriptor key;
		int rank;
		setNode* parent;
		setNode(face_descriptor k) :key(k), rank(0), parent(NULL) {}
	}setNode;

	typedef struct Set {
		setNode* root;
	}Set;

	typedef struct edge {
		face_descriptor u;
		face_descriptor v;
		edge(face_descriptor value1, face_descriptor value2) :u(value1), v(value2) {}
	}edge;

	setNode* Make_Set(face_descriptor k);

	setNode* Find_Set(setNode* x);
	void Set_Union(setNode* x, setNode* y);
	void forestSet_Create(Set forestSet[], vector<face_descriptor> vertex);
	void Compute_conComponents(Set forestSet[], vector<edge>edgeArray);

	void Print_conComponents(Set forestSet[], vector<face_descriptor> vertex);
	int main();
	//void Print_conComponents(Set forestSet[], char vertex[], int vNum);
}

#endif

#endif