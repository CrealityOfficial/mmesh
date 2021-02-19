#include "SupportTree.h"
#include <cmath>
#include <math.h>

#ifndef PI 
#define PI 3.141596
#endif
#define SUPPORT_TREE_ANGLE_THRES 45.0*PI/180.0
#define SUPPORT_VERTEX_BETWEEN_LENMIN 0.05  //2 mm
#define SUPPORT_VERTEX_BETWEEN_LENMAX 2.0  //2 mm

TreeNode* InsertNodeToSortTree(TreeNode** mainnode, TreeNode* addnode)
{
	TreeNode * innernode= *mainnode;
	if (addnode == NULL)
		return NULL;
	if (innernode == NULL)  // 
	{
		// 初始化新插入的结点
		*mainnode = (TreeNode*)new TreeNode();
		(*mainnode)->treenodetype = TreeNodeEnum::ROOT_SUPPORT;
		(*mainnode)->child = addnode;
	}
	else // 如果树不为空
	{
		if (innernode->child != NULL) // 需要叶结点不为空
			innernode->child = InsertNodeToSortTree(&(innernode->child), addnode);
		else
			innernode->child = addnode;
	}
	return innernode;
}
void DestroyTreeNode(TreeNode* pRoot)
{
	if (pRoot != NULL)
	{
		while (pRoot->child !=NULL)
		{
			DestroyTreeNode(pRoot->child);
			pRoot->child = NULL;
		}
		delete pRoot;
		pRoot = NULL;
	}
}
static unsigned int NodeNumInner = 0;
unsigned int CalculateNodeNumInner(TreeNode* nodeptr)
{
	if (nodeptr != NULL)
	{
		if (nodeptr->child != NULL)
			NodeNumInner++;
		NodeNumInner = CalculateNodeNumInner(nodeptr->child);
	}
	return NodeNumInner;
}

unsigned int CalculateNodeNum(TreeNode* nodeptr)
{
	 unsigned int num = 0;
	 NodeNumInner=0;
	if (nodeptr != NULL)
	{
		if (nodeptr->child != NULL)
			num++;
		num = CalculateNodeNumInner(nodeptr->child);
	}
	return num;
}
TreeNodeEnum getNodeTreePoint(TreeNode* nodeptr, std::vector<trimesh::vec3> &points)
{
	static unsigned int num = 0;
	TreeNodeEnum treenodetype = TreeNodeEnum::UNKNOW_SUPPORT;
	if (nodeptr != NULL)
	{
		switch (nodeptr->treenodetype)
		{
			case TreeNodeEnum::BRANCH_SUPPORT:
				points.emplace_back(nodeptr->points[0]);
				points.emplace_back(nodeptr->points[2]);
				points.emplace_back(nodeptr->points[1]);
				points.emplace_back(nodeptr->points[2]);
				treenodetype = TreeNodeEnum::BRANCH_SUPPORT;
				break;
			case TreeNodeEnum::MAIN_SUPPORT:
				points.emplace_back(nodeptr->points[0]);
				points.emplace_back(nodeptr->points[1]);
				treenodetype = TreeNodeEnum::MAIN_SUPPORT;
				break;
			case TreeNodeEnum::ROOT_SUPPORT:
				treenodetype = TreeNodeEnum::ROOT_SUPPORT;
				break;
			default :
				std::cout << "unknow  tree node type" << std::endl;
		}
	}
	return treenodetype;
}
void getNodeTreeALLPoint(TreeNode* nodeptr, std::vector<trimesh::vec3>& points)
{
	static unsigned int num = 0;
	TreeNode* nodeptrtemp = nodeptr;
	while (nodeptrtemp != NULL)
	{
		getNodeTreePoint(nodeptrtemp, points);
		nodeptrtemp = nodeptrtemp->child;
	}
}
SupportTreeObj::SupportTreeObj()
{

}

SupportTreeObj::~SupportTreeObj()
{
	for (int index=0;index<m_TreeContainer.size();index++)
	{

		DestroyTreeNode(m_TreeContainer[index]);
	}
	m_TreeContainer.clear();
}
bool  SupportTreeObj::calculateCrossPoint(trimesh::vec3 point1, trimesh::vec3 point2, trimesh::vec3 &crossPoint)
{
	bool retvalue = false;
	float tanAngle = tanf(SUPPORT_TREE_ANGLE_THRES);
	float deltX = point2.x - point1.x;
	float deltY = point2.y - point1.y;
	float deltZ = point2.z - point1.z;
	float deltXYlen = sqrtf(deltX * deltX + deltY * deltY);
	float r1 = point1.z* tanAngle;
	float r2 = point2.z* tanAngle;
	float tangAngleTest = deltXYlen/ deltZ;//测试低点圆锥是不是在内部
	if ((deltXYlen < (r1 + r2))&&(r1>0)&&(r2>0))//确保两圆锥有交点或在内部
	{
		if (tangAngleTest >= tanAngle)
		{
			crossPoint.z = (point1.z + point2.z - deltXYlen / tanAngle) / 2.0;
			r1 = (point1.z - crossPoint.z) * tanAngle;
			r2 = (point2.z - crossPoint.z) * tanAngle;
			crossPoint.x = point1.x + r1 * deltX / (r1 + r2);
			crossPoint.y = point1.y + r1 * deltY / (r1 + r2);
			retvalue = true;
		}
		else//低点圆锥在内部
		{
			crossPoint = point1 ;
			retvalue = true;

		}
	}
	return retvalue;
}

void SupportTreeObj::createTreeContainer(std::vector < TreeNode*> &treeContainer,std::vector<trimesh::vec3> points)
{
	TreeNode* treeContainerPtr=NULL;
	std::vector<trimesh::vec3> pointcircle = points;
	std::vector<bool> pointAvalible;
	bool firstEnterflg = true;
	int indexCircle = 0;
	int indexCircletimes = 0;
	int indexCircleMax = pointcircle.size();
	pointAvalible.resize(indexCircleMax, true);
	//for (indexCircle=0; indexCircle< indexCircleMax; indexCircle++)
	while (1)
	{
		trimesh::vec3 PointZmax(0.0, 0.0, -1000.0);
		trimesh::vec3 PointZmin(0.0, 0.0, 0.0);
		float distanceMin = SUPPORT_VERTEX_BETWEEN_LENMAX+1.0;
		int indexPointZmax = -1;
		int indexPointNearZmax = -1;
		//搜索最高点
		for (int index = 0; index < indexCircleMax; index++)
		{
			if (pointAvalible[index] == false)//标识该顶点已用完
				continue;
			if (pointcircle[index].z > PointZmax.z)
			{
				PointZmax = pointcircle[index];
				indexPointZmax = index;
			}
		}
			//搜索最近点
		for (int index = 0; index < indexCircleMax; index++)
		{
			if (pointAvalible[index] == false)//标识该顶点已用完
				continue;
			{
				float deltX = 0.0;
				float deltY = 0.0;
				float deltZ = 0.0;
				float deltXYZlen = 0.0;
				if (indexPointZmax == index)//排除最高点项
					continue;
				deltX = PointZmax.x - pointcircle[index].x;
				deltY = PointZmax.y - pointcircle[index].y;
				deltZ = PointZmax.z - pointcircle[index].z;
				//deltXYZlen = deltX * deltX + deltY * deltY + deltZ * deltZ;
				deltXYZlen = deltX * deltX + deltY * deltY;

				if (deltXYZlen < distanceMin)
				{
					distanceMin = deltXYZlen;
					PointZmin = pointcircle[index];
					indexPointNearZmax = index;
				}
			}
		}
		if ( indexPointNearZmax == -1)
		{
			{
				std::cout << "the last point" << std::endl;
				TreeNode* child = (TreeNode*) new TreeNode;
				child->points.emplace_back(PointZmax);
				child->points.emplace_back(trimesh::vec3(PointZmax.x, PointZmax.y, 0.0));
				child->treenodetype = TreeNodeEnum::MAIN_SUPPORT;
				child->child = NULL;
				InsertNodeToSortTree(&treeContainerPtr, child);
				treeContainer.emplace_back(treeContainerPtr);
				//////////////////////////
				treeContainerPtr = NULL;


			}
			break;
		}
		if (PointZmin.z > PointZmax.z)
		{
			trimesh::vec3  PointZminmax = PointZmax;
			int indexPointZmaxmin = indexPointZmax;
			PointZmax = PointZmin;
			indexPointZmax = indexPointNearZmax;
			PointZmin = PointZmax;
			indexPointNearZmax = indexPointZmaxmin;
		}

		//if ((distanceMin > SUPPORT_VERTEX_BETWEEN_LENMIN)&& (distanceMin < SUPPORT_VERTEX_BETWEEN_LENMAX))
		if (distanceMin < SUPPORT_VERTEX_BETWEEN_LENMAX)
		{
			trimesh::vec3 crossPoint(0.0, 0.0, 0.0);
			bool avalible = calculateCrossPoint(PointZmin, PointZmax, crossPoint);
			if (avalible)
			{
				//TreeNode* child = (TreeNode*)malloc(sizeof(TreeNode));
				TreeNode* child = (TreeNode*) new TreeNode;
				child->points.emplace_back(PointZmax);
				if (crossPoint == PointZmin)
				{
					child->points.emplace_back(PointZmin);
					//pointAvalible[indexPointNearZmax] = false;
					std::cout << "contain point inner" << std::endl;
					child->treenodetype = TreeNodeEnum::MAIN_SUPPORT;
				}
				else
				{ 
					child->points.emplace_back(PointZmin);
					child->points.emplace_back(crossPoint);
					pointcircle[indexPointNearZmax]= crossPoint;
					child->treenodetype = TreeNodeEnum::BRANCH_SUPPORT;
				}
				///
				child->child = NULL;
				InsertNodeToSortTree(&treeContainerPtr, child);
				pointAvalible[indexPointZmax] = false;
				///////

			}
			else
			{
				std::cout << "no cross point" << std::endl;
				TreeNode* child = (TreeNode*) new TreeNode;
				child->points.emplace_back(PointZmax);
				child->points.emplace_back(trimesh::vec3(PointZmax.x, PointZmax.y, 0.0));
				child->treenodetype = TreeNodeEnum::MAIN_SUPPORT;
				child->child = NULL;
				InsertNodeToSortTree(&treeContainerPtr, child);
				treeContainer.emplace_back(treeContainerPtr);
				//////////////////////////
				pointAvalible[indexPointZmax] = false;
				treeContainerPtr = NULL;


			}
		}
		else 
		{
			std::cout << "distanceMin out of range" << std::endl;

			TreeNode* child = (TreeNode*) new TreeNode;
			child->points.emplace_back(PointZmax);
			child->points.emplace_back(trimesh::vec3(PointZmax.x, PointZmax.y, 0.0));
			child->treenodetype = TreeNodeEnum::MAIN_SUPPORT;
			child->child = NULL;
			InsertNodeToSortTree(&treeContainerPtr, child);
			pointAvalible[indexPointZmax] = false;
			treeContainer.emplace_back(treeContainerPtr);
			treeContainerPtr = NULL;
			//////////////////////////


		}
		//else
		//{
		//	pointAvalible[indexPointZmax] = false;
		//}
	};
}
