#include "SupportTree.h"
#ifndef PI 
#define PI 3.141596
#endif
#define SUPPORT_TREE_ANGLE_THRES 45.0*PI/180.0
#define SUPPORT_VERTEX_BETWEEN_LENMIN 0.4  //2 mm
#define SUPPORT_VERTEX_BETWEEN_LENMAX 10.0  //2 mm

TreeNode* InsertNodeToSortTree(TreeNode** mainnode, TreeNode* addnode)
{
	TreeNode * innernode= *mainnode;
	if (addnode == NULL)
		return NULL;
	if (innernode == NULL)  // 
	{
		// 初始化新插入的结点
		*mainnode = (TreeNode*)new TreeNode();
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
unsigned int CalculateNodeNum(TreeNode* nodeptr)
{
	static unsigned int num = 0;
	if (nodeptr != NULL)
	{
		if (nodeptr->child != NULL)
			num++;
		num = CalculateNodeNum(nodeptr->child);
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
	float tanAngle = std::tanf(SUPPORT_TREE_ANGLE_THRES);
	float deltX = point2.x - point1.x;
	float deltY = point2.y - point1.y;
	float deltZ = point2.z - point1.z;
	float deltXYlen = std::sqrtf(deltX * deltX + deltY * deltY);
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
	while (1)
	{
		trimesh::vec3 PointZmax(0.0, 0.0, -1000.0);
		trimesh::vec3 PointZmin(0.0, 0.0, 0.0);
		float distanceMin = SUPPORT_VERTEX_BETWEEN_LENMAX;
		int indexPointZmax = -1;
		int indexPointNearZmax = -1;

		for (int index = 0; index < pointcircle.size(); index++)
		{
			if (firstEnterflg)
			{
				pointAvalible.push_back(true);
			}
			else if (pointAvalible[index] == false)//标识该顶点已用完
				continue;
			if (pointcircle[index].z > PointZmax.z)
			{
				PointZmax= pointcircle[index];
				indexPointZmax = index;
			}
		}
		firstEnterflg = false;
		if (indexPointZmax == -1)
			break;
		for (int index = 0; index < pointcircle.size(); index++)
		{
			float deltX			= 0.0;
			float deltY			= 0.0;
			float deltZ			=0.0;
			float deltXYZlen	= 0.0;
			if (indexPointZmax == index)//排除自已项
				continue;
			if (pointAvalible[index] == false)//标识该顶点已用完
				continue;
			 deltX = PointZmax.x - pointcircle[index].x;
			 deltY = PointZmax.y - pointcircle[index].y;
			 deltZ = PointZmax.z - pointcircle[index].z;
			 deltXYZlen = deltX * deltX + deltY * deltY + deltZ * deltZ;

			if (deltXYZlen < distanceMin)
			{
				distanceMin = deltXYZlen;
				PointZmin = pointcircle[index];
				indexPointNearZmax = index;
			}
		}
		if ((distanceMin > SUPPORT_VERTEX_BETWEEN_LENMIN)&& (distanceMin < SUPPORT_VERTEX_BETWEEN_LENMAX))
		{
			trimesh::vec3 crossPoint(0.0, 0.0, 0.0);
			bool avalible = calculateCrossPoint(PointZmin, PointZmax, crossPoint);
			if (avalible)
			{
				//TreeNode* child = (TreeNode*)malloc(sizeof(TreeNode));
				TreeNode* child = (TreeNode*) new TreeNode;
				child->points.emplace_back(PointZmax);
				child->points.emplace_back(PointZmin);
				child->points.emplace_back(crossPoint);
				if(crossPoint== PointZmin)
				child->treenodetype = TreeNodeEnum::MAIN_SUPPORT;
				else
				child->treenodetype = TreeNodeEnum::BRANCH_SUPPORT;
				///
				child->child = NULL;
				InsertNodeToSortTree(&treeContainerPtr, child);
				///////
				pointAvalible[indexPointZmax] = false;
				pointAvalible[indexPointNearZmax] = false;
				pointAvalible.emplace_back(true);
				pointcircle.emplace_back(crossPoint);

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
				//////////////////////////
				pointAvalible[indexPointZmax] = false;
				treeContainer.emplace_back(treeContainerPtr);
				treeContainerPtr = NULL;


			}
		}
		else if(distanceMin >= SUPPORT_VERTEX_BETWEEN_LENMAX|| (indexPointZmax == pointcircle.size()-1))//相距比较远的点单独生成支撑点，或最后一个点生成主支撑
		{
			TreeNode* child = (TreeNode*) new TreeNode;
			child->points.emplace_back(PointZmax);
			child->points.emplace_back(trimesh::vec3(PointZmax.x, PointZmax.y, 0.0));
			child->treenodetype = TreeNodeEnum::MAIN_SUPPORT;
			child->child = NULL;
			InsertNodeToSortTree(&treeContainerPtr, child);
			//////////////////////////
			pointAvalible[indexPointZmax] = false;
			treeContainer.emplace_back(treeContainerPtr);
			treeContainerPtr = NULL;

		}
		else
		{
			pointAvalible[indexPointZmax] = false;
		}
	};
	unsigned int nodnumber=CalculateNodeNum(treeContainer[0]);
}
