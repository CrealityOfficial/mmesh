#ifndef __DLP_SUPPORTTREE_H__
#define __DLP_SUPPORTTREE_H__
#include "trimesh2/TriMesh.h"
#include <vector>
enum class TreeNodeEnum
{
	ROOT_SUPPORT,
	MAIN_SUPPORT,
	BRANCH_SUPPORT,
	UNKNOW_SUPPORT
};

typedef struct __TREE_NODE__
{
	std::vector<trimesh::vec3> points;   // Êý¾ÝÓò,start->end******Zmax Zmin Zcross
	TreeNodeEnum treenodetype;
	struct __TREE_NODE__* child;  // ×óÖ¸Õë
}TreeNode;

TreeNode* InsertNodeToSortTree(TreeNode** mainnode, TreeNode* addnode);
void DestroyTreeNode(TreeNode* pRoot);
unsigned int CalculateNodeNum(TreeNode* nodeptr);
TreeNodeEnum getNodeTreePoint(TreeNode* nodeptr, std::vector<trimesh::vec3>& points);
void getNodeTreeALLPoint(TreeNode* nodeptr, std::vector<trimesh::vec3>& points);
class SupportTreeObj
{
public:
	SupportTreeObj();
	~SupportTreeObj();
public:
	std::vector < TreeNode *> m_TreeContainer;
public:
	void createTreeContainer(std::vector < TreeNode*> &treeContainer, std::vector<trimesh::vec3> points);
	bool calculateCrossPoint(trimesh::vec3 point1, trimesh::vec3 point2,trimesh::vec3& crossPoint);

};
#endif // __DLP_SUPPORTTREE_H__
