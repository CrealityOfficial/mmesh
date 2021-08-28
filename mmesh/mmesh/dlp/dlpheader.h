#ifndef MMESH_DLPHEADER_1613700735852_H
#define MMESH_DLPHEADER_1613700735852_H
#include "trimesh2/Vec.h"
#include "mmesh/trimesh/Clustering.h"

namespace mmesh
{
	struct VerticalC
	{
		trimesh::vec3 position;
		bool platform;
		int indexCells;
	};
	struct DLPISource
	{
		trimesh::vec3 position;
		trimesh::vec3 normal;
		int typeflg;
		struct VerticalC posHit;
#ifdef CX_BOOST_CLUSTER
		ClusteredPoints clusteredPts;
#endif
	};
	struct DLPISources
	{
		std::vector<DLPISource> sources;
#ifdef CX_BOOST_CLUSTER
		ClusteredPoints clusteredSrcID;
#endif
	};


	struct DLPSupportParam
	{
		//top
		int topContactType;
		float topContactRadius;
		float topContactDepth;
		int topLinkType;
		float topLinkTRadius;
		float topLinkBRadius;
		float topLinkLen;

		//middle
		float middleRadius;

		//bottom
		float bottomHeight;
		float bottomTRadius;
		float bottomBRadius;

		//model contact
		float modelContactLen;

		//create info
		trimesh::vec3 top;
		trimesh::vec3 middle;
		trimesh::vec3 bottom;

		bool platform;

		std::string RaftType;
		float RaftRatio;
		float RaftHeight;
		float RaftSize;

		float TipDiameter;

		trimesh::vec3 visualTop;
		trimesh::vec3 visualMiddle;
		trimesh::vec3 visualBottom;
	};

	struct DLPCrossLinkInfo
	{
		trimesh::vec3 start;
		trimesh::vec3 end;
		float radius;

		trimesh::vec3 visualStart;
		trimesh::vec3 visualEnd;
	};

	struct AutoDLPSupportParam
	{
		bool platform;
		float density;
		float autoAngle;
		float supportlength;
		float startheight;
		float manualAngle;
		float space;
		float baseSpace;

	};
}

#endif // MMESH_DLPHEADER_1613700735852_H