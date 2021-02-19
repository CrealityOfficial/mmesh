#ifndef MMESH_DLPHEADER_1613700735852_H
#define MMESH_DLPHEADER_1613700735852_H
#include "trimesh2/Vec.h"

namespace mmesh
{
	struct DLPISource
	{
		trimesh::vec3 position;
		trimesh::vec3 normal;
	};

	struct VerticalC
	{
		trimesh::vec3 position;
		bool platform;
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
	};
}

#endif // MMESH_DLPHEADER_1613700735852_H