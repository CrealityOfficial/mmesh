#include "dlpsupportgenerator.h"
#include "mmesh/trimesh/trimeshutil.h"
#include "mmesh/trimesh/shapecreator.h"
#include "mmesh/dlp/dlpsimplequickdata.h"
#include <trimesh2/TriMesh_algo.h>

#include "mmesh/dlpcomponent/dlplink.h"
#include "mmesh/dlpcomponent/dlpmiddle.h"
#include "mmesh/dlpcomponent/dlpmodeltrunk.h"
#include "mmesh/dlpcomponent/dlpplatformtrunk.h"
#include "mmesh/dlpcomponent/dlptop.h"

namespace mmesh
{
	DLPTop* createTop(const trimesh::vec3& contactCenter, const trimesh::vec3& linkBottom, DLPSupportParam* supportParam)
	{
		TopParam topParam;
		topParam.contactCenter = contactCenter;
		topParam.topContactRadius = supportParam->topContactRadius;
		topParam.topContactDepth = supportParam->topContactDepth;
		topParam.linkBottom = linkBottom;
		topParam.topLinkBRadius = supportParam->topLinkBRadius;
		topParam.topLinkTRadius = supportParam->topLinkTRadius;

		return new DLPTop(topParam);
	}

	DLPMiddle* createMiddle(const trimesh::vec3& up, const trimesh::vec3& down, DLPSupportParam* supportParam)
	{
		MiddleParam middleParam;
		middleParam.up = up;
		middleParam.down = down;
		middleParam.radius = supportParam->middleRadius;

		return new DLPMiddle(middleParam);
	}

	DLPPlatformTrunk* createPlatformTrunk(const trimesh::vec3& bottomBottom, DLPSupportParam* supportParam)
	{
		PlatformTrunkParam platformTrunkParam;
		platformTrunkParam.bottomBottom = bottomBottom;
		platformTrunkParam.bottomHeight = supportParam->bottomHeight;
		platformTrunkParam.bottomBRadius = supportParam->bottomBRadius;
		platformTrunkParam.bottomTRadius = supportParam->bottomTRadius;

		return new DLPPlatformTrunk(platformTrunkParam);
	}

	DLPModelTrunk* createModelTrunk(const trimesh::vec3& bottomBottom, DLPSupportParam* supportParam)
	{
		ModelTrunkParam modelTrunkParam;
		modelTrunkParam.bottomBottom = bottomBottom;
		modelTrunkParam.middleRadius = supportParam->middleRadius;
		modelTrunkParam.bottomBRadius = supportParam->topLinkTRadius;
		modelTrunkParam.bottomHeight = supportParam->bottomHeight;

		return new DLPModelTrunk(modelTrunkParam);
	}

	DLPSupportGenerator::DLPSupportGenerator()
		:m_mesh(nullptr)
	{

	}

	DLPSupportGenerator::~DLPSupportGenerator()
	{

	}

	void DLPSupportGenerator::setInput(trimesh::TriMesh* mesh)
	{
		m_mesh = mesh;
	}

	void DLPSupportGenerator::setParam(const DLPSupportGeneratorParam& param)
	{
		m_param = param;
	}

	trimesh::TriMesh* DLPSupportGenerator::generateSupport(trimesh::TriMesh* mesh)
	{
		DLPSimpleQuickData quickData;
		quickData.setMeshData(mesh);

		AutoDLPSupportParam autoParam;
		autoParam.platform = false;
		autoParam.autoAngle = 50.0f;
		autoParam.manualAngle = 89.0f;
		autoParam.supportlength = 2.0f;
		autoParam.startheight = 3.0f;
		autoParam.density = 70.0f;
		autoParam.space = 3.0f;

		DLPSupportParam param;
		param.topContactType = 0;
		param.topContactRadius = 0.4f;
		param.topContactDepth = 0.0f;
		param.topLinkType = 0;
		param.topLinkTRadius = 0.2f;
		param.topLinkBRadius = 0.4f;
		param.topLinkLen = 1.0f;

		param.middleRadius = 0.4f;

		param.bottomHeight = 1.0f;
		param.bottomTRadius = 3.0f;
		param.bottomBRadius = 2.0f;

		param.modelContactLen = 2.0f;

		std::vector<DLPISource> sources;
		quickData.autoDlpSources(sources, &autoParam);

		std::vector<DLPMiddle*> middleComponents;
		auto createFromSource = [&autoParam, &param, &quickData, &middleComponents](DLPISource& source, std::vector<DLPComponent*>& components) {
			float len = param.topLinkLen;

			trimesh::vec3 top = source.position;
			trimesh::vec3 middle = top + len * source.normal;
			if (middle.z <= 1.0f) middle = top;

			trimesh::vec3 testPoint = (source.normal == trimesh::vec3(0.0f, 0.0f, -1.0f) ? top : middle);
			VerticalC vertical;
			if (quickData.check(vertical, testPoint)
				&& trimesh::len(vertical.position - top) > autoParam.supportlength)
			{
				trimesh::vec3 contactCenter = top;
				trimesh::vec3 linkBottom = middle;

				components.push_back(createTop(contactCenter, linkBottom, &param));

				if (vertical.platform)
				{
					trimesh::vec3 bottomBottom = vertical.position;
					components.push_back(createPlatformTrunk(bottomBottom, &param));

					trimesh::vec3 middleTop = linkBottom;
					trimesh::vec3 middleBottom = bottomBottom + trimesh::vec3(0.0f, 0.0f, param.bottomHeight);
					DLPMiddle* middleComponent = createMiddle(middleTop, bottomBottom, &param);
					middleComponent->setIsPlatform(true);

					components.push_back(middleComponent);
					middleComponents.push_back(middleComponent);
				}

				if (!vertical.platform)
				{
					trimesh::vec3 middleTop = linkBottom;
					trimesh::vec3 bottomBottom = vertical.position;
					components.push_back(createModelTrunk(bottomBottom, &param));

					trimesh::vec3 middleBottom = bottomBottom + trimesh::vec3(0.0f, 0.0f, param.bottomHeight);
					DLPMiddle* middleComponent = createMiddle(middleTop, middleBottom, &param);

					components.push_back(middleComponent);
					middleComponents.push_back(middleComponent);
				}
			}
		};
		auto GetProperMiddlesFromMiddle = [&middleComponents](DLPMiddle* dlpMiddle, std::vector<DLPMiddle*>& properMiddles) {
			const MiddleParam& param = dlpMiddle->param();
			trimesh::vec3 middle = param.up;
			trimesh::vec3 bottom = param.down;
			float supportLen = trimesh::len(middle - bottom);

			DLPMiddle* nearestMiddle = nullptr;
			float minLen = 1000.0f;
			for (DLPMiddle* supDLPMiddle : middleComponents)
			{
				const MiddleParam& supParam = supDLPMiddle->param();
				if (supDLPMiddle == dlpMiddle || dlpMiddle->isPlatform() != supDLPMiddle->isPlatform())
					continue;

				trimesh::vec3 supMiddle = supParam.up;
				trimesh::vec3 supBottom = supParam.down;
				float supLen = trimesh::len(supMiddle - supBottom);

				if (std::fabs(supportLen - supLen) > 20.0f)
					continue;

				if (bottom.z > supMiddle.z || middle.z < supBottom.z)
					continue;

				float maxz = middle.z > supMiddle.z ? supMiddle.z : middle.z;
				float minz = bottom.z > supBottom.z ? bottom.z : supBottom.z;

				if (maxz - minz < 2.0f)
					continue;

				trimesh::vec3 delta = bottom - supBottom;
				delta.z = 0.0f;
				float l = trimesh::len(delta);

				if (l > 20.0f)
					continue;

				if (l < minLen)
				{
					minLen = l;
					nearestMiddle = supDLPMiddle;
				}
			}

			if (nearestMiddle && (std::find(properMiddles.begin(), properMiddles.end(), nearestMiddle) == properMiddles.end()))
			{
				properMiddles.push_back(nearestMiddle);
			}
		};
		auto generateFrom2Middles = [&autoParam, &param](DLPMiddle* dlpMiddle1, DLPMiddle* dlpMiddle2, std::vector<DLPComponent*>& links) {
			float offset = autoParam.startheight;
			const MiddleParam& param1 = dlpMiddle1->param();
			const MiddleParam& param2 = dlpMiddle2->param();

			trimesh::vec3 middle1 = param1.up;
			trimesh::vec3 middle2 = param2.up;
			trimesh::vec3 bottom1 = param1.down;
			trimesh::vec3 bottom2 = param2.down;
			bottom1.z += offset;
			bottom2.z += offset;

			float minz = bottom1.z > bottom2.z ? bottom1.z : bottom2.z;
			float maxz = middle2.z > middle1.z ? middle1.z : middle2.z;
			float radius = param1.radius > param2.radius ? param2.radius : param1.radius;

			trimesh::vec3 xy1 = bottom1;
			trimesh::vec3 xy2 = bottom2;

			auto f = [&dlpMiddle1, &dlpMiddle2, &radius, &param1, &param2, &links](trimesh::vec3& start, trimesh::vec3& end) {

				LinkParam param;
				param.startRadius = param1.radius;
				param.endRadius = param2.radius;
				param.start = start;
				param.end = end;

				links.push_back(new DLPLink(param));
			};

			float gap = trimesh::len(xy1 - xy2);
			float delta = gap;
			int time = (maxz - minz) / delta;

			for (int i = 0; i < time; ++i)
			{
				float lowz = minz + (float)i * delta;
				float upz = lowz + delta;

				trimesh::vec3 v1low = trimesh::vec3(xy1.x, xy1.y, lowz);
				trimesh::vec3 v1up = trimesh::vec3(xy1.x, xy1.y, upz);
				trimesh::vec3 v2low = trimesh::vec3(xy2.x, xy2.y, lowz);
				trimesh::vec3 v2up = trimesh::vec3(xy2.x, xy2.y, upz);

				if (gap < 6.0f * radius)
				{
					int t = (int)(10.0f / gap);
					if (t != 0 && i % t == 0) f(v1low, v2up);
				}
				else if (gap > 12.0f * radius)
				{
					f(v1low, v2up);
					f(v1up, v2low);
				}
				else
				{
					if (i % 2 == 0) f(v1low, v2up);
					else f(v1up, v2low);
				}
			}
		};

		std::vector<DLPComponent*> totalComponents;
		for (DLPISource& source : sources)
		{
			std::vector<DLPComponent*> components;
			createFromSource(source, components);

			totalComponents.insert(totalComponents.end(), components.begin(), components.end());
			for (DLPComponent* component : components)
			{
				if (component->type() == DLPUserType::eDLPMiddle)
				{
					std::vector<DLPMiddle*> properMiddles;
					GetProperMiddlesFromMiddle((DLPMiddle*)component, properMiddles);
					for (DLPMiddle* properMiddle : properMiddles)
					{
						std::vector<DLPComponent*> linkComponents;
						generateFrom2Middles((DLPMiddle*)component, properMiddle, linkComponents);
						
						totalComponents.insert(totalComponents.end(), linkComponents.begin(), linkComponents.end());
					}
				}
			}
		}

		trimesh::TriMesh* supportMesh = nullptr;
		int totalSize = 0;
		size_t size = totalComponents.size();
		std::vector<trimesh::ivec2> startAndSizes;
		if (size > 0)
		{
			startAndSizes.resize(size);

			int start = 0;
			for (size_t i = 0; i < size; ++i)
			{
				trimesh::ivec2 c(start, 0);
				c.y = totalComponents.at(i)->count();
				startAndSizes.at(i) = c;
				start += c.y;
			}

			totalSize = startAndSizes.back().x + startAndSizes.back().y;
		}

		if (totalSize > 0)
		{
			supportMesh = new trimesh::TriMesh();
			supportMesh->vertices.resize(totalSize);
			for (size_t i = 0; i < size; ++i)
			{
				trimesh::ivec2 c = startAndSizes.at(i);

				totalComponents.at(i)->create(&supportMesh->vertices.at(c.x));
			}

			int triNum = totalSize / 3;
			if (triNum > 0)
			{
				supportMesh->faces.resize(triNum);
				for (int i = 0; i < triNum; ++i)
					supportMesh->faces.at(i) = trimesh::TriMesh::Face(3 * i, 3 * i + 1, 3 * i + 2);
			}
		}

		for (DLPComponent* component : totalComponents)
			delete component;
		totalComponents.clear();

		return supportMesh;
	}

	trimesh::TriMesh* DLPSupportGenerator::generatePad()
	{
		trimesh::box3 box = m_mesh->bbox;

		trimesh::vec3 size = box.size();
		trimesh::vec3 extend = trimesh::vec3(size.x, size.y, 0.0f) * 0.2f;
		trimesh::vec3 extendMin = box.min - extend;
		trimesh::vec3 extendMax = box.max + extend;

		box += extendMin;
		box += extendMax;

		box.min.z = -3.0f;
		box.max.z = 0.0f;
		trimesh::TriMesh* mesh = createBox(box);
		return mesh;
	}

	trimesh::TriMesh* DLPSupportGenerator::generate()
	{
		if(!m_mesh)
			return nullptr;

		trimesh::TriMesh* rawMesh = new trimesh::TriMesh();
		rawMesh->faces = m_mesh->faces;
		rawMesh->vertices = m_mesh->vertices;
		rawMesh->bbox = m_mesh->bbox;

		trimesh::vec3 d = rawMesh->bbox.min;
		d.z = 10.0f;

		trimesh::trans(rawMesh, d - rawMesh->bbox.min);

		trimesh::TriMesh* support = generateSupport(rawMesh);
		std::vector<trimesh::TriMesh*> meshes;
		meshes.push_back(rawMesh);
		if (support)
			meshes.push_back(support);

		if (m_param.pad_enable)
		{
			trimesh::TriMesh* pad = generatePad();
			if (pad)
				meshes.push_back(pad);
		}

		trimesh::TriMesh* outMesh = new trimesh::TriMesh();
		mmesh::mergeTriMesh(outMesh, meshes);

		int size = (int)meshes.size();
		for (int i = 0; i < size; ++i)
			delete meshes.at(i);
		meshes.clear();

		return outMesh;
	}
}