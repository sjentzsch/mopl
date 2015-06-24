/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <queue>

#include <boost/lexical_cast.hpp>

#include <rl/math/Quaternion.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>
#include <rl/sg/solid/Scene.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/Scene.h>
#include <rl/sg/so/Scene.h>
#include <rl/sg/Model.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/mdl/Dynamic.h>

#include "XmlFactory.h"
#include "DamaModel.h"
#include "DamaSampler.h"

namespace dama
{
	XmlFactory::XmlFactory()
	{
	}

	XmlFactory::~XmlFactory()
	{
	}

	boost::shared_ptr< DamaModel > XmlFactory::create(const ::std::string& filename, const bool isViewerModel)
	{
		boost::shared_ptr< DamaModel > model;
		model = boost::make_shared< DamaModel >();
		this->load(filename, model.get(), isViewerModel);
		return model;
	}

	void XmlFactory::load(const ::std::string& filename, DamaModel* model, const bool isViewerModel)
	{
		::rl::xml::DomParser parser;

		::rl::xml::Document doc = parser.readFile(filename, "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);

		doc.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);

		::rl::xml::Path path(doc);

		model->prefixName = path.eval("//dama").getNodeTab(0).getAttribute("prefixName").getValue();
		model->description = path.eval("//dama").getNodeTab(0).getAttribute("description").getValue();

		model->debugMode = path.eval("count(//planner/debugMode) > 0").getBoolval();

		model->workspaceSampling = path.eval("count(//planner/workspaceSampling) > 0").getBoolval();
		if(model->workspaceSampling)
		{
			rl::xml::Object xmlWorkspaceSampling = path.eval("//planner/workspaceSampling");
			model->workspaceSamplingEdges = boost::lexical_cast< ::std::size_t >(xmlWorkspaceSampling.getNodeTab(0).getAttribute("edges").getValue());
		}
		else
			model->workspaceSamplingEdges = 0;

		// TODO: work with shared pointers instead of 'new'?? but the object is not available after load returns ...

		if(!isViewerModel)
		{
			rl::xml::Object xmlSceneFile = path.eval("//model/sceneFile");
			rl::sg::solid::Scene* scene = new rl::sg::solid::Scene();
			scene->load(xmlSceneFile.getNodeTab(0).getUri(xmlSceneFile.getNodeTab(0).getAttribute("href").getValue()));
			model->model = scene->getModel(0);
			model->scene = scene;
		}
		else
		{
			rl::xml::Object xmlSceneFile;
			if(path.eval("count(//model/sceneHDFile) > 0").getBoolval())
				xmlSceneFile = path.eval("//model/sceneHDFile");
			else
				xmlSceneFile = path.eval("//model/sceneFile");
			rl::sg::so::Scene* scene = new rl::sg::so::Scene();
			scene->load(xmlSceneFile.getNodeTab(0).getUri(xmlSceneFile.getNodeTab(0).getAttribute("href").getValue()));
			model->model = scene->getModel(0);
			model->scene = scene;
		}

		// TODO: init mdl2 when it is clear how to handle model2-stuff for GUI

		rl::xml::Object xmlMdlFile = path.eval("//model/mdlFile");
		rl::mdl::XmlFactory mdlFactory;
		model->mdl = dynamic_cast< rl::mdl::Dynamic* >(mdlFactory.create(xmlMdlFile.getNodeTab(0).getUri(xmlMdlFile.getNodeTab(0).getAttribute("href").getValue())));

		if(path.eval("count(//model/mdlGraspFile) > 0").getBoolval())
		{
			rl::xml::Object xmlMdlGraspFile = path.eval("//model/mdlGraspFile");
			rl::mdl::XmlFactory mdlGraspFactory;
			model->mdlGrasp = dynamic_cast< rl::mdl::Dynamic* >(mdlGraspFactory.create(xmlMdlGraspFile.getNodeTab(0).getUri(xmlMdlGraspFile.getNodeTab(0).getAttribute("href").getValue())));
		}
		else
		{
			model->mdlGrasp = model->mdl;
		}

		model->isViewerModel = isViewerModel;

		// allRevoluteJoints, allPrismaticJoints, coupleJoint1And2, trajSampleTime
		model->allRevoluteJoints = path.eval("count(//model/allJointsAreRevolute) > 0").getBoolval();
		model->allPrismaticJoints = path.eval("count(//model/allJointsArePrismatic) > 0").getBoolval();
		model->coupleJoint1And2 = path.eval("count(//model/coupleJoint1AndJoint2) > 0").getBoolval();

		// numRobots, numObjects, dimObjects, minimumObjects, maximumObjects
		rl::xml::Object xmlModel = path.eval("//model");
		model->numRobots = 1;
		model->numObjects = boost::lexical_cast< ::std::size_t >(xmlModel.getNodeTab(0).getAttribute("numOfObjects").getValue());
		model->dimObjects = boost::lexical_cast< ::std::size_t >(xmlModel.getNodeTab(0).getAttribute("dimOfObjects").getValue());
		rl::xml::Object xmlObjectSamplingSpace = path.eval("//model/objectSamplingSpace");
		model->minimumObjects.resize(model->dimObjects);
		model->minimumObjects(0) = boost::lexical_cast< double >(xmlObjectSamplingSpace.getNodeTab(0).getAttribute("x_min").getValue());
		model->minimumObjects(1) = boost::lexical_cast< double >(xmlObjectSamplingSpace.getNodeTab(0).getAttribute("y_min").getValue());
		model->minimumObjects(2) = boost::lexical_cast< double >(xmlObjectSamplingSpace.getNodeTab(0).getAttribute("z_min").getValue());
		model->maximumObjects.resize(model->dimObjects);
		model->maximumObjects(0) = boost::lexical_cast< double >(xmlObjectSamplingSpace.getNodeTab(0).getAttribute("x_max").getValue());
		model->maximumObjects(1) = boost::lexical_cast< double >(xmlObjectSamplingSpace.getNodeTab(0).getAttribute("y_max").getValue());
		model->maximumObjects(2) = boost::lexical_cast< double >(xmlObjectSamplingSpace.getNodeTab(0).getAttribute("z_max").getValue());
		//::std::cout << "minimumObjects: " << model->minimumObjects(0) << ", " << model->minimumObjects(1) << ", " << model->minimumObjects(2) << ::std::endl;
		//::std::cout << "maximumObjects: " << model->maximumObjects(0) << ", " << model->maximumObjects(1) << ", " << model->maximumObjects(2) << ::std::endl;

		// objectSupportSurfaces
		rl::xml::Object xmlObjectSupportSurface = path.eval("//model/objectSupportSurface");
		for(int i=0; i<xmlObjectSupportSurface.getNodeNr(); ++i)
		{
			double z = boost::lexical_cast< double >(xmlObjectSupportSurface.getNodeTab(i).getAttribute("z").getValue());

			::rl::math::Vector minObjSS(model->dimObjects);
			minObjSS(0) = boost::lexical_cast< double >(xmlObjectSupportSurface.getNodeTab(i).getAttribute("x_min").getValue());
			minObjSS(1) = boost::lexical_cast< double >(xmlObjectSupportSurface.getNodeTab(i).getAttribute("y_min").getValue());
			minObjSS(2) = z;

			::rl::math::Vector maxObjSS(model->dimObjects);
			maxObjSS(0) = boost::lexical_cast< double >(xmlObjectSupportSurface.getNodeTab(i).getAttribute("x_max").getValue());
			maxObjSS(1) = boost::lexical_cast< double >(xmlObjectSupportSurface.getNodeTab(i).getAttribute("y_max").getValue());
			maxObjSS(2) = z;

			::std::string name = xmlObjectSupportSurface.getNodeTab(i).getAttribute("name").getValue();
			model->vecSupportSurface.push_back(new DamaSupportSurface(name, z, minObjSS, maxObjSS));
		}




		// Planner
		model->dRrt = new DamaRrt(model->getNumMovableComponents(), model->vecSupportSurface.size());
		model->dRrt->model = model;
		model->dRrt->dModel = model;

		// start
		rl::xml::Object xmlStart = path.eval("//problem/start/q");
		model->dRrt->start = new rl::math::Vector(xmlStart.getNodeNr());
		// TODO: assert xmlStart.getNodeNr() == model.dof()
		for(int i=0; i<xmlStart.getNodeNr(); ++i)
		{
			(*model->dRrt->start)(i) = boost::lexical_cast<double>(xmlStart.getNodeTab(i).getContent());

			if(xmlStart.getNodeTab(i).hasAttribute("unit"))
			{
				if("deg" == xmlStart.getNodeTab(i).getAttribute("unit").getValue())
				{
					(*model->dRrt->start)(i) *= rl::math::DEG2RAD;
				}
			}
		}

		// goal and goalDefined
		rl::xml::Object xmlGoal = path.eval("//problem/goal/q");
		model->dRrt->goal = new rl::math::Vector(xmlGoal.getNodeNr());
		model->dRrt->goalDimDefined = new ::std::vector<bool>(xmlGoal.getNodeNr());
		// TODO: assert xmlGoal.getNodeNr() == model.dof()
		for(int i=0; i<xmlGoal.getNodeNr(); ++i)
		{
			(*model->dRrt->goal)(i) = boost::lexical_cast<double>(xmlGoal.getNodeTab(i).getContent());

			if(xmlGoal.getNodeTab(i).hasAttribute("unit"))
			{
				if("deg" == xmlGoal.getNodeTab(i).getAttribute("unit").getValue())
				{
					(*model->dRrt->goal)(i) *= rl::math::DEG2RAD;
				}
			}

			if(xmlGoal.getNodeTab(i).hasAttribute("free"))
			{
				if("true" == xmlGoal.getNodeTab(i).getAttribute("free").getValue())
				{
					model->dRrt->goalDimDefined->at(i) = false;
				}
				else
				{
					model->dRrt->goalDimDefined->at(i) = true;
				}
			}
			else
			{
				model->dRrt->goalDimDefined->at(i) = true;
			}
		}

		// set robot to start pose
		model->mdl->setPosition(*model->dRrt->start);
		model->mdl->forwardPosition();


		// Sampler
		model->dRrt->dSampler = new DamaSampler();
		model->dRrt->dSampler->model = model;
		model->dRrt->dSampler->dModel = model;
		model->dRrt->sampler = model->dRrt->dSampler;


		// Planner Settings
		if(path.eval("count(//planner/kinematicSoftRange) > 0").getBoolval())
		{
			rl::xml::Object xmlKinematicSoftRange = path.eval("//planner/kinematicSoftRange");
			model->kinematicSoftRangeScale = boost::lexical_cast< double >(xmlKinematicSoftRange.getNodeTab(0).getAttribute("scale").getValue());
		}
		else
			model->kinematicSoftRangeScale = 1.0;
		model->dRrt->dSampler->init();	// important: init the sampler after setting the kinematicSoftRangeScale

		model->dRrt->delta = path.eval("number(//planner/delta)").getFloatval(1.0);
		if("deg" == path.eval("string(//planner/delta/@unit)").getStringval())
			model->dRrt->delta *= rl::math::DEG2RAD;

		model->dRrt->duration = ::std::chrono::duration_cast< ::std::chrono::steady_clock::duration >( ::std::chrono::duration< double >( path.eval("number(//planner/duration)").getFloatval(0) ));
		if(model->dRrt->duration <= ::std::chrono::duration_cast< ::std::chrono::steady_clock::duration >( ::std::chrono::duration< double >(0.01f) ))
			model->dRrt->duration = ::std::chrono::steady_clock::duration::max();
			
		model->dRrt->extendStep = path.eval("number(//planner/extendStep)").getFloatval(0.1);

		model->dRrt->epsilon = path.eval("number(//planner/epsilon)").getFloatval(1.0e-3f);
		model->epsilon = model->dRrt->epsilon;
		model->epsilonTransformed = model->transformedDistance(model->epsilon);

		model->dRrt->probability = 0.5;	// obsolete, is used by rl-planners to determine the probability to sample a goal

		model->dRrt->kd = path.eval("count(//planner/kdTree) > 0").getBoolval();

		// Seed
		rl::xml::Object xmlSeed = path.eval("//planner/seed");
		model->seedType = xmlSeed.getNodeTab(0).getAttribute("type").getValue();
		model->groupSeed = boost::lexical_cast< unsigned int >(path.eval("number(//planner/seed)").getFloatval(time(NULL)));
		if(model->seedType == "singleOnly")
		{
			model->groupSeed = boost::lexical_cast< unsigned int >(xmlSeed.getNodeTab(0).getAttribute("group").getValue());
			model->singleSeed = boost::lexical_cast< ::std::size_t >(xmlSeed.getNodeTab(0).getAttribute("single").getValue());
		}
		else
		{
			model->groupSeed = boost::lexical_cast< unsigned int >(xmlSeed.getNodeTab(0).getAttribute("group").getValue());
			model->singleSeed = boost::lexical_cast< ::std::size_t >(xmlSeed.getNodeTab(0).getAttribute("single").getValue());
		}

		model->numRuns = boost::lexical_cast< ::std::size_t >(path.eval("number(//planner/numRuns)").getFloatval(1));

		model->bidirectional = path.eval("count(//planner/bidirectional) > 0").getBoolval();
		model->hierarchical = path.eval("count(//planner/hierarchical) > 0").getBoolval();
		if(model->hierarchical)
		{
			model->objectPathTimeout = boost::lexical_cast< rl::math::Real >(path.eval("//planner/hierarchical").getNodeTab(0).getAttribute("objectPathTimeout").getValue());
			if(model->objectPathTimeout <= 0.01)
				model->objectPathTimeout = std::numeric_limits< rl::math::Real >::max();
			model->subProblemTimeout = boost::lexical_cast< rl::math::Real >(path.eval("//planner/hierarchical").getNodeTab(0).getAttribute("subProblemTimeout").getValue());
			if(model->subProblemTimeout <= 0.01)
				model->subProblemTimeout = std::numeric_limits< rl::math::Real >::max();
		}
		model->accurateDistance = path.eval("count(//planner/accurateDistance) > 0").getBoolval();
		model->dRrt->setConfig(model->bidirectional, model->hierarchical, model->accurateDistance);

		//::std::cout << "model->dRrt->duration: " << model->dRrt->duration << ::std::endl;

		// Sampling
		rl::xml::Object xmlSampling = path.eval("//planner/sampling");
		model->sampleRobotPoseProb = boost::lexical_cast< rl::math::Real >(xmlSampling.getNodeTab(0).getAttribute("robotPoseProb").getValue());
		model->sampleObjectsFreeProb = boost::lexical_cast< rl::math::Real >(xmlSampling.getNodeTab(0).getAttribute("objectsFreeProb").getValue());
		model->sampleRobotRandProb = boost::lexical_cast< rl::math::Real >(xmlSampling.getNodeTab(0).getAttribute("robotRandProb").getValue());

		// Metrics
		rl::xml::Object xmlMetricRobot = path.eval("//planner/metricRobot");
		model->metRadToMeter = boost::lexical_cast< rl::math::Real >(xmlMetricRobot.getNodeTab(0).getAttribute("radToMeter").getValue());
		model->metQuatToMeter = boost::lexical_cast< rl::math::Real >(xmlMetricRobot.getNodeTab(0).getAttribute("quatToMeter").getValue());
		model->metOverallRobotWeight = boost::lexical_cast< rl::math::Real >(xmlMetricRobot.getNodeTab(0).getAttribute("overallRobotWeight").getValue());

		rl::xml::Object xmlMetricComposed = path.eval("//planner/metricComposed");
		model->metType = xmlMetricComposed.getNodeTab(0).getAttribute("type").getValue();
		if(model->metType == "SumComponentMovementWithObjectPenalty")
		{
			model->metMovingObjectPenalty = boost::lexical_cast< rl::math::Real >(xmlMetricComposed.getNodeTab(0).getAttribute("movingObjectPenalty").getValue());
		}
		else if(model->metType == "ForwardDistanceWithObjectPenalty" || model->metType == "MiniESPMetricWithPenalties")
		{
			model->metMovingObjectPenalty = boost::lexical_cast< rl::math::Real >(xmlMetricComposed.getNodeTab(0).getAttribute("movingObjectPenalty").getValue());
			model->metMoveToObjectPenalty = boost::lexical_cast< rl::math::Real >(xmlMetricComposed.getNodeTab(0).getAttribute("moveToObjectPenalty").getValue());
			model->metMoveToObjectFactor = boost::lexical_cast< rl::math::Real >(xmlMetricComposed.getNodeTab(0).getAttribute("moveToObjectFactor").getValue());
		}

		// Inverse Kinematics
		if(path.eval("count(//planner/inverseKinematics) > 0").getBoolval())
		{
			rl::xml::Object xmlInverseKinematics = path.eval("//planner/inverseKinematics");
			DamaModel::ikPostureGainMaxStep = boost::lexical_cast< rl::math::Real >(xmlInverseKinematics.getNodeTab(0).getAttribute("postureGainMaxStepDeg").getValue()) * rl::math::DEG2RAD;
			if(xmlInverseKinematics.getNodeTab(0).hasAttribute("prematureQuit") && xmlInverseKinematics.getNodeTab(0).getAttribute("prematureQuit").getValue() == "true")
				DamaModel::ikPrematureQuit = true;
			else
				DamaModel::ikPrematureQuit = false;
		}

		// Collision-Detection-related and Primitive-related
		rl::xml::Object xmlPrimitives = path.eval("//planner/primitives");
		if(xmlPrimitives.getNodeTab(0).hasAttribute("numEndEffectorBodies"))
			model->numEndEffectorBodies = boost::lexical_cast< ::std::size_t >(xmlPrimitives.getNodeTab(0).getAttribute("numEndEffectorBodies").getValue());
		else
			model->numEndEffectorBodies = 1;
		if(model->isViewerModel)
			model->currEndEffectorBodyVis = (::rl::sg::so::Body*)(model->scene->getModel(0)->getBody(model->scene->getModel(0)->getNumBodies() - model->numEndEffectorBodies));
		else
			model->currEndEffectorBody = model->scene->getModel(0)->getBody(model->scene->getModel(0)->getNumBodies() - model->numEndEffectorBodies);

		// Primitives
		// NOTE: also take a look at and maybe adapt DamaModel::isColliding if adding new primitives
		model->vecDamaPrim.clear();
		if(path.eval("count(//planner/primitives/primTransit) > 0").getBoolval())
		{
			DamaPrimTransit::getInstance()->dModel = model;
			model->vecDamaPrim.push_back(DamaPrimTransit::getInstance());
			rl::xml::Object xmlPrim = path.eval("//planner/primitives/primTransit");
			::std::string endEffectorBodySearch = "";
			if(xmlPrim.getNodeTab(0).hasAttribute("endEffectorBody"))
				endEffectorBodySearch = xmlPrim.getNodeTab(0).getAttribute("endEffectorBody").getValue();
			this->setPrimEndEffectorBody(model, endEffectorBodySearch, DamaPrimTransit::getInstance());
		}
		if(path.eval("count(//planner/primitives/primPush) > 0").getBoolval())
		{
			// set the push variables within primPush
			rl::xml::Object xmlPrimPush = path.eval("//planner/primitives/primPush");
			if(xmlPrimPush.getNodeTab(0).hasAttribute("dist_to_object_xy"))
				DamaPrimPush::DIST_TO_OBJECT_XY = boost::lexical_cast< rl::math::Real >(xmlPrimPush.getNodeTab(0).getAttribute("dist_to_object_xy").getValue());
			if(xmlPrimPush.getNodeTab(0).hasAttribute("height_offset_object"))
				DamaPrimPush::HEIGHT_OFFSET_OBJECT = boost::lexical_cast< rl::math::Real >(xmlPrimPush.getNodeTab(0).getAttribute("height_offset_object").getValue());
			if(xmlPrimPush.getNodeTab(0).hasAttribute("max_push_dist"))
				DamaPrimPush::MAX_PUSH_DIST = boost::lexical_cast< rl::math::Real >(xmlPrimPush.getNodeTab(0).getAttribute("max_push_dist").getValue());
			if(xmlPrimPush.getNodeTab(0).hasAttribute("max_push_dist_joint_angle"))
				DamaPrimPush::MAX_PUSH_DIST_JOINT = boost::lexical_cast< rl::math::Real >(xmlPrimPush.getNodeTab(0).getAttribute("max_push_dist_joint_angle").getValue()) * rl::math::DEG2RAD;

			if(path.eval("count(//planner/primitives/primPush/primPushMobile) > 0").getBoolval())
			{
				DamaPrimPushMobile::getInstance()->dModel = model;
				model->vecDamaPrim.push_back(DamaPrimPushMobile::getInstance());
				rl::xml::Object xmlPrim = path.eval("//planner/primitives/primPush/primPushMobile");
				if(xmlPrim.getNodeTab(0).hasAttribute("postproc_extend_dist"))
					DamaPrimPushMobile::getInstance()->POSTPROC_EXTEND_DIST = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("postproc_extend_dist").getValue());
				::std::string endEffectorBodySearch = "";
				if(xmlPrim.getNodeTab(0).hasAttribute("endEffectorBody"))
					endEffectorBodySearch = xmlPrim.getNodeTab(0).getAttribute("endEffectorBody").getValue();
				this->setPrimEndEffectorBody(model, endEffectorBodySearch, DamaPrimPushMobile::getInstance());
			}
			if(path.eval("count(//planner/primitives/primPush/primPushInterior) > 0").getBoolval())
			{
				DamaPrimPushInterior::getInstance()->dModel = model;
				model->vecDamaPrim.push_back(DamaPrimPushInterior::getInstance());
				rl::xml::Object xmlPrim = path.eval("//planner/primitives/primPush/primPushInterior");
				if(xmlPrim.getNodeTab(0).hasAttribute("tilt_hand_inward_angle"))
					DamaPrimPushInterior::getInstance()->TILT_HAND_INWARD_ANGLE = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("tilt_hand_inward_angle").getValue()) * rl::math::DEG2RAD;
				if(xmlPrim.getNodeTab(0).hasAttribute("tilt_hand_downward_angle"))
					DamaPrimPushInterior::getInstance()->TILT_HAND_DOWNWARD_ANGLE = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("tilt_hand_downward_angle").getValue()) * rl::math::DEG2RAD;
				if(xmlPrim.getNodeTab(0).hasAttribute("postproc_extend_dist"))
					DamaPrimPushInterior::getInstance()->POSTPROC_EXTEND_DIST = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("postproc_extend_dist").getValue());
				::std::string endEffectorBodySearch = "";
				if(xmlPrim.getNodeTab(0).hasAttribute("endEffectorBody"))
					endEffectorBodySearch = xmlPrim.getNodeTab(0).getAttribute("endEffectorBody").getValue();
				this->setPrimEndEffectorBody(model, endEffectorBodySearch, DamaPrimPushInterior::getInstance());
			}
			if(path.eval("count(//planner/primitives/primPush/primPushExterior) > 0").getBoolval())
			{
				DamaPrimPushExterior::getInstance()->dModel = model;
				model->vecDamaPrim.push_back(DamaPrimPushExterior::getInstance());
				rl::xml::Object xmlPrim = path.eval("//planner/primitives/primPush/primPushExterior");
				if(xmlPrim.getNodeTab(0).hasAttribute("tilt_hand_inward_angle"))
					DamaPrimPushExterior::getInstance()->TILT_HAND_INWARD_ANGLE = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("tilt_hand_inward_angle").getValue()) * rl::math::DEG2RAD;
				if(xmlPrim.getNodeTab(0).hasAttribute("tilt_hand_downward_angle"))
					DamaPrimPushExterior::getInstance()->TILT_HAND_DOWNWARD_ANGLE = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("tilt_hand_downward_angle").getValue()) * rl::math::DEG2RAD;
				if(xmlPrim.getNodeTab(0).hasAttribute("postproc_extend_dist"))
					DamaPrimPushExterior::getInstance()->POSTPROC_EXTEND_DIST = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("postproc_extend_dist").getValue());
				::std::string endEffectorBodySearch = "";
				if(xmlPrim.getNodeTab(0).hasAttribute("endEffectorBody"))
					endEffectorBodySearch = xmlPrim.getNodeTab(0).getAttribute("endEffectorBody").getValue();
				this->setPrimEndEffectorBody(model, endEffectorBodySearch, DamaPrimPushExterior::getInstance());
			}
			if(path.eval("count(//planner/primitives/primPush/primPushFrontal) > 0").getBoolval())
			{
				DamaPrimPushFrontal::getInstance()->dModel = model;
				model->vecDamaPrim.push_back(DamaPrimPushFrontal::getInstance());
				rl::xml::Object xmlPrim = path.eval("//planner/primitives/primPush/primPushFrontal");
				if(xmlPrim.getNodeTab(0).hasAttribute("tilt_hand_inward_angle"))
					DamaPrimPushFrontal::getInstance()->TILT_HAND_INWARD_ANGLE = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("tilt_hand_inward_angle").getValue()) * rl::math::DEG2RAD;
				if(xmlPrim.getNodeTab(0).hasAttribute("tilt_hand_downward_angle"))
					DamaPrimPushFrontal::getInstance()->TILT_HAND_DOWNWARD_ANGLE = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("tilt_hand_downward_angle").getValue()) * rl::math::DEG2RAD;
				if(xmlPrim.getNodeTab(0).hasAttribute("postproc_extend_dist"))
					DamaPrimPushFrontal::getInstance()->POSTPROC_EXTEND_DIST = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("postproc_extend_dist").getValue());
				::std::string endEffectorBodySearch = "";
				if(xmlPrim.getNodeTab(0).hasAttribute("endEffectorBody"))
					endEffectorBodySearch = xmlPrim.getNodeTab(0).getAttribute("endEffectorBody").getValue();
				this->setPrimEndEffectorBody(model, endEffectorBodySearch, DamaPrimPushFrontal::getInstance());
			}
		}
		if(path.eval("count(//planner/primitives/primPickup) > 0").getBoolval())
		{
			DamaPrimPickup::getInstance()->dModel = model;
			model->vecDamaPrim.push_back(DamaPrimPickup::getInstance());
			rl::xml::Object xmlPrim = path.eval("//planner/primitives/primPickup");
			if(xmlPrim.getNodeTab(0).hasAttribute("height_offset_object"))
				DamaPrimPickup::HEIGHT_OFFSET_OBJECT = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("height_offset_object").getValue());
			if(xmlPrim.getNodeTab(0).hasAttribute("height_pickup_dist_object"))
				DamaPrimPickup::HEIGHT_PICKUP_DIST_OBJECT = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("height_pickup_dist_object").getValue());
			if(xmlPrim.getNodeTab(0).hasAttribute("min_plane_dist_for_pickup"))
				DamaPrimPickup::MIN_PLANE_DIST_FOR_PICKUP = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("min_plane_dist_for_pickup").getValue());
			if(xmlPrim.getNodeTab(0).hasAttribute("tilt_hand_x_angle"))
				DamaPrimPickup::PICKUP_X_AXIS = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("tilt_hand_x_angle").getValue()) * rl::math::DEG2RAD;
			if(xmlPrim.getNodeTab(0).hasAttribute("tilt_hand_y_angle"))
				DamaPrimPickup::PICKUP_Y_AXIS = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("tilt_hand_y_angle").getValue()) * rl::math::DEG2RAD;
			if(xmlPrim.getNodeTab(0).hasAttribute("tilt_hand_z_angle"))
				DamaPrimPickup::PICKUP_Z_AXIS = boost::lexical_cast< rl::math::Real >(xmlPrim.getNodeTab(0).getAttribute("tilt_hand_z_angle").getValue()) * rl::math::DEG2RAD;
			::std::string endEffectorBodySearch = "";
			if(xmlPrim.getNodeTab(0).hasAttribute("endEffectorBody"))
				endEffectorBodySearch = xmlPrim.getNodeTab(0).getAttribute("endEffectorBody").getValue();
			this->setPrimEndEffectorBody(model, endEffectorBodySearch, DamaPrimPickup::getInstance());
		}
		if(path.eval("count(//planner/primitives/primTransferRigid) > 0").getBoolval())
		{
			DamaPrimTransferRigid::getInstance()->dModel = model;
			model->vecDamaPrim.push_back(DamaPrimTransferRigid::getInstance());
			rl::xml::Object xmlPrim = path.eval("//planner/primitives/primTransferRigid");
			::std::string endEffectorBodySearch = "";
			if(xmlPrim.getNodeTab(0).hasAttribute("endEffectorBody"))
				endEffectorBodySearch = xmlPrim.getNodeTab(0).getAttribute("endEffectorBody").getValue();
			this->setPrimEndEffectorBody(model, endEffectorBodySearch, DamaPrimTransferRigid::getInstance());
		}

		// Remove all end-effector bodies but the first one
		::std::size_t initialNumBodies = model->scene->getModel(0)->getNumBodies();
		for(::std::size_t i=initialNumBodies-1; i>initialNumBodies-model->numEndEffectorBodies; --i)
			model->scene->getModel(0)->remove(model->scene->getModel(0)->getBody(i));



		// PostProcessing
		model->pathSmoothing = path.eval("count(//postProc/pathSmoothing) > 0").getBoolval();
		model->extendPushes = path.eval("count(//postProc/extendPushes) > 0").getBoolval();


		// Viewer
		model->viewerMode = path.eval("//viewer").getNodeTab(0).getAttribute("mode").getValue();
		if(model->viewerMode != "loop" && model->viewerMode != "once" && model->viewerMode != "off")
			model->viewerMode = "loop";
		model->viewerOnlyResult = path.eval("//viewer").getNodeTab(0).getAttribute("onlyResult").getValue();
		if(model->viewerOnlyResult != "on" && model->viewerOnlyResult != "off")
			model->viewerOnlyResult = "off";
		model->speedFactor = boost::lexical_cast< double >(path.eval("//viewer").getNodeTab(0).getAttribute("speedFactor").getValue());
		model->showFullTree = path.eval("count(//viewer/showFullTree) > 0").getBoolval();
		model->showExportableTree = path.eval("count(//viewer/showExportableTree) > 0").getBoolval();
		if(model->showExportableTree)
		{
			model->pointSize = boost::lexical_cast< double >(path.eval("//viewer/showExportableTree").getNodeTab(0).getAttribute("pointSize").getValue());
			model->lineWidth = boost::lexical_cast< double >(path.eval("//viewer/showExportableTree").getNodeTab(0).getAttribute("lineWidth").getValue());
		}
		else
		{
			model->pointSize = 8.0;
			model->lineWidth = 1.0;
		}
	}

	void XmlFactory::setPrimEndEffectorBody(DamaModel* model, ::std::string endEffectorBodySearch, DamaPrim* damaPrim)
	{
		if(endEffectorBodySearch == "")
		{
			if(model->isViewerModel)
				damaPrim->endEffectorBodyVis = model->currEndEffectorBodyVis;
			else
				damaPrim->endEffectorBody = model->currEndEffectorBody;
		}

		bool foundBody = false;
		for(::std::size_t i=model->scene->getModel(0)->getNumBodies()-1; i>model->scene->getModel(0)->getNumBodies()-1-model->numEndEffectorBodies; --i)
		{
			if(model->scene->getModel(0)->getBody(i)->getName() == endEffectorBodySearch)
			{
				if(model->isViewerModel)
					damaPrim->endEffectorBodyVis = (::rl::sg::so::Body*)(model->scene->getModel(0)->getBody(i));
				else
					damaPrim->endEffectorBody = model->scene->getModel(0)->getBody(i);
				foundBody = true;
				break;
			}
		}
		if(!foundBody)
		{
			if(model->isViewerModel)
				damaPrim->endEffectorBodyVis = model->currEndEffectorBodyVis;
			else
				damaPrim->endEffectorBody = model->currEndEffectorBody;
		}
	}
}
