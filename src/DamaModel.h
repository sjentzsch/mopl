/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAMODEL_H_
#define _DAMA_DAMAMODEL_H_

#include <algorithm>
#include <rl/math/Rotation.h>
#include <rl/plan/DistanceModel.h>
#include <rl/sg/Body.h>
#include <rl/sg/so/Body.h>

#include "DamaRrt.h"
#include "DamaSupportSurface.h"
#include "DamaPrimPickup.h"
#include "DamaPrimPushExterior.h"
#include "DamaPrimPushInterior.h"
#include "DamaPrimPushFrontal.h"
#include "DamaPrimPushMobile.h"
#include "DamaPrimTransferRigid.h"
#include "DamaPrimTransit.h"
#include "Timer.h"

namespace dama
{
	namespace ESP
	{
		enum Res {OK, UNREACHABLE, ALREADY_THERE};
		static const int nRes = 3;
		static const char * cRes[nRes] = {"OK", "UNREACHABLE", "ALREADY_THERE"};
	}

	class DamaModel : public ::rl::plan::DistanceModel
	{
	public:
		DamaModel();

		virtual ~DamaModel();

		virtual void printQLine(const ::rl::math::Vector& q, bool showDefinedState = false, bool addNewLine = true, bool printFullState = false, bool jointsInDeg = true, ::std::ostream& os = ::std::cout) const;

		virtual void printTransform(::rl::math::Transform& t, bool addNewLine = true) const;

		virtual ::std::size_t getDof() const;

		virtual ::std::size_t getDof(::std::size_t index) const;

		virtual void getMaximum(::rl::math::Vector& maximum) const;

		virtual void getMaximum(::rl::math::Vector& maximum, ::std::size_t subspace) const;

		virtual void getMinimum(::rl::math::Vector& minimum) const;

		virtual void getMinimum(::rl::math::Vector& minimum, ::std::size_t subspace) const;

		virtual bool isValid(const ::rl::math::Vector& q) const;

		virtual ::rl::math::Vector getSubspace(const ::rl::math::Vector& q, ::std::size_t subspace) const;

		virtual ::std::size_t getNumMovableComponents() const;

		virtual ::rl::math::Real distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;

		virtual ::rl::math::Real inverseOfTransformedDistance(const ::rl::math::Real& d) const;

		virtual ::rl::math::Real transformedDistance(const ::rl::math::Real& d) const;

		virtual ::rl::math::Real transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;

		virtual ::rl::math::Real maxDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const;

		virtual ::rl::math::Real minDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const;

		virtual ::rl::math::Real minDistanceToRectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cuttingDimension) const;

		virtual ::rl::math::Real newDistance(const ::rl::math::Real& dist, const ::rl::math::Real& oldOff, const ::rl::math::Real& newOff, const int& cuttingDimension) const;

		virtual ::std::size_t indexFromSubspace(::std::size_t subspace) const;

		virtual ::rl::math::Real metricSubspaceDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, ::std::size_t subspace) const;

		virtual bool isMovingSubspace(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, ::std::size_t subspace) const;

		virtual bool isMovingObject(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;

		virtual bool isMoving(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;

		virtual bool isGoal(const ::rl::math::Vector& q) const;

		virtual ::rl::math::Real robotMovementDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;

		virtual ::rl::math::Real metricConnectInterpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;

		virtual ::rl::math::Real cartesianRobotDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;

		virtual ::rl::math::Real cartesianRobotDistanceToObject(const ::rl::math::Transform& T, const ::rl::math::Vector& q, ::std::size_t subspace) const;

		virtual const ::rl::math::Transform& calcForwardKinematics(const ::rl::math::Vector& q, const ::std::size_t positionIndex, ::rl::mdl::Dynamic* currMdl = NULL) const;

		static bool calcInversePositionTaskPosture(::rl::mdl::Kinematic* kin,
				const ::rl::math::Transform& x_target,
				::rl::math::Vector& q_current,
				const rl::math::Matrix& task_space_projection = rl::math::Matrix::Identity(6,6),
				const bool coupleJoint1And2 = true,
				const ::std::size_t& iterations = 100,
				rl::math::Real limit_q_target_step_norm = 30 * rl::math::DEG2RAD,
				rl::math::Real limit_q_posture_step_norm = 30 * rl::math::DEG2RAD
			);
		
		static bool calcInversePositionTaskPostureMeka(::rl::mdl::Kinematic* kin,
				const ::rl::math::Transform& x_target,
				::rl::math::Vector& q_current,
				const rl::math::Matrix& task_space_projection = rl::math::Matrix::Identity(6,6),
				const ::std::size_t& iterations = 100,
				rl::math::Real limit_q_target_step_norm = 30 * rl::math::DEG2RAD,
				rl::math::Real limit_q_posture_step_norm = 30 * rl::math::DEG2RAD
			);

		virtual void completePartialSample(const ::rl::math::Vector& samplePart, const ::rl::math::Vector& nearest, ::rl::math::Vector& sampleFull) const;

		virtual ESP::Res emptySpacePlanner(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, ::std::vector< ::std::string >& vecEdgeAction, const bool usePreciseVersion = false) const;

		virtual void interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q, ::std::string edgeAction = "") const;

		virtual void setPosition(const ::rl::math::Vector& q);

		virtual void setPosition(const ::rl::math::Vector& q, ::std::size_t subspace);

		virtual void updateFrames(const bool& doUpdateModel = true);

		using Model::isColliding;

		virtual bool isColliding(::std::string primName = DamaPrimTransit::getInstance()->getName());

		virtual bool isCollidingWithScene(::std::size_t subspace);

		virtual bool isOnSupportSurface(const ::rl::math::Vector& q, const ::std::size_t subspace, ::std::size_t* supportSurface = NULL) const;

		virtual void updateRobotPosition(::rl::math::Vector& q, ::rl::mdl::Dynamic* myMdl = NULL);

		DamaRrt* dRrt;

		// task description names
		::std::string prefixName;
		::std::string description;

		::std::size_t numRobots;	// number of robots
		::std::size_t numObjects;	// number of objects
		::std::size_t dimObjects;	// dimensions of each object (= DoF)

		::rl::math::Vector maximumObjects;	// vector containing max. joint values for the objects
		::rl::math::Vector minimumObjects;	// vector containing min. joint values for the objects

		::std::vector< DamaSupportSurface* > vecSupportSurface;

		::rl::math::Real epsilon;	// distance threshold for equal-check in configuration comparison
		::rl::math::Real epsilonTransformed;	// distance threshold for equal-check in configuration comparison

		::rl::math::Real timeIK;	// store the time needed for IK

		::std::string seedType;
		unsigned int groupSeed;
		::std::size_t singleSeed;

		::std::size_t numRuns;	// number of runs of the same problem given the group seed

		bool debugMode;								// TODO: not so nice to store it here ...

		bool workspaceSampling;						// TODO: not so nice to store it here ...
		::std::size_t workspaceSamplingEdges;		// TODO: not so nice to store it here ...

		double kinematicSoftRangeScale;

		::std::vector<bool> currIsChosen;			// TODO: not so nice to store it here ...
		bool forwardSearch;							// TODO: not so nice to store it here ...
		bool checkCollisionRobotObjects;			// TODO: not so nice to store it here ...
		bool allRevoluteJoints;						// TODO: not so nice to store it here ...
		bool allPrismaticJoints;					// TODO: not so nice to store it here ...
		bool coupleJoint1And2;						// TODO: not so nice to store it here ...

		// for dRrt config stuff
		bool bidirectional;
		bool hierarchical;
		bool accurateDistance;

		// for hierarchical planning only
		::rl::math::Real objectPathTimeout;
		::rl::math::Real subProblemTimeout;

		bool isViewerModel;							// TODO: delete me later after debugging ...

		// Metric-related for Robot
		::rl::math::Real metRadToMeter;
		::rl::math::Real metQuatToMeter;
		::rl::math::Real metOverallRobotWeight;

		// Metric-related for composed space
		::std::string metType;
		::rl::math::Real metMovingObjectPenalty;
		::rl::math::Real metMoveToObjectPenalty;
		::rl::math::Real metMoveToObjectFactor;

		// Sample-related constants
		::rl::math::Real sampleRobotPoseProb;
		::rl::math::Real sampleObjectsFreeProb;
		::rl::math::Real sampleRobotRandProb;

		// IK-related
		static rl::math::Real ikPostureGainMaxStep; // reasonable maximum step per iteration
		static bool ikPrematureQuit;				// not further optimizing posture, generates different grasps wrt start value

		// PostProc
		bool pathSmoothing;
		bool extendPushes;

		// Viewer-related
		::std::string viewerMode;
		::std::string viewerOnlyResult;
		bool showFullTree;
		bool showExportableTree;
		double pointSize;
		double lineWidth;
		double speedFactor;

		::rl::mdl::Dynamic* mdlGrasp;				// second mdl file for alternative operational points

		// Primitives and Collision-Detection-related (-> individual end-effector bodies)
		::std::vector< DamaPrim* > vecDamaPrim;		// Thread.cpp needs it to be public ...
		::std::size_t numEndEffectorBodies;
		// TODO: only have one currEndEffectorBody, replace the vis one by using also the default one, should work according to Andre
		::rl::sg::Body* currEndEffectorBody;
		::rl::sg::so::Body* currEndEffectorBodyVis;

	protected:

	private:

	};
}

#endif /* _DAMA_DAMAMODEL_H_ */
