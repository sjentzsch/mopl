/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <boost/lexical_cast.hpp>

#include "DamaModel.h"
#include "DamaSampler.h"

namespace dama
{
	DamaSampler::DamaSampler() :
		Sampler(),
		dModel(NULL),
		randReal(
			::boost::mt19937(static_cast< ::boost::mt19937::result_type >(0)),
			::boost::uniform_real< ::rl::math::Real >(0.0f, 1.0f)
		)
	{
	}

	DamaSampler::~DamaSampler()
	{
	}

	void DamaSampler::init()
	{
		vecMaximum.resize(this->dModel->getNumMovableComponents());
		vecMinimum.resize(this->dModel->getNumMovableComponents());
		for(::std::size_t i=0; i<this->dModel->getNumMovableComponents(); ++i)
		{
			::rl::math::Vector maximum(this->dModel->getDof(i));
			this->dModel->getMaximum(maximum, i);
			vecMaximum.at(i) = maximum;
			::rl::math::Vector minimum(this->dModel->getDof(i));
			this->dModel->getMinimum(minimum, i);
			vecMinimum.at(i) = minimum;
		}
	}

	// should not be called, instead use the version with index
	void DamaSampler::generate(::rl::math::Vector& q)
	{
		bool robotObjectCoupled = false;
		this->generateOnly(q, 0, robotObjectCoupled);
	}

	void DamaSampler::generateOnly(::rl::math::Vector& q, const ::std::size_t indexSubspace, const ::std::size_t supportSurface, const ::std::vector<bool>* qIsDefined)
	{
		::std::size_t startIndex = this->dModel->indexFromSubspace(indexSubspace);

		::rl::math::Vector vecCurrMinimum = vecMinimum.at(indexSubspace);
		::rl::math::Vector vecCurrMaximum = vecMaximum.at(indexSubspace);

		if(indexSubspace > 0 && supportSurface < this->dModel->vecSupportSurface.size())
		{
			vecCurrMinimum = this->dModel->vecSupportSurface.at(supportSurface)->getMin();
			vecCurrMaximum = this->dModel->vecSupportSurface.at(supportSurface)->getMax();
		}

		while(true)
		{
			for(::std::size_t i=0; i<this->dModel->getDof(indexSubspace); ++i)
			{
				if(qIsDefined == NULL || qIsDefined->at(startIndex+i) == false)
				{
					if(this->dModel->coupleJoint1And2 && indexSubspace == 0 && ((i == 1 && qIsDefined != NULL && qIsDefined->at(startIndex+2) == true) || (i == 2)))
						q(startIndex+i) = q(startIndex+(i == 1 ? 2 : 1));
					else
						q(startIndex+i) = vecCurrMinimum(i) + this->randReal() * (vecCurrMaximum(i) - vecCurrMinimum(i));
				}
			}

			// As sample will be in the air, sample as long as there is no valid grasp position of the arm for our object
			if(indexSubspace > 0 && supportSurface == this->dModel->vecSupportSurface.size() && !this->dModel->allPrismaticJoints)
			{
				// calculate the robot position to grasp the object
				rl::math::Transform T_grasp;
				T_grasp.translation().x() = q(startIndex);
				T_grasp.translation().y() = q(startIndex+1);
				T_grasp.translation().z() = q(startIndex+2) + DamaPrimPickup::HEIGHT_OFFSET_OBJECT;
				T_grasp.linear() = (
					rl::math::AngleAxis(DamaPrimPickup::PICKUP_Z_AXIS, rl::math::Vector3::UnitZ()) *	// WILL BE FREE AXIS
					rl::math::AngleAxis(DamaPrimPickup::PICKUP_Y_AXIS, rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(DamaPrimPickup::PICKUP_X_AXIS, rl::math::Vector3::UnitX())
				).toRotationMatrix();

				rl::math::Matrix66 constraint_position_orient = rl::math::Matrix66::Zero();
				constraint_position_orient.diagonal() << 1, 1, 1, 1, 1, 0;

				// TODO: somewhat arbitrary for now -> better to begin on top of table instead of in the nullposition?!
				//rl::math::Vector q_ik_start = (rl::math::Vector(10) << 0.0, 0.0, 0.0, 0.0, 15.0 * rl::math::DEG2RAD, -20.0 * rl::math::DEG2RAD, 12.0 * rl::math::DEG2RAD, 112.0 * rl::math::DEG2RAD, -10.0 * rl::math::DEG2RAD, 0.0).finished();

				rl::math::Vector q_ik_start = (rl::math::Vector(10) << 6.0, 2.0, 2.0, 80.0, 0.0, -40.0, 30.0, 120.0, 0.0, 15.0).finished() * rl::math::DEG2RAD;

				//rl::math::Vector q_ik_start = rl::math::Vector::Ones(this->dModel->getDof(0)) * 20 * rl::math::DEG2RAD;

				if(this->dModel->debugMode)
				{
					::std::cout << "Calculate IK to T = (";
					this->dModel->printTransform(T_grasp, false);
					::std::cout << ") starting at q_deg = (";
					this->dModel->printQLine(q_ik_start * rl::math::RAD2DEG, false, false, false, false);
					::std::cout << ")" << ::std::endl;
				}

				dama::Timer timerIK;
				timerIK.start();
				bool calcInverse = DamaModel::calcInversePositionTaskPosture(this->dModel->mdlGrasp, T_grasp, q_ik_start, constraint_position_orient, this->dModel->coupleJoint1And2);
				timerIK.stop();
				this->dModel->timeIK += timerIK.elapsed();

				if(calcInverse)
				{
					/*if(qIsDefined != NULL)
					{
						for(::std::size_t i=0; i<this->dModel->getDof(0); ++i)
							assert(!(qIsDefined->at(i)));
					}*/
					this->dModel->updateRobotPosition(q, this->dModel->mdlGrasp);
					break;
				}
			}
			else
				break;
		}

		//this->model->clip(q);	// TODO: need to uncomment in the future?!
	}

	void DamaSampler::generateOnlyCollisionFree(::rl::math::Vector& q, const ::std::size_t subspace, const ::std::size_t supportSurface, const ::std::vector<bool>* qIsDefined)
	{
		// TODO: maybe also respect robotObjectCoupled and look for robot-collisions in this case?! test it!

		do
		{
			this->generateOnly(q, subspace, supportSurface, qIsDefined);
			if(this->dModel->debugMode)
			{
				::std::string printSupportSurface = supportSurface < this->dModel->vecSupportSurface.size() ? boost::lexical_cast< ::std::string >(supportSurface) : "NONE";
				::std::cout << "DamaSampler::generateOnlyCollisionFree for subspace " << subspace << " and support surface " << printSupportSurface << ":" << ::std::endl;
				this->dModel->printQLine(q, false, true, true, true);
			}
			this->dModel->setPosition(q, subspace);
			if(subspace == 0)
				this->dModel->updateFrames();
		}
		while(this->dModel->isCollidingWithScene(subspace));
	}

	void DamaSampler::seed(const ::boost::mt19937::result_type& value)
	{
		this->randReal.engine().seed(value);
	}
}
