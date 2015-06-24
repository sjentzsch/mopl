/*
 * Copyright (c) 2015, Andre Gaschler, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <fstream>

#include <rl/math/Vector.h>
#include <rl/mdl/XmlFactory.h>

#include "XmlFactory.h"
#include "DamaModel.h"

rl::math::Transform
TransformFromPositionEulerDegrees(rl::math::Vector v)
{
	assert(v.size() == 6);
	
	rl::math::Transform x;
	x.translation() = v.head(3);
	x.linear() = (
			rl::math::AngleAxis(v(5) * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *
			rl::math::AngleAxis(v(4) * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(v(3) * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
		).toRotationMatrix();
	
	return x;
}

int main(int argc, char** argv)
{
	
	rl::mdl::XmlFactory factory;
	std::shared_ptr< rl::mdl::Model > model(factory.create("data/models/mekabot-convex/rlmdl/mekabot-torso-rightarm.xml"));
	rl::mdl::Kinematic* kinematics = dynamic_cast< rl::mdl::Kinematic* >(model.get());	
	
	bool res_all = true;
	
	{
		rl::math::Vector t(6);
		t << 0, 0.26, 0.101, -45.993, 111.091, 135.993;
		rl::math::Vector q(10);
		q << 6, 2, 2, 80, 0, -40, 30, 120, 0, 15;
		q *= rl::math::DEG2RAD;
		rl::math::Transform T = TransformFromPositionEulerDegrees(t);
		std::cout << "t: " << t.transpose() << std::endl;
		std::cout << "q before: " << q.transpose() << std::endl;
		bool res = dama::DamaModel::calcInversePositionTaskPosture(kinematics, T, q, rl::math::Matrix::Identity(6,6));
		std::cout << " IK " << ((res) ? "succeeded" : " failed") << std::endl;
		std::cout << "q after: " << q.transpose() << std::endl;
		res_all &= res;
	}
	{
		rl::math::Vector t(6);
		t << -5.07506e-05, 8.64659e-05, 0.101, -45.993, 111.091, 166.403;
		rl::math::Vector q(10);
		q << 6.57836, 2.56248, 2.56248, 80.4364, 2.59975, -20.9232, 23.294, 128.773, -15.4085, 26.8855;
		q *= rl::math::DEG2RAD;
		rl::math::Transform T = TransformFromPositionEulerDegrees(t);
		std::cout << "t: " << t.transpose() << std::endl;
		std::cout << "q before: " << q.transpose() << std::endl;
		bool res = dama::DamaModel::calcInversePositionTaskPosture(kinematics, T, q, rl::math::Matrix::Identity(6,6));
		std::cout << " IK " << ((res) ? "succeeded" : " failed") << std::endl;
		std::cout << "q after: " << q.transpose() << std::endl;
		res_all &= res;
	}
	
	if (res_all)
		return EXIT_SUCCESS;
	else
		return EXIT_FAILURE;
}
