/*
 * Copyright (c) 2015, Andre Gaschler, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef _RL_HAL_KUKAFRI_H_
#define _RL_HAL_KUKAFRI_H_

#include <string>
#include <rl/math/Transform.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>

#include "Timer.h"

#include <FastResearchInterface.h>

namespace rl
{
	namespace hal
	{
		class KukaFRI : public JointPositionActuator, public JointPositionSensor
		{
		public:
			KukaFRI(
					const ::std::size_t& dof,
					const ::std::chrono::nanoseconds& updateRate,
					const char* configuration_file
			);
			
			virtual ~KukaFRI();
			
			void close();

			void getJointPosition(::rl::math::Vector& q) const;
			
			void getEstimatedForcesAndTorques(::rl::math::Vector& x);

			void halt();

			void open();
			
			void release();

			void setJointPosition(const ::rl::math::Vector& q);

			void start(FastResearchInterface::LWRControlModes mode);
			
			void start();

			void step();

			void stop();

		public:
			FastResearchInterface * fri;
		private:
			dama::Timer timer;
		};
	}
}

#endif // _RL_HAL_KUKAFRI_H_
