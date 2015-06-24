/*
 * Copyright (c) 2015, Andre Gaschler, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <rl/math/Rotation.h>
#include <rl/hal/DeviceException.h>

#include "KukaFRI.h"

namespace rl
{
	namespace hal
	{
		KukaFRI::KukaFRI(
			const ::std::size_t& dof,
			const ::std::chrono::nanoseconds& updateRate,
			const char* configuration_file
		) :
			AxisController(dof, updateRate),
			JointPositionActuator(dof, updateRate),
			JointPositionSensor(dof, updateRate)
		{
			assert(dof == 7);
			this->fri = new FastResearchInterface(configuration_file);
		}
		
		KukaFRI::~KukaFRI()
		{
			if(isRunning())
				fri->StopRobot();
			delete fri;
		}

		void
		KukaFRI::close()
		{

		}
		
		void
		KukaFRI::getJointPosition(::rl::math::Vector& q) const
		{
			assert(q.size() >= this->getDof());
			float values[LBR_MNJ];
			fri->GetMeasuredJointPositions(values);
			for(std::size_t i = 0; i < LBR_MNJ; i++)
			{
				q[i] = values[i];
			}
		}
		
		void
		KukaFRI::getEstimatedForcesAndTorques(::rl::math::Vector& x)
		{
			assert(x.size() >= FRI_CART_VEC);
			float values[FRI_CART_VEC];
			fri->GetEstimatedExternalCartForcesAndTorques(values);
			for(std::size_t i = 0; i < FRI_CART_VEC; i++)
			{
				x[i] = values[i];
			}
		}
		
		void
		KukaFRI::halt()
		{

		}
		
		void
		KukaFRI::open()
		{

		}
		
		void
		KukaFRI::release()
		{

		}
		
		void
		KukaFRI::setJointPosition(const ::rl::math::Vector& q)
		{
			assert(q.size() >= this->getDof());
			
			float values[LBR_MNJ];
			for(std::size_t i = 0; i < LBR_MNJ; i++)
			{
				values[i] = q[i];
			}
			fri->SetCommandedJointPositions(values);
		}
		
		void
		KukaFRI::start(FastResearchInterface::LWRControlModes mode)
		{
			float JointStiffnessValues[LBR_MNJ];
			float JointDampingValues[LBR_MNJ];
			for(std::size_t i = 0; i < LBR_MNJ; i++)
			{
				JointStiffnessValues[i] = 250.0; //1000.0;
				JointDampingValues[i] = 0.7; //0.2;//0.7;
			}
			float CartStiffnessValues[FRI_CART_VEC];
			float CartDampingValues[FRI_CART_VEC];
			for(std::size_t i = 0; i < FRI_CART_VEC; i++)
			{
				CartStiffnessValues[i] = 10.0;
				CartDampingValues[i] = 0.7;
			}
			fri->SetCommandedJointStiffness(JointStiffnessValues);
			fri->SetCommandedJointDamping(JointDampingValues);
			fri->SetCommandedCartStiffness(CartStiffnessValues);
			fri->SetCommandedCartDamping(CartDampingValues);
			fri->StartRobot(mode);
			this->setRunning(true);
		}
		
		void
		KukaFRI::start()
		{
			this->start(FastResearchInterface::JOINT_POSITION_CONTROL);
		}
		
		void
		KukaFRI::step()
		{
			this->timer.start();
			fri->WaitForKRCTick();
			this->timer.stop();
			if(this->timer.elapsedDuration() < this->getUpdateRate() * 0.75)
				printf("Notice: KukaFRI::step WaitForKRCTick took only %1.7f s.\n", this->timer.elapsed());
		}
		
		void
		KukaFRI::stop()
		{
			fri->StopRobot();
			this->setRunning(false);
		}

	}
}
