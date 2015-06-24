/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_TIMER_H_
#define _DAMA_TIMER_H_

#include <chrono>
#include <thread>

namespace dama
{
	class Timer
	{
	private:
		::std::chrono::steady_clock::time_point startTime, stopTime;
		
	public:
		Timer()
		{
		}

		virtual ~Timer()
		{
		}
		
		void start()
		{
			this->startTime = ::std::chrono::steady_clock::now();
		}
		
		void stop()
		{
			this->stopTime = ::std::chrono::steady_clock::now();
		}
		
		double elapsed()
		{
			::std::chrono::steady_clock::duration elapsedTime = this->stopTime - this->startTime;
			return (::std::chrono::duration_cast< ::std::chrono::duration< double > >(elapsedTime)).count();
		}
		
		::std::chrono::steady_clock::duration elapsedDuration()
		{
			return (this->stopTime - this->startTime);
		}
		
		static void sleep(double timeInSeconds)
		{
			::std::this_thread::sleep_for( ::std::chrono::duration< double >(timeInSeconds) );
		}
	};
}

#endif /* _DAMA_TIMER_H_ */
