#ifndef CONTROL_TOOLBOX__BLOCKWAVE_H
#define CONTROL_TOOLBOX__BLOCKWAVE_H

#include <ros/ros.h>
#include <math.h>

namespace control_toolbox 
{
	class BlockWave
	{
		public:
			BlockWave();
			~BlockWave();
			bool init(double frequency, double dutyCycle, double amplitude);
			double update(double time);
			
		private:
			double frequency_;			
			double duty_cycle_;
			double amplitude_;
			
			double cmd_;
			
			double period_;
			double up_time_;
			double down_time_;
			double threshold_;
	};	
}

#endif

