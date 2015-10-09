#include <control_toolbox/block_wave.h>

namespace control_toolbox 
{
		BlockWave::BlockWave()
		{
			frequency_ = 0.0;			
			duty_cycle_ = 0.0;
			amplitude_ = 0.0;
			
			cmd_ = 0.0;
			
			period_ = 0.0;
			up_time_ = 0.0;
			down_time_ = 0.0;
			threshold_ = 1.0;
		}
		
		BlockWave::~BlockWave()
		{}
		
		bool BlockWave::init(double frequency, double dutyCycle, double amplitude)
		{
			frequency_ = frequency;
			duty_cycle_ = dutyCycle;
			amplitude_ = amplitude;
			
			cmd_ = 0.0;
			
			period_ = 1.0/frequency_;
			up_time_ = period_ * duty_cycle_;
			down_time_ = period_ - up_time_;
			threshold_ = sin(2.0*M_PI*frequency_*(down_time_/4.0));
			return true;
		}
		
		double BlockWave::update(double time)
		{
			double sin_value = sin(2.0*M_PI*frequency_*(time + down_time_/2.0));
			
			if(sin_value > 0 && sin_value > threshold_)
				cmd_ = amplitude_;
			else if(sin_value < 0 && sin_value < -threshold_)
				cmd_ = -amplitude_;
			else
				cmd_ = 0;
			
			return cmd_;
		}
}
