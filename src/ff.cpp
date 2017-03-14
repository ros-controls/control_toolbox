#include <control_toolbox/ff.h>

using namespace std;
using namespace ros;

namespace control_toolbox 
{
	bool Ff::init(const ros::NodeHandle &node)
	{
		ros::NodeHandle nh(node);		

		// Load Ff gains from parameter server
		nh.param("v", gains_.v_gain_, 0.0);
		nh.param("c", gains_.c_gain_, 0.0);
		
		c_bi_directional_ = false;
			
		bool pos_def = nh.getParam("c_pos", gains_.c_gain_pos_);
		bool neg_def = nh.getParam("c_neg", gains_.c_gain_neg_);
		if (!pos_def || !neg_def)
		{
			gains_.c_gain_pos_ = 0;
			gains_.c_gain_neg_ = 0;
		}
		else
			c_bi_directional_ = true;
		
		p_input_last_ = 0.0;

		return true;
	}
	
	double Ff::computeCommand(double input, Duration dt)
	{
		//ROS_INFO("ff:: input: %f",input);
		if (dt.toSec() > 0.0) // Calculate the derivative error
		{
			input_dot = (input - p_input_last_) / dt.toSec();
			p_input_last_ = input;
		}
		else
		 input_dot = 0.0;

		return computeCommand(input, input_dot, dt);
	}
	
	double Ff::computeCommand(double input, double input_dot, Duration dt)
	{		
		double v_term = gains_.v_gain_*input;
		double c_term;
		
		if(c_bi_directional_)
		{
			if(sgn(input_dot) > 0)
				c_term = gains_.c_gain_pos_*sgn(input_dot);
			else
				c_term = gains_.c_gain_neg_*sgn(input_dot);	
		}
		else
			c_term = gains_.c_gain_*sgn(input_dot);	 
		
		double cmd_ = v_term + c_term;
		//ROS_INFO("ff:: cmd_: %f",cmd_);
			
		return cmd_;
	}
}
