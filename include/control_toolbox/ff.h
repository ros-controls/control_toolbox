#ifndef CONTROL_TOOLBOX__FF_H
#define CONTROL_TOOLBOX__FF_H

#include <ros/ros.h>

using namespace ros;

namespace control_toolbox 
{
	template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
	}
	
	struct Gains
	{
		// Optional constructor for passing in values
		Gains(double v, double c)
		: v_gain_(v),
		c_gain_(c),
		c_gain_pos_(c),
		c_gain_neg_(c)
		{}
		// Default constructor
		Gains() {}
		double v_gain_;  /**< Viscus gain. */
		double c_gain_;  /**< Columb gain. */
		double c_gain_pos_;  /**< Bi-directional Columb gain. */
		double c_gain_neg_;  /**< Bi-directional Columb gain. */
	};
  
	class Ff
	{
		public:
			Ff() {};
			~Ff() {};
			
			bool init(const NodeHandle &node);
			double computeCommand(double input, Duration dt);
			double computeCommand(double input, double input_dot, Duration dt);
			
		private:
			Gains gains_;
			double input_dot;
			double p_input_last_;
			bool c_bi_directional_;
			
	};
}

#endif
