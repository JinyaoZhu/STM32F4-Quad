#include "sys_params.h"

sys_params_t g_sys_params = {0.0883,      /* length */
                              0.460,      /* mass */
                              9.81,       /* gravity */
                              0.000466,     /* Ixx */
	                            0.000466,     /* Iyy */
	                            0.0007,     /* Izz */
                              3.1744*4,      /* max force_z  = 4*factor_pwm2f */
	                            0.00,       /* min force_z */
                              3.1744 //4.0861       /*factor_pwm2f f=factor_pwm2f*pwm for each motor */
                              };   
