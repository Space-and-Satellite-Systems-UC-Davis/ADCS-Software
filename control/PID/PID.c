#include "PID.h"
#include "adcs_math/sensors.h"
#include "print_scan.h"

double PID_command(
	double target,
	double state,
	uint64_t t_curr,
	PID_controller *controller
) {
	if (target) { printMsg("\r\nPrev Controller parameters => ePrev: %f, eCum: %f, dt: %u\r\n", controller->e_prev, controller->e_cumulative, t_curr - controller->t_prev); }
	float e_curr = state;
	float dt = get_delta_t(t_curr, (controller->t_prev));
	dt /= 100;
	if (dt <= 0) {
		printMsg("Small dt found of %f with tcurr: %u, tprev: %u\r\n", dt, (uint32_t) t_curr, (uint32_t) controller->t_prev);
	}

	float e_derivative = (e_curr - controller->e_prev)/dt;

	float command = (controller->P_gain)*e_curr
				   + (controller->I_gain)*(controller->e_cumulative)
				   + (controller->D_gain)*e_derivative;

	if (target) {
		printMsg("Curr Controller Parameters => eCurr: %f, eDeriv: %f, dt: %f\r\n", e_curr, e_derivative, dt);
		printMsg("Command result: %f\r\n", command);
	}


	// update the accumulated error, but make sure it doesn't become too large
	controller->e_cumulative = controller->e_cumulative + dt*e_curr;  // scale down since dt is in ms
	if (controller->e_cumulative > 1000) { controller->e_cumulative = 1000; }
	else if (controller->e_cumulative < -1000) { controller->e_cumulative = -1000; }

	// update the prev values
	controller->e_prev = e_curr;
	controller->t_prev = t_curr;
	
	return command;
}

void PID_init(
	double target_init,
	double state_init,
	uint64_t t_init,
	double P_gain,
	double I_gain,
	double D_gain,
	PID_controller *controller
) {
	controller->P_gain = P_gain;
	controller->I_gain = I_gain;
	controller->D_gain = D_gain;
	controller->e_prev = state_init - target_init;
	controller->t_prev = t_init;
	controller->e_cumulative = 0.0;
}
