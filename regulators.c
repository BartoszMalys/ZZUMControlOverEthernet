/*
 * regulators.c
 *
 *  Created on: Nov 16, 2021
 *      Author: Arkadiusz
 */

#include "regulators.h"

void regulator_init(regulator_structure *regulator, float Kp_init, float Ti_init, float Td_init, int anti_windup_limit_init, float hysteresis_init, float deadzone_init, int three_pos_reg_controlvalue_init) {
	regulator->previous_error = 0;
	regulator->total_error = 0;
	regulator->Tp = 0.75;

	regulator->Kp = Kp_init;
	regulator->Ti = Ti_init;
	regulator->Td = Td_init;

	regulator->anti_windup_limit = anti_windup_limit_init;

	regulator->hysteresis = hysteresis_init;
	regulator->deadzone = deadzone_init;
	regulator->three_pos_reg_controlvalue = three_pos_reg_controlvalue_init;
}

void regulator_reset(regulator_structure *regulator) {
	regulator->total_error = 0;
	regulator->previous_error = 0;
}

int PID_calculate(regulator_structure *pid_regulator, regulation_algorithm regulator, float set_value, float process_variable) {
	float error;
	float P, I, D;
	float max_total_error;

	error = set_value - process_variable;			//obliczenie uchybu

	pid_regulator->total_error += error;		//sumowanie uchybu

	max_total_error = (pid_regulator->anti_windup_limit*pid_regulator->Ti)/(pid_regulator->Kp*pid_regulator->Tp);

	//Anti-Windup - ograniczenie sumowania błędu
	if (pid_regulator->total_error > max_total_error) {
		pid_regulator->total_error = max_total_error;
	} else if (pid_regulator->total_error < -max_total_error) {
		pid_regulator->total_error = -max_total_error;
	}

	P = (float)(pid_regulator->Kp * error);																				//odpowiedź członu proporcjonalnego
	if (pid_regulator->Ti != 0){
		I = (float)(((pid_regulator->Kp/pid_regulator->Ti)*pid_regulator->total_error)*pid_regulator->Tp);				//odpowiedź członu całkującego
	} else {
		I = 0;
	}
	D = (float)((pid_regulator->Kp * pid_regulator->Td)*((error - pid_regulator->previous_error)/pid_regulator->Tp));	//odpowiedź członu różniczkującego

	//Anti-Windup - ograniczenie odpowiedzi członu całkującego
	if (I > pid_regulator->anti_windup_limit) {
		I = pid_regulator->anti_windup_limit;
	}
	else if (I < -pid_regulator->anti_windup_limit) {
		I = -pid_regulator->anti_windup_limit;
	}

	//aktualizacja zmiennej z poprzednią wartością błędu
	pid_regulator->previous_error = error;

	switch (regulator) {
		case PID_regulator:
			return (int)(P + I + D);	//odpowiedź regulatora PID
			break;
		case PI_regulator:
			return (int)(P + I);		//odpowiedź regulatora PI
			break;
		case PD_regulator:
			return (int)(P + D);		//odpowiedź regulatora PD
			break;
		case P_regulator:
			return (int)(P);			//odpowiedź regulatora P
			break;
		default:
			return (int)0;				//odpowiedź domyślna
			break;
	}
}

int THREE_position_calculate(regulator_structure *three_position_regulator, float set_value, float process_variable) {

	if(process_variable<set_value-(three_position_regulator->deadzone+three_position_regulator->hysteresis)){										// Jeśli spełnione to grzałka pracuje
		three_position_regulator->previouscontrol = 1;
		return 1;
	} else if(process_variable>set_value+(three_position_regulator->deadzone+three_position_regulator->hysteresis)){								// Jeśli spełnione to wentylator pracuje
		three_position_regulator->previouscontrol = -1;
		return -1;
	} else if(process_variable>set_value-three_position_regulator->deadzone &&
			  process_variable<set_value+three_position_regulator->deadzone) {																		// Jeśli spełnione to grzałka i wntylator nie pracują
		three_position_regulator->previouscontrol = 0;
		return 0;
	} else {
		return three_position_regulator->previouscontrol;
	}
}

//int ON_OFF_calculate(regulator_structure *on_off_regulator, float set_value,float process_variable, int controlvalue) {
//
//	// Chłodzenie
//	if (process_variable > set_value + on_off_regulator->hysteresis/2) {
//		return controlvalue;
//	} else if (process_variable < set_value - on_off_regulator->hysteresis/2) {
//		return 0;
//	}
//
//	// Grzanie
//	if (process_variable < set_value - on_off_regulator->hysteresis/2) {
//		return controlvalue;
//	} else if (process_variable > set_value + on_off_regulator->hysteresis/2) {
//		return 0;
//	}
//}
