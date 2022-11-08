/*
 * regulators.h
 *
 *  Created on: Nov 16, 2021
 *      Author: Arkadiusz
 */

#ifndef INC_REGULATORS_H_
#define INC_REGULATORS_H_

#include "stm32l4xx.h"

//
typedef enum {
	PID_regulator = 0,				// Pełny regulator PID
	THREE_position_regulator = 1, 	// Regulacja trójpołożeniowa
	PI_regulator = 2,				// Regulator w wersji PI
	PD_regulator = 3,				// Regulator w wersji PD
	P_regulator = 4					// Regulator w wersji P
}regulation_algorithm;

// Strukltura zawierająca parametry algorytmów regulacji
typedef struct {
	float previous_error; 		//Poprzedni błąd dla członu różniczkującego
	float total_error;			//Suma uchybów dla członu całkującego
	float Tp;					//Czas próbkowania

	// regulatory: PID, PI, PD, P
	float Kp;					//Wzmocnienie członu proporcjonalnego
	float Ti;					//Czas całkowania
	float Td;					//Czas różniczkowania
	int anti_windup_limit;		//Anti-Windup - ograniczenie członu całkującego

	// regulatory: dwupołożeniowy, trójpołożeniowy
	float hysteresis;			//Szerokość pętli histerezy
	float deadzone;				//Zakres strefy nieczułości w regulascji trójpołożeniowej
	int three_pos_reg_controlvalue;
	int previouscontrol;
}regulator_structure;

// Funkcje do obsługi egulatorów PID, PI, PD, P

void regulator_init(regulator_structure *pid_regulator, float Kp_init, float Ti_init, float Td_init, int anti_windup_limit_init, float histerezis_init, float deadzone_init, int three_pos_reg_controlvalue_init);

void regulator_reset(regulator_structure *pid_regulator);

int PID_calculate(regulator_structure *pid_regulator, regulation_algorithm regulator, float set_value, float process_variable);

int THREE_position_calculate(regulator_structure *three_position_regulator, float set_value, float process_variable);

#endif /* INC_REGULATORS_H_ */
