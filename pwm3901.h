/*
 * ADNS3080.h
 *
 *  Created on: 19.04.2019 ã.
 *      Author: ru
 */

#ifndef PWM3901_H_
#define PWM3901_H_

#include "hw_config.h"

uint8_t PWM3901_Reset(void);
void LedOnOf(uint8_t ll);
void ReadMot(uint8_t* arr);
void ReadBurstMotion(uint8_t* arr, uint8_t nreg);

#endif /* TABLE_H_ */
