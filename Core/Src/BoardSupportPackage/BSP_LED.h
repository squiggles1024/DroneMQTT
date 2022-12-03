/*
 * BSL_LED.h
 *
 *  Created on: Oct 27, 2022
 *      Author: evanl
 */

#ifndef SRC_BOARDSUPPORTPACKAGE_BSP_LED_H_
#define SRC_BOARDSUPPORTPACKAGE_BSP_LED_H_
#include <stdint.h>
/*See BSP_LED.c for detailed descriptions*/
/*Public function prototypes*/
void BSP_LEDToggleRed(void);
void BSP_LEDToggleGreen(void);

void BSP_LEDTurnOffRed(void);
void BSP_LEDTurnOnRed(void);

void BSP_LEDTurnOffGreen(void);
void BSP_LEDTurnOnGreen(void);


#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_LED_H_ */
