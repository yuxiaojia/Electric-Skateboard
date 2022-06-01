/*
 *  Created on: Jan 18, 2022
 *      Author: Joe Krachey
 */

#ifndef LEDS_H__
#define LEDS_H__

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

// Pin definitions for the ECE453 Staff Dev board
#define PIN_LED_RED 		P5_0
#define PIN_LED_GREEN 	    P5_1       // ADD CODE
#define PIN_LED_YELLOW 		P5_2       // ADD CODE
#define PIN_LED_BLUE 		P5_3       // ADD CODE

/* Initializes the IO functions */
void leds_init(void);

#endif 
