/******************************************************************************
* File Name: speed_led_task.h
*
* Description: This file is the public interface of status_led_task.c source
*              file.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 * Include guard
 ******************************************************************************/
#ifndef SPEED_LED_TASK_H_
#define SPEED_LED_TASK_H_
// Pin definitions for the ECE453 Staff Dev board
#define PIN_LED_1		P12_7
#define PIN_LED_2 	    P5_5       // ADD CODE
#define PIN_LED_3		P5_3       // ADD CODE
#define PIN_LED_4 		P5_2       // ADD CODE

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "uart_debug.h"
#include "leds.h"
#include "capsense_task.h"

extern QueueHandle_t pwm_led_data_q;

typedef struct
{
	uint8_t pwm;            // Duty cycle of the LED
}   pwm_led_data_t;

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void task_speed_led(void *param);



#endif


/* [] END OF FILE */
