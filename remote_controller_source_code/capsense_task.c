/******************************************************************************
* File Name: capsense_task.c
*
* Description: This file contains the task that that controls the capsense
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
 * Header file includes
 ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "capsense_task.h"
#include "uart_debug.h"

#include "at42qt2120.h"

bool capsense_presence;

/*******************************************************************************
* Function Name: vtask_status_led
********************************************************************************
* Summary:
*  Task that controls the status LED.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void task_capsense(void *param)
{
    /* Variable used to store the return values of RTOS APIs */
    BaseType_t rtos_api_result;

    /* Remove warning for unused parameter */
    (void)param;


    i2c_init();

	uint8_t button_state, button_prev_state = AT42QT2120_read_buttons();

	bool last_led_state = true;

	TickType_t last_wake_time;
	//Set toggle interval to 20ms
	const TickType_t sample_interval = 200/portTICK_PERIOD_MS;

	// Initialize the last_wake_time variable with the current time
	last_wake_time = xTaskGetTickCount();

	capsense_presence = false;

    /* Repeatedly running part of the task */
    for(;;)
    {
    	// Wait for the next cycle.
		vTaskDelayUntil( &last_wake_time, sample_interval );

		button_prev_state = button_state;
		button_state = AT42QT2120_read_buttons();


		// button pressed
		if ((button_state & 0x01) && !button_prev_state)
		{

			task_print_info("button pressed");
			capsense_presence = true;

		} else if (!button_state && button_prev_state) {
			task_print_info("button released");
			capsense_presence = false;
		}
    }
}

