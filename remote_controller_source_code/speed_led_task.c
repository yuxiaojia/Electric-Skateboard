/******************************************************************************
* File Name: speed_led_task.c
*
* Description: This file contains the task that that controls the speed LEDs
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
#include "speed_led_task.h"


/*******************************************************************************
 * Macros
 ******************************************************************************/

QueueHandle_t pwm_led_data_q;

/*******************************************************************************
 * Function prototype
 ******************************************************************************/

/*******************************************************************************
* Function Name: vtask_ece453_led
********************************************************************************
* Summary:
*  Task that controls the status LED.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void task_speed_led(void *param)
{
	/* Initialize RED LED*/
	cyhal_gpio_init(
			PIN_LED_1,                // Pin
			CYHAL_GPIO_DIR_OUTPUT,      // Direction
			CYHAL_GPIO_DRIVE_STRONG,    // Drive Mode
			true);				        // InitialValue

    /* ADD CODE to configure the GREEN LED as an output */
	cyhal_gpio_init(
				PIN_LED_2,                // Pin
				CYHAL_GPIO_DIR_OUTPUT,      // Direction
				CYHAL_GPIO_DRIVE_STRONG,    // Drive Mode
				true);				        // InitialValue

    /* ADD CODE to configure the YELLOW LED as an output */
	cyhal_gpio_init(
					PIN_LED_3,                // Pin
					CYHAL_GPIO_DIR_OUTPUT,      // Direction
					CYHAL_GPIO_DRIVE_STRONG,    // Drive Mode
					true);				        // InitialValue

    /* ADD CODE to configure the BLUE LED as an output */
	cyhal_gpio_init(
					PIN_LED_4,                // Pin
					CYHAL_GPIO_DIR_OUTPUT,      // Direction
					CYHAL_GPIO_DRIVE_STRONG,    // Drive Mode
					true);				        // InitialValue



        /* ADD CODE BELOW based on the comments provided */

        /* Block until a command has been received from the BLE task.
         * The data sent should be recieved on the FreeRTOS queue that
         * is defined in ece453_led_task.h.  Be sure to examine the
         * ece453_led_data_t struct when determining what data will be
         * received from the BLE task.

        /* Modify the RED or GREEN LEDs duty cycle based on the information
         * that was received from the BLE Task. Remember, that LED is active-low.
         * If you recieve a PWM value of 45 (45%), the time the PIN conrolling the
         * LED should be set to 1 is 55 (100 - Desired PWM Value)
         *
         * Make use of the HAL documentation to determine how to configure the pins
         * used to control the RED and GREEN LEDs as PWM pins */
	for(;;) {
		int pwm_led_data;
		xQueueReceive(pwm_led_data_q, &pwm_led_data, portMAX_DELAY);
		task_print_info("receive %d", pwm_led_data);
			if (pwm_led_data <= 3) {
				cyhal_gpio_write(PIN_LED_1, 1); // OFF
				cyhal_gpio_write(PIN_LED_2, 1); // OFF
				cyhal_gpio_write(PIN_LED_3, 1); // OFF
				cyhal_gpio_write(PIN_LED_4, 1); // OFF
			}
			if (pwm_led_data <= 7 ) {
				cyhal_gpio_write(PIN_LED_1, 0); // ON
				cyhal_gpio_write(PIN_LED_2, 1); // OFF
				cyhal_gpio_write(PIN_LED_3, 1); // OFF
				cyhal_gpio_write(PIN_LED_4, 1); // OFF
			} else if ( pwm_led_data <= 14 ) {
				cyhal_gpio_write(PIN_LED_1, 0); // ON
				cyhal_gpio_write(PIN_LED_2, 0); // ON
				cyhal_gpio_write(PIN_LED_3, 1); // OFF
				cyhal_gpio_write(PIN_LED_4, 1); // OFF
			} else if ( pwm_led_data <= 21 ) {
				cyhal_gpio_write(PIN_LED_1, 0); // ON
				cyhal_gpio_write(PIN_LED_2, 0); // ON
				cyhal_gpio_write(PIN_LED_3, 0); // ON
				cyhal_gpio_write(PIN_LED_4, 1); // OFF
			} else {
				cyhal_gpio_write(PIN_LED_1, 0); // ON
				cyhal_gpio_write(PIN_LED_2, 0); // ON
				cyhal_gpio_write(PIN_LED_3, 0); // ON
				cyhal_gpio_write(PIN_LED_4, 0); // ON
			}
	   }
}



/* [] END OF FILE */

