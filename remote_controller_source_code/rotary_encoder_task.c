///******************************************************************************
//* File Name: rotary_encoder_task.c
//*
//* Description: This file contains the task that that controls the rotary encoder
//*
//* Related Document: See README.md
//*
//*******************************************************************************
//* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
//*******************************************************************************
//* This software, including source code, documentation and related materials
//* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
//* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
//* protection (United States and foreign), United States copyright laws and
//* international treaty provisions. Therefore, you may use this Software only
//* as provided in the license agreement accompanying the software package from
//* which you obtained this Software ("EULA").
//*
//* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
//* non-transferable license to copy, modify, and compile the Software source
//* code solely for use in connection with Cypress's integrated circuit products.
//* Any reproduction, modification, translation, compilation, or representation
//* of this Software except as specified above is prohibited without the express
//* written permission of Cypress.
//*
//* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
//* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
//* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
//* reserves the right to make changes to the Software without notice. Cypress
//* does not assume any liability arising out of the application or use of the
//* Software or any product or circuit described in the Software. Cypress does
//* not authorize its products for use in any products where a malfunction or
//* failure of the Cypress product may reasonably be expected to result in
//* significant property damage, injury or death ("High Risk Product"). By
//* including Cypress's product in a High Risk Product, the manufacturer of such
//* system or application assumes all risk of such use and in doing so agrees to
//* indemnify Cypress against all liability.
//*******************************************************************************/
//
//
///*******************************************************************************
// * Header file includes
// ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "rotary_encoder_task.h"
#include "uart_debug.h"


#include "cycfg_ble.h"

#include "capsense_task.h"

/*****************************************************************************\
 * Function:    writeLed
 * Input:       uint8_t - brightnes ... a number between 0 and 100
 * Returns:     void
 * Description:
 *   If there is a connection and you have discovered the brightness characteristic
 *   then write it/
\*****************************************************************************/
void sendLed(uint8_t brightness)
{
    if(Cy_BLE_GetConnectionState(cy_ble_connHandle[0]) != CY_BLE_CONN_STATE_CLIENT_DISCOVERED)
    {
        task_print_info("Not connected\r\n");
        return;
    }
    task_print_info("Brightness = %d\r\n", brightness);

    cy_stc_ble_gattc_write_req_t myVal;

    myVal.handleValPair.attrHandle = cy_ble_customCServ [CY_BLE_CUSTOMC_LEDS_SERVICE_INDEX].customServChar[CY_BLE_CUSTOMC_LEDS_GREEN_CHAR_INDEX].customServCharHandle[0];
    myVal.handleValPair.value.val = &brightness;
    myVal.handleValPair.value.len = 1;
    myVal.connHandle = cy_ble_connHandle[0];

    //if (Cy_BLE_GATT_GetBusyStatus(btConnHandle.attId) == CY_BLE_STACK_STATE_FREE) {

    cy_en_ble_api_result_t rslt = Cy_BLE_GATTC_WriteCharacteristicValue(&myVal);
    if (rslt != CY_BLE_SUCCESS)
    	task_print_info("BLE GATTC write error: %d\r\n", rslt);
    //}
}



///*******************************************************************************
//* Function Name: vtask_status_led
//********************************************************************************
//* Summary:
//*  Task that controls the status LED.
//*
//* Parameters:
//*  void *param : Task parameter defined during task creation (unused)
//*
//*******************************************************************************/
void task_rotary_encoder(void *param)
{

	/* Variable used to store the return values of RTOS APIs */
	TickType_t last_wake_time;

	/* Remove warning for unused parameter */
    (void)param;


    // Set toggle interval to 20ms
    //  const TickType_t sample_interval = 200/portTICK_PERIOD_MS;

	// Initialize the last_wake_time variable with the current time
    // last_wake_time = xTaskGetTickCount();


    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;


    cyhal_quaddec_t quaddec_obj;
    uint32_t        count = 0x8000, prev_count = 0x8000;

    // 3-25, converted to float on skateboard
    uint8_t speed_pwm = 1;

    // Initialize the quadrature decoder object. Does not use index ('pin' is NC) and does not use a
    // pre-configured clock source ('clk' is NULL).
    // TODO: Change pins
    cy_rslt_t rslt = CY_RSLT_SUCCESS;
    rslt = cyhal_quaddec_init(&quaddec_obj, PIN_ROT_A, PIN_ROT_B, NC, CYHAL_QUADDEC_RESOLUTION_1X, NULL,
                              1000000);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    // Start the quadrature decoder
    rslt = cyhal_quaddec_start(&quaddec_obj);
    CY_ASSERT(CY_RSLT_SUCCESS == rslt);

    bool enabled = false;

    /* Repeatedly running part of the task */
    while(1) {

        //	vTaskDelayUntil( &last_wake_time, sample_interval );

        count = cyhal_quaddec_read_counter(&quaddec_obj);

        if (capsense_presence && !enabled) {
        	enabled = true;
        } else if (!capsense_presence && enabled) {
        	enabled = false;
			task_print_info("presence lost, ramping down speed");
			for (; speed_pwm >= 1; speed_pwm--) {
				sendLed(speed_pwm);
				 xQueueSendToFront(pwm_led_data_q, &speed_pwm, portMAX_DELAY);
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
        }

        if (!enabled) {
        	// do nothing
        // Check for direction of change
        } else if (count > prev_count && speed_pwm < 25) {
            // Clockwise rotation
        	// Speed up
			speed_pwm++;
        	task_print_info("speed up");
			sendLed(speed_pwm);
			//last_led_state = !last_led_state;
			xQueueSendToFront(pwm_led_data_q, &speed_pwm, portMAX_DELAY);
			task_print_info("sent");
        } else if (count < prev_count && speed_pwm > 1) {
            // Counter Clockwise rotation
        	// Slow down
        	speed_pwm--;
        	task_print_info("slow down");
			sendLed(speed_pwm);
			//last_led_state = !last_led_state;
			xQueueSendToFront(pwm_led_data_q, &speed_pwm, portMAX_DELAY);
			task_print_info("sent");
        }
        prev_count = count;

        vTaskDelay(xDelay);
	}
}

