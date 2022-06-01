/******************************************************************************
* File Name:rotary_encoder_task.h
*
* Description: This file is the public interface of capsense_task.c source
*              file.

*******************************************************************************/


/*******************************************************************************
 * Include guard
 ******************************************************************************/
#ifndef SOURCE_ROTARY_ENCODER_TASK_H_
#define SOURCE_ROTARY_ENCODER_TASK_H_
#define PIN_ROT_A P5_0
#define PIN_ROT_B P5_1

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "queue.h"
#include "speed_led_task.h"

#include "cy_pdl.h"
#include "cyhal.h"


/*******************************************************************************
 * Function prototype
 ******************************************************************************/
/* Task_StatusLed updates status LED indications */
void task_rotary_encoder(void *param);


#endif /* SOURCE_STATUS_LED_TASK_H_ */


/* [] END OF FILE */
