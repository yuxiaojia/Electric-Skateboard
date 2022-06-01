/******************************************************************************
* File Name: capsense_task.h
*
* Description: This file is the public interface of capsense_task.c source
*              file.

*******************************************************************************/


/*******************************************************************************
 * Include guard
 ******************************************************************************/
#ifndef SOURCE_STATUS_CAPSENSE_TASK_H_
#define SOURCE_STATUS_CAPSENSE_TASK_H_



/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "queue.h"
#include "speed_led_task.h"


/* Public Global Variables */
extern bool capsense_presence;

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
/* Task_StatusLed updates status LED indications */
void task_capsense(void *param);


#endif /* SOURCE_STATUS_LED_TASK_H_ */


/* [] END OF FILE */
