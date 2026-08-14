/*==================================================================================================
* Project : RTD AUTOSAR 4.7
* Platform : CORTEXM
* Peripheral : S32K3XX
* Dependencies : none
*
* Autosar Version : 4.7.0
* Autosar Revision : ASR_REL_4_7_REV_0000
* Autosar Conf.Variant :
* SW Version : 6.0.0
* Build Version : S32K3_RTD_6_0_0_D2506_ASR_REL_4_7_REV_0000_20250610
*
* Copyright 2020 - 2025 NXP
*
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be 
*   used strictly in accordance with the applicable license terms.  By expressly 
*   accepting such terms or by downloading, installing, activating and/or otherwise 
*   using the software, you are agreeing that you have read, and that you agree to 
*   comply with and are bound by, such license terms.  If you do not agree to be 
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
==================================================================================================*/

/**
*   @file main.c
*
*   @addtogroup main_module main module documentation
*   @{
*/

/* Including necessary configuration files. */
#include "FreeRTOS.h"
#include "task.h"

#include "string.h"

/* User includes */
#include "app.h"
#include "bsp.h"
#include "term_task.h"


/* Welcome messages displayed at the console */
#define WELCOME_MSG "This is the autonomous ATV Control Unit!\r\n"

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
	/* Initialize board */
	Bsp_Init_StatusType Bsp_Init_Status = Bsp_Init();
	if (Bsp_Init_Status != BSP_INIT_SUCCESS)
	{
		while(1);
	}

	/* TermTask init */
	Term_Task_Init();

	/* Send a welcome message */
	Lpuart_Uart_Ip_SyncSend(UART_INSTANCE, (const uint8_t*)WELCOME_MSG, strlen(WELCOME_MSG), 50000);

	/* Create tasks and start the scheduler */
	xTaskCreate(Term_Task, (const char* const)"TermTask", STK_SMALL, (void*)0, PRIO_TERM, NULL);
	vTaskStartScheduler();

	/* Deinit board */
	Bsp_Deinit();

    for( ;; );

    return 0;
}

/** @} */
