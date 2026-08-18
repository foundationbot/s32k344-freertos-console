/*
 * cantx_sched_task.c
 *
 *  Created on: Aug 14, 2026
 *      Author: vcngo
 */

#include <stdio.h>
#include <string.h>

#include "cantx_sched_task.h"

/* # of messages to receive */
#define FLEXCAN_NUMBER_OF_MSG      (100U)

/* Define a text buffer large enough to hold a full CAN FD frame string */
static char uartTxBuffer[256];

Flexcan_Ip_MsgBuffType aRxDataBuffer[FLEXCAN_NUMBER_OF_MSG];

volatile uint8 u8TxConfirmCnt 		= 0U;
/* Separate the write pointer (ISR) from the read pointer (Task) */
volatile uint32 u32RxWriteIdx = 0U;
static uint32 u32RxReadIdx = 0U;

uint32 MsgIdx = 1U;

Flexcan_Ip_DataInfoType RxInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 64U,
        .is_polling = FALSE,
        .is_remote = FALSE
};
Flexcan_Ip_DataInfoType TxInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 64U,
		.fd_padding  = 0U,
		.enable_brs  = TRUE,
        .is_polling = FALSE,
        .is_remote = FALSE
};

void FlexCAN0_UserCallback(uint8 instance, Flexcan_Ip_EventType eventType,
		uint32 buffIdx, const Flexcan_Ip_StateType * flexcanState);

void CanTx_Task_Init(void)
{
	/* Configure Rx message buffer */
	FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN_0, RX_MB_IDX, &RxInfo, 0x0U);

	/* Start trigger to receive messages */
	FlexCAN_Ip_Receive(INST_FLEXCAN_0, RX_MB_IDX, &aRxDataBuffer[u32RxWriteIdx], FALSE);
}

/* Ping-pong testing of CAN communication */
void CanTx_Task(void *p)
{
	(void)p;
	uint32_t ulNotificationValue;
	uint8_t i;
	uint16_t stringLen;

	for ( ;; )
	{
		/* Wait for a CAN frame and echo it back */
		ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if (ulNotificationValue > 0)
		{
			while (u32RxReadIdx != u32RxWriteIdx)
			{
				/* 1. Extract frame information from the active read index slot */
				uint32_t canId  = aRxDataBuffer[u32RxReadIdx].msgId;
				uint8_t  dataLen = aRxDataBuffer[u32RxReadIdx].dataLen;
				uint8_t* pData   = aRxDataBuffer[u32RxReadIdx].data;

				/* 2. Format the header info: CAN ID and Payload Length */
				stringLen = sprintf(uartTxBuffer, "\r\n[CAN RX] ID: 0x%03lX | Len: %d | Data: ", canId, dataLen);

				/* 3. Safely append each data byte as hex characters to the string buffer */
				for (i = 0; i < dataLen; i++)
				{
					// Ensure we do not overflow our text buffer array
					if (stringLen < (sizeof(uartTxBuffer) - 6))
					{
						stringLen += sprintf(&uartTxBuffer[stringLen], "%02X ", pData[i]);
					}
				}

				/* Append a clean newline sequence terminating the console line */
				stringLen += sprintf(&uartTxBuffer[stringLen], "\r\n");

				/* 4. Transmit the completed string block synchronously over LPUART6 */
				/* Adjust timeout (e.g., 1000ms) to ensure characters print smoothly */
				Lpuart_Uart_Ip_SyncSend(UART_INSTANCE, (const uint8_t *)uartTxBuffer, stringLen, 50000);

				/* --- Existing echo logic --- */
				TxInfo.data_length = dataLen;
				FlexCAN_Ip_Send(INST_FLEXCAN_0, TX_MB_IDX, &TxInfo, canId, pData);
				Siul2_Dio_Ip_TogglePins(LED_PORT, (1U << LED_PIN));

				/* Move read pointer forward */
				u32RxReadIdx = (u32RxReadIdx + 1) % FLEXCAN_NUMBER_OF_MSG;
			}
		}
	}
}

void FlexCAN0_UserCallback(uint8 instance, Flexcan_Ip_EventType eventType,
		uint32 buffIdx, const Flexcan_Ip_StateType * flexcanState)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (FLEXCAN_EVENT_RX_COMPLETE == eventType)
    {
    	uint32 nextWriteIdx = (u32RxWriteIdx + 1) % FLEXCAN_NUMBER_OF_MSG;

		/* Prepare to receive the next message */
		FlexCAN_Ip_Receive(INST_FLEXCAN_0, RX_MB_IDX, &aRxDataBuffer[nextWriteIdx], FALSE);
		u32RxWriteIdx = nextWriteIdx;

		vTaskNotifyGiveFromISR(CanTx_Task_Handle, &xHigherPriorityTaskWoken);
		/* Safely update the index variable AFTER assigning the new buffer */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    if (FLEXCAN_EVENT_TX_COMPLETE == eventType)
    {
        u8TxConfirmCnt++;
    }
    (void)instance;
    (void)buffIdx;
    (void)flexcanState;
}
