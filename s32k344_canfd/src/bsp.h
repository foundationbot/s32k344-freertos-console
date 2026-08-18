/* bsp.h — board/SoC bring-up for NXP S32K344 (Cortex-M7) via RTD.
 * Clock, watchdog, interrupts. The FreeRTOS ARM_CM7 port owns the SysTick-based tick,
 * so Bsp_StartOsTick() is a no-op here (kept for API parity with the AURIX build).
 */
#ifndef BSP_H
#define BSP_H

#include <stdint.h>

#include "Mcal.h"
#include "Clock_Ip.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
#include "Lpuart_Uart_Ip.h"
#include "Lpuart_Uart_Ip_Irq.h"
#include "IntCtrl_Ip.h"
#include "FlexCAN_Ip.h"

/* Define channel */
#define UART_INSTANCE (6U)

/* Macros to define MB's ID */
#define RX_MB_IDX (0U)
#define TX_MB_IDX (1U)

typedef enum
{
	BSP_INIT_SUCCESS = 0x00U,
    BSP_INIT_ERROR = 0x01U,

} Bsp_Init_StatusType;

/* Board init: Clock (PLL) check, disable init watchdogs, enable IRQs */
Bsp_Init_StatusType Bsp_Init(void);

/* Board deinit */
void Bsp_Deinit(void);

/* Actual PLL frequency reported by RTD */
uint32_t Bsp_GetCpuClockHz(void);

#endif /* BSP_H */
