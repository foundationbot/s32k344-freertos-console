#include "app.h"
#include "bsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "string.h"


#define BUFFER_SIZE 64

static uint8_t  Rx_Buffer[BUFFER_SIZE]; /* line buffer -- owned by Term_Task only now */
static uint32_t Buffer_Idx = 0;

static uint8_t  Rx_Byte_Staging;        /* single-byte landing pad the driver writes into */

/* One queued entry per raw byte received. The ISR only ever pushes into
 * this and re-arms reception -- it never calls a transmit function, so
 * there is exactly one context (Term_Task) that ever touches the LPUART
 * transmit path. That's what eliminates the Async-vs-Sync TX conflict,
 * rather than trying to serialize two transmitting contexts against each
 * other. A depth of 8 is generous for human typing speed; raise it if
 * you ever expect bursts (e.g. pasted text) faster than Term_Task can
 * drain it. */
static QueueHandle_t s_rx_byte_queue = NULL;

void LPUART6_UserCallback(const uint8 HwInstance, const Lpuart_Uart_Ip_EventType Event, const void *UserData);

/**
 * @brief   Creates the byte-relay queue. Call once at system init, before
 *          the scheduler starts and before LPUART6 interrupts are enabled.
 */
void Term_Task_Init(void)
{
    s_rx_byte_queue = xQueueCreate(8, sizeof(uint8_t));
}

/**
* @brief        Terminal task: owns ALL line editing, echo, and transmit
*               for this console. The ISR only captures raw bytes and
*               hands them off -- it never transmits anything itself.
* @details      Event-driven: blocks on s_rx_byte_queue, which
*               LPUART6_UserCallback() feeds one byte at a time.
*/
void Term_Task(void *p)
{
    (void)p;

    Buffer_Idx = 0;

    /* Arm reception exactly once, ever -- the callback below keeps
     * re-arming itself for the next byte every time, independent of
     * line boundaries, so there's never a gap where a keystroke could
     * be dropped between one line finishing and the next starting. */
    Lpuart_Uart_Ip_AsyncReceive(UART_INSTANCE, &Rx_Byte_Staging, 1);

    for( ;; )
    {
        uint8_t ch;

        if (xQueueReceive(s_rx_byte_queue, &ch, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        switch (ch)
        {
        case '\r':
        case '\n':
            /* PuTTY (serial/raw) sends CR on Enter by default, not LF --
             * accept either. Echo a clean CRLF regardless of which one
             * arrived, so the terminal always advances a line cleanly. */
            Lpuart_Uart_Ip_SyncSend(UART_INSTANCE, (const uint8_t*)"\r\n", 2U, 50000);

            Rx_Buffer[Buffer_Idx] = 0U;

            Lpuart_Uart_Ip_SyncSend(UART_INSTANCE, (const uint8_t*)"Received: ", 10U, 50000);
            Lpuart_Uart_Ip_SyncSend(UART_INSTANCE, (const uint8_t*)Rx_Buffer,
                                     strlen((const char*)Rx_Buffer), 50000);
            Lpuart_Uart_Ip_SyncSend(UART_INSTANCE, (const uint8_t*)"\r\n", 2U, 50000);

            Siul2_Dio_Ip_TogglePins(LED_PORT, (1U << LED_PIN));
            Buffer_Idx = 0;
            break;

        case '\b':
        case 0x7F: /* DEL -- what many terminals send for Backspace */
            if (Buffer_Idx > 0U)
            {
                Buffer_Idx--;
                Lpuart_Uart_Ip_SyncSend(UART_INSTANCE, (const uint8_t*)"\b \b", 3U, 50000);
            }
            break;

        default:
            if ((ch >= 0x20U) && (ch < 0x7FU) && (Buffer_Idx < (BUFFER_SIZE - 1U)))
            {
                Rx_Buffer[Buffer_Idx++] = ch;
                /* Echo the character back. Safe here -- this is the ONLY
                 * context that ever calls a transmit function, so there's
                 * no possibility of racing a still-in-flight send. */
                Lpuart_Uart_Ip_SyncSend(UART_INSTANCE, &ch, 1U, 50000);
            }
            /* else: unprintable char, or line full -- silently dropped */
            break;
        }
    }
}

/**
* @brief   LPUART6_UserCallback -- pure transport relay, nothing else.
*
* @details On each received byte: push it to Term_Task's queue, then
*          immediately re-arm reception for the next byte. No line
*          editing, no terminator detection, no transmit calls happen
*          here at all -- all of that moved into Term_Task, which is now
*          the sole owner of the transmit path. This is what fixes the
*          "no echo of the confirmation message" bug: it wasn't really a
*          semaphore-timing problem, it was two different contexts
*          (this ISR and Term_Task) both trying to transmit on the same
*          instance without the driver serializing them for us.
*/
void LPUART6_UserCallback(const uint8 HwInstance, const Lpuart_Uart_Ip_EventType Event, const void *UserData)
{
	(void)UserData;

	if(HwInstance == UART_INSTANCE)
	{
		switch(Event){
		case LPUART_UART_IP_EVENT_RX_FULL:
		{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			uint8_t rxch = Rx_Byte_Staging;

			(void)xQueueSendFromISR(s_rx_byte_queue, &rxch, &xHigherPriorityTaskWoken);

			/* Re-arm immediately for the next byte -- independent of
			 * whatever Term_Task is doing with bytes already queued. */
			(void)Lpuart_Uart_Ip_SetRxBuffer(UART_INSTANCE, &Rx_Byte_Staging, 1U);

			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			break;
		}
		case LPUART_UART_IP_EVENT_TX_EMPTY:
			break;
		case LPUART_UART_IP_EVENT_END_TRANSFER:
			break;
		case LPUART_UART_IP_EVENT_ERROR:
			break;
		default:
			break;
		}
	}
}
