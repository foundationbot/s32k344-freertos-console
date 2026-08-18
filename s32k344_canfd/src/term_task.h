/* Given by the callback once a full line (Enter, or buffer full) is
 * ready; Term_Task blocks on this with zero CPU cost instead of
 * busy-polling the driver's transfer status. Must be created via
 * Term_Task_Init() before LPUART6 interrupts can fire — call it from
 * your system init sequence, before vTaskStartScheduler().
 */

#ifndef TERM_TASK_H_
#define TERM_TASK_H_

#include "app.h"
#include "bsp.h"


void Term_Task_Init(void);

void Term_Task(void *p);


#endif /* TERM_TASK_H_ */
