/*
 * FreeRTOS Kernel V11.1.0
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


#ifndef PORTMACRO_H
#define PORTMACRO_H

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR          char
#define portFLOAT         float
#define portDOUBLE        double
#define portLONG          long
#define portSHORT         short
#define portSTACK_TYPE    uint32_t
#define portBASE_TYPE     long

typedef portSTACK_TYPE   StackType_t;
typedef long             BaseType_t;
typedef unsigned long    UBaseType_t;

#if ( configTICK_TYPE_WIDTH_IN_BITS == TICK_TYPE_WIDTH_16_BITS )
    typedef uint16_t     TickType_t;
    #define portMAX_DELAY              ( TickType_t ) 0xffff
#elif ( configTICK_TYPE_WIDTH_IN_BITS == TICK_TYPE_WIDTH_32_BITS )
    typedef uint32_t     TickType_t;
    #define portMAX_DELAY              ( TickType_t ) 0xffffffffUL

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
 * not need to be guarded with a critical section. */
    #define portTICK_TYPE_IS_ATOMIC    1
#else
    #error configTICK_TYPE_WIDTH_IN_BITS set to unsupported tick type width.
#endif
/*-----------------------------------------------------------*/

extern uint8_t vPortGET_CORE_ID(void);
/* Multi-core */
#define portRTOS_LOCK_COUNT                     configNUMBER_OF_CORES
#define portMAX_CORE_COUNT                      configNUMBER_OF_CORES

/* Check validity of number of cores specified in config */
#if ( configNUMBER_OF_CORES < 1 || portMAX_CORE_COUNT < configNUMBER_OF_CORES )
    #error "Invalid number of cores specified in config!"
#endif

#if(configNUMBER_OF_CORES > 1)
#if ( configTICK_CORE < 0 || configTICK_CORE > configNUMBER_OF_CORES )
    #error "Invalid tick core specified in config!"
#endif
#endif
/* FreeRTOS core id is always zero based, so always 0 if we're running on only one core */
#if (configNUMBER_OF_CORES > 1)
    #define portGET_CORE_ID()    vPortGET_CORE_ID()
#else
    #define portGET_CORE_ID()    0
#endif

/* portNOP() is not required by this port. */
#define portNOP()

#define portINLINE              __inline

#ifndef portFORCE_INLINE
    #define portFORCE_INLINE    inline __attribute__( ( always_inline ) )
#endif
/* MSCM - Peripheral instance base addresses */
#if defined (CPU_S32G399A)
#define CPXNUM_OFFSET                            (0x04)
#define IRCP0ISR0_OFFSET                         (0xA60)                     /**< Interrupt Router CP0 Interrupt Status, offset: 0xA60 */
#define IRCP0IGR0_OFFSET                         (0xA64)                     /**< Interrupt Router CP0 Interrupt Generation, offset: 0xA64 */
#define IRCP_ID_OFFSET                           (0x70)
#define CPXNUM_CPN_MASK                          (0xFFu)  
#else
#define CPXNUM_OFFSET                            (0x04)
#define IRCP0ISR0_OFFSET                         (0x200)                     /**< Interrupt Router CP0 Interrupt Status, offset: 0x200 */
#define IRCP0IGR0_OFFSET                         (0x204)                     /**< Interrupt Router CP0 Interrupt Generation, offset: 0x204 */
#define IRCP_ID_OFFSET                           (0x20)
#define CPXNUM_CPN_MASK                          (0xFFU)
#endif
/*-----------------------------------------------------------*/

#define SYS_REG32(address)      ( *(volatile int   *)(address) ) /**<  32-bit register */
/** Peripheral MSCM base address */
#define REPRESENTS_INT        0
#if defined (CPU_S32G274A) || defined (CPU_S32G399A)
#define MSMC_BASE             0x40198000u
#define INT_ID                1u
#define CORE_M7_OFFSET        (0x4u)
#else
#define MSMC_BASE             0x40260000u
#define INT_ID                0u
#define CORE_M7_OFFSET        (0x0u)
#endif

#define MSMC_IRCP_ISR         MSMC_BASE + IRCP0ISR0_OFFSET
#define MSMC_IRCP_IGR         MSMC_BASE + IRCP0IGR0_OFFSET
#define MSCM_CPXNUM           MSMC_BASE + CPXNUM_OFFSET
#define NVIC_ISER             0XE000E100
#define NVIC_IPR              0XE000E400
#define NVIC_SHIFT_PRI        4
#define NVIC_MASK_PRI         0xFFu
#define INT_EN_BIT            0u

#define GET_MSCM_CPXNUM                                                   SYS_REG32(MSCM_CPXNUM)
#define TRIGGER_ISR_TO_CORE(target_core)                                  SYS_REG32(MSMC_IRCP_IGR + (target_core + CORE_M7_OFFSET) * IRCP_ID_OFFSET + REPRESENTS_INT * 0x8) |= (1 << INT_EN_BIT)
#define CLEAR_ISR_CORE_TO_CORE(target_core, master_core)                  SYS_REG32(MSMC_IRCP_ISR + (target_core + CORE_M7_OFFSET) * IRCP_ID_OFFSET + REPRESENTS_INT * 0x8) |= (1 << master_core)
#define NVIC_ENABLE_IRQ(int_id)                                           SYS_REG32(NVIC_ISER) |= (1 << int_id)
#define NVIC_SET_PRIORITY(int_id, pri)                                    SYS_REG32(NVIC_IPR + (int_id / 4) * 32) |= (uint32_t)(((((uint32_t)pri & NVIC_MASK_PRI) << NVIC_SHIFT_PRI * (int_id % 4))))
 
void vYieldCore( int xCoreID );
#define portDISABLE_INTERRUPTS()                  vPortRaiseBASEPRI()
#define portENABLE_INTERRUPTS()                   vPortSetBASEPRI( 0 )

/*-----------------------------------------------------------*/
/* Critical nesting count management. */
#define portCRITICAL_NESTING_IN_TCB    0

extern UBaseType_t uxCriticalNesting[ configNUMBER_OF_CORES ];
#define portGET_CRITICAL_NESTING_COUNT(  )          ( uxCriticalNesting[ ( vPortGET_CORE_ID() ) ] )
#define portSET_CRITICAL_NESTING_COUNT( x )         ( uxCriticalNesting[ ( vPortGET_CORE_ID() ) ] = ( x ) )
#define portINCREMENT_CRITICAL_NESTING_COUNT(  )    ( uxCriticalNesting[ ( vPortGET_CORE_ID() ) ]++ )
#define portDECREMENT_CRITICAL_NESTING_COUNT(  )    ( uxCriticalNesting[ ( vPortGET_CORE_ID() ) ]-- )

#define portSET_INTERRUPT_MASK_FROM_ISR()         ulPortRaiseBASEPRI()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )    vPortSetBASEPRI( x )

#if ( configNUMBER_OF_CORES == 1)
    extern void vPortEnterCritical( void );
    extern void vPortExitCritical( void );
    #define portENTER_CRITICAL()                      vPortEnterCritical()
    #define portEXIT_CRITICAL()                       vPortExitCritical()
#else
    extern void vTaskEnterCritical( void );
    extern void vTaskExitCritical( void );
    extern UBaseType_t vTaskEnterCriticalFromISR( void );
    extern void vTaskExitCriticalFromISR( UBaseType_t uxSavedInterruptStatus );
    #define ISR_LOCK                                  (0u)
    #define TASK_LOCK                                 (1u)
    #define portYIELD_CORE( a )                       vYieldCore( a )

    #define portSET_INTERRUPT_MASK()                  ulPortRaiseBASEPRI()
    #define portCLEAR_INTERRUPT_MASK( ulState )       vPortSetBASEPRI(ulState)

    #define portGET_ISR_LOCK()                        vPortRecursiveLock(ISR_LOCK, pdTRUE)
    #define portRELEASE_ISR_LOCK()                    vPortRecursiveLock(ISR_LOCK, pdFALSE)   
    #define portGET_TASK_LOCK()                       vPortRecursiveLock(TASK_LOCK, pdTRUE)
    #define portRELEASE_TASK_LOCK()                   vPortRecursiveLock(TASK_LOCK, pdFALSE)

    #define portENTER_CRITICAL()                      vTaskEnterCritical()
    #define portEXIT_CRITICAL()                       vTaskExitCritical()

    #define portENTER_CRITICAL_FROM_ISR()             vTaskEnterCriticalFromISR()
    #define portEXIT_CRITICAL_FROM_ISR( x )           vTaskExitCriticalFromISR( x )
#endif 

#define portRTOS_LOCK_COUNT                     configNUMBER_OF_CORES
#define portMAX_CORE_COUNT                      configNUMBER_OF_CORES
/*-----------------------------------------------------------*/

#if ( configNUMBER_OF_CORES > 1)

int32_t GateSmp_tryLock(uint32_t *gateWord);

void GateSmp_unlock(uint32_t *gateWord);

/* Read 32b value shared between cores */
extern uint32_t Get_32(volatile uint32_t* x);

/* Write 32b value shared between cores */
extern void Set_32(volatile uint32_t* x, uint32_t value);
portFORCE_INLINE static void vPortRaiseBASEPRI( void );

extern uint32_t GateWord[ portRTOS_LOCK_COUNT ];
extern volatile uint32_t ucOwnedByCore[ portMAX_CORE_COUNT ];
extern volatile uint32_t ucRecursionCountByLock[ portRTOS_LOCK_COUNT ];

/*-----------------------------------------------------------*/
static inline void vPortRecursiveLock(uint32_t ulLockNum, BaseType_t uxAcquire)
{

    uint32_t ulCoreNum = portGET_CORE_ID();
    uint32_t ulLockBit = 1u << ulLockNum;

    /* Lock acquire */
    if (uxAcquire)
    {
        /* Check if spinlock is available */
        /* If spinlock is not available check if the core owns the lock */
        /* If the core owns the lock wait increment the lock count by the core */
        /* If core does not own the lock wait for the spinlock */
        if( GateSmp_tryLock( &GateWord[ulLockNum] ) != 0)
        {
            /* Check if the core owns the spinlock */
            if( Get_32(&ucOwnedByCore[ulCoreNum]) & ulLockBit )
            {
                configASSERT( Get_32(&ucRecursionCountByLock[ulLockNum]) != 255u);
                Set_32(&ucRecursionCountByLock[ulLockNum], (Get_32(&ucRecursionCountByLock[ulLockNum])+1));
                return;
            }

            /* Preload the gate word into the cache */
            uint32_t dummy = GateWord[ulLockNum];
            dummy++;

            /* Wait for spinlock */
            while( GateSmp_tryLock(&GateWord[ulLockNum]) != 0);
        }

         /* Add barrier to ensure lock is taken before we proceed */
    	__asm__ volatile ("dsb sy" ::: "memory");
    	__asm__ volatile ("dmb sy" ::: "memory");

        /* Assert the lock count is 0 when the spinlock is free and is acquired */
        configASSERT(Get_32(&ucRecursionCountByLock[ulLockNum]) == 0);

        /* Set lock count as 1 */
        Set_32(&ucRecursionCountByLock[ulLockNum], 1);
        /* Set ucOwnedByCore */
        Set_32(&ucOwnedByCore[ulCoreNum], (Get_32(&ucOwnedByCore[ulCoreNum]) | ulLockBit));
    }
    /* Lock release */
    else
    {
        /* Assert the lock is not free already */
        configASSERT( (Get_32(&ucOwnedByCore[ulCoreNum]) & ulLockBit) != 0 );
        configASSERT( Get_32(&ucRecursionCountByLock[ulLockNum]) != 0 );

        /* Reduce ucRecursionCountByLock by 1 */
        Set_32(&ucRecursionCountByLock[ulLockNum], (Get_32(&ucRecursionCountByLock[ulLockNum]) - 1) );

        if( !Get_32(&ucRecursionCountByLock[ulLockNum]) )
        {
            Set_32(&ucOwnedByCore[ulCoreNum], (Get_32(&ucOwnedByCore[ulCoreNum]) & ~ulLockBit));
            GateSmp_unlock(&GateWord[ulLockNum]);
        	__asm__ volatile ("dsb sy" ::: "memory");
        	__asm__ volatile ("dmb sy" ::: "memory");

        }
    }
}
#endif

/* Architecture specifics. */
#define portSTACK_GROWTH      ( -1 )
#define portTICK_PERIOD_MS    ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT    8
#define portDONT_DISCARD      __attribute__( ( used ) )

/*-----------------------------------------------------------*/

/* Scheduler utilities. */
#define portYIELD()                                     \
    {                                                   \
        /* Set a PendSV to request a context switch. */ \
        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT; \
                                                        \
        /* Barriers are normally not required but do ensure the code is completely \
         * within the specified behaviour for the architecture. */ \
        __asm volatile ( "dsb" ::: "memory" );                     \
        __asm volatile ( "isb" );                                  \
    }

#define portNVIC_INT_CTRL_REG     ( *( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT    ( 1UL << 28UL )
#define portEND_SWITCHING_ISR( xSwitchRequired ) \
    do                                           \
    {                                            \
        if( xSwitchRequired != pdFALSE )         \
        {                                        \
            traceISR_EXIT_TO_SCHEDULER();        \
            portYIELD();                         \
        }                                        \
        else                                     \
        {                                        \
            traceISR_EXIT();                     \
        }                                        \
    } while( 0 )
#define portYIELD_FROM_ISR( x )    portEND_SWITCHING_ISR( x )
/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
 * not necessary for to use this port.  They are defined so the common demo files
 * (which build with all the ports) will build. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters )    void vFunction( void * pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters )          void vFunction( void * pvParameters )
/*-----------------------------------------------------------*/

/* Tickless idle/low power functionality. */
#ifndef portSUPPRESS_TICKS_AND_SLEEP
    extern void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );
    #define portSUPPRESS_TICKS_AND_SLEEP( xExpectedIdleTime )    vPortSuppressTicksAndSleep( xExpectedIdleTime )
#endif
/*-----------------------------------------------------------*/

/* Architecture specific optimisations. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
    #define configUSE_PORT_OPTIMISED_TASK_SELECTION    1
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

/* Generic helper function. */
    __attribute__( ( always_inline ) ) static inline uint8_t ucPortCountLeadingZeros( uint32_t ulBitmap )
    {
        uint8_t ucReturn;

        __asm volatile ( "clz %0, %1" : "=r" ( ucReturn ) : "r" ( ulBitmap ) : "memory" );

        return ucReturn;
    }

/* Check the configuration. */
    #if ( configMAX_PRIORITIES > 32 )
        #error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
    #endif

/* Store/clear the ready priorities in a bit map. */
    #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities )    ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
    #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities )     ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

/*-----------------------------------------------------------*/

    #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities )    uxTopPriority = ( 31UL - ( uint32_t ) ucPortCountLeadingZeros( ( uxReadyPriorities ) ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/*-----------------------------------------------------------*/

#ifdef configASSERT
    void vPortValidateInterruptPriority( void );
    #define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()    vPortValidateInterruptPriority()
#endif


portFORCE_INLINE static BaseType_t xPortIsInsideInterrupt( void )
{
    uint32_t ulCurrentInterrupt;
    BaseType_t xReturn;

    /* Obtain the number of the currently executing interrupt. */
    __asm volatile ( "mrs %0, ipsr" : "=r" ( ulCurrentInterrupt )::"memory" );

    if( ulCurrentInterrupt == 0 )
    {
        xReturn = pdFALSE;
    }
    else
    {
        xReturn = pdTRUE;
    }

    return xReturn;
}

/*-----------------------------------------------------------*/

portFORCE_INLINE static void vPortRaiseBASEPRI( void )
{
    uint32_t ulNewBASEPRI;

    __asm volatile
    (
        "   mov %0, %1                                              \n" \
        "   cpsid i                                                 \n" \
        "   msr basepri, %0                                         \n" \
        "   isb                                                     \n" \
        "   dsb                                                     \n" \
        "   cpsie i                                                 \n" \
        : "=r" ( ulNewBASEPRI ) : "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) : "memory"
    );
    __asm__ volatile ("dsb sy" ::: "memory");
    __asm__ volatile ("dmb sy" ::: "memory");
}

/*-----------------------------------------------------------*/

portFORCE_INLINE static uint32_t ulPortRaiseBASEPRI( void )
{
    uint32_t ulOriginalBASEPRI, ulNewBASEPRI;

    __asm volatile
    (
        "   mrs %0, basepri                                         \n" \
        "   mov %1, %2                                              \n" \
        "   cpsid i                                                 \n" \
        "   msr basepri, %1                                         \n" \
        "   isb                                                     \n" \
        "   dsb                                                     \n" \
        "   cpsie i                                                 \n" \
        : "=r" ( ulOriginalBASEPRI ), "=r" ( ulNewBASEPRI ) : "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) : "memory"
    );

    __asm__ volatile ("dsb sy" ::: "memory");
    __asm__ volatile ("dmb sy" ::: "memory");
    /* This return will not be reached but is necessary to prevent compiler
     * warnings. */
    return ulOriginalBASEPRI;
}
/*-----------------------------------------------------------*/

portFORCE_INLINE static void vPortSetBASEPRI( uint32_t ulNewMaskValue )
{
    __asm volatile
    (
        "   msr basepri, %0 " ::"r" ( ulNewMaskValue ) : "memory"
    );

    __asm__ volatile ("dsb sy" ::: "memory");
}
/*-----------------------------------------------------------*/

#define portMEMORY_BARRIER()    __asm volatile ( "" ::: "memory" )

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* PORTMACRO_H */
