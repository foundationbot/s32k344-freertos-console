/* app.h — ATV VCU POC (FreeRTOS) : global config, types, IDs, externs
 *
 * One image, two zones. Build with -DZONE_FRONT or -DZONE_REAR.
 * Target: NXP S32K144
 */
#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ---------- zone selection ---------- */
#if !defined(ZONE_FRONT) && !defined(ZONE_REAR)
#  error "Define ZONE_FRONT or ZONE_REAR"
#endif
#ifdef ZONE_FRONT
#  define ZONE_IS_FRONT 1
#else
#  define ZONE_IS_FRONT 0
#endif

/* ---------- CAN channels (physical port index 0..4) ---------- */
typedef enum {
    CH0 = 0,   /* Front: Brake        |  Rear: Housekeeping */
    CH_STEER,  /* 1  this axle's 2 steering actuators       */
    CH_PROP,   /* 2  this axle's 2 inverters                */
    CH_SUSP,   /* 3  this axle's 2 suspension actuators     */
    CH_COMPUTE,/* 4  <-> peer VCU + Jetson                  */
    CH_COUNT
} can_ch_t;

#define CH_BRAKE_OR_HK  CH0

/* ---------- CAN message IDs (11-bit, TODO update with actual IDs) ---------- */
/* Received */
#define ID_INV_L_STATUS   0x110u   /* inverter left  (FL or RL): speed+torque actual */
#define ID_INV_R_STATUS   0x111u   /* inverter right (FR or RR) */
#define ID_STEER_L_STATUS 0x120u
#define ID_STEER_R_STATUS 0x121u
#define ID_SUSP_L_STATUS  0x130u
#define ID_SUSP_R_STATUS  0x131u
#define ID_BRAKE_STATUS   0x140u   /* front only */
#define ID_PEER_STATE     0x200u   /* from the other VCU (compute bus) */
#define ID_JETSON_CMD     0x300u   /* Jetson command request (compute bus) */
/* Transmitted */
#define ID_INV_L_CMD      0x118u
#define ID_INV_R_CMD      0x119u
#define ID_STEER_L_CMD    0x128u
#define ID_STEER_R_CMD    0x129u
#define ID_SUSP_L_CMD     0x138u
#define ID_SUSP_R_CMD     0x139u
#define ID_BRAKE_CMD      0x148u
#define ID_SELF_STATE     0x201u   /* our heartbeat/state to peer */

/* Telemetry to Jetson on the compute bus (CAN-FD). Actual id = base + zone offset,
 * so Front and Rear don't collide on the shared compute bus. */
#define TLM_ZONE_OFF      0x040u    /* Front = +0x000, Rear = +0x040 */
#define ID_TLM_INV        0x310u    /* speed/torque of this axle's 2 inverters */
#define ID_TLM_STEER      0x312u    /* current angles of this axle's 2 steer actuators */
#define ID_TLM_SUSP       0x314u    /* current positions of this axle's 2 susp actuators */
#define ID_TLM_BRAKE      0x316u    /* brake pressure/command (Front only) */

/* UDS / diagnostics (ISO-TP over CAN). POC: carried on the compute bus so it works on the
 * wired FlexCAN instance; PRODUCTION: put UDS on a dedicated diagnostic/OBD bus, not the
 * safety compute bus. Physical req/resp are zone-distinct; functional is broadcast. */
#define CH_DIAG           CH_COMPUTE
#define UDS_ID_PHYS_REQ   (ZONE_IS_FRONT ? 0x7E0u : 0x7E1u)   /* tester -> this ECU */
#define UDS_ID_PHYS_RESP  (ZONE_IS_FRONT ? 0x7E8u : 0x7E9u)   /* this ECU -> tester */
#define UDS_ID_FUNC_REQ   0x7DFu                              /* functional broadcast */

/* ---------- Steering: Lenze i950 servo drives on CANopen (CiA 402 CSP) ---------- */
/* Two nodes per axle; node IDs are zone-distinct. Configure node ID / baud / PDO map /
 * mode of operation (0x6060 = 8) in Lenze EASY Starter to match these. */
#define STEER_NODE_L       (ZONE_IS_FRONT ? 1u : 3u)
#define STEER_NODE_R       (ZONE_IS_FRONT ? 2u : 4u)
#define STEER_INC_PER_DEG  100          /* CiA402 position increments per degree (calibrate) */

/* ---------- task priorities (higher = more urgent) ---------- */
/* Priorities per the canonical design table. */
#define PRIO_SAFETY      (tskIDLE_PRIORITY + 10) /* Watchdog / Safety Monitor */
#define PRIO_CANRX       (tskIDLE_PRIORITY + 9) /* CAN_Rx_Deferred */
#define PRIO_CANTX       (tskIDLE_PRIORITY + 8) /* CAN_Tx_Scheduler */
#define PRIO_CROSSVCU    (tskIDLE_PRIORITY + 8) /* Cross_VCU_Interface (+leader/demand) */
#define PRIO_BRAKE       (tskIDLE_PRIORITY + 8) /* Brake_Control (highest ASIL actuator) */
#define PRIO_PROP        (tskIDLE_PRIORITY + 7) /* Traction_Control */
#define PRIO_STEER       (tskIDLE_PRIORITY + 7) /* Steering_Control */
#define PRIO_SUSP        (tskIDLE_PRIORITY + 5) /* Suspension_Ctrl */
#define PRIO_AUTONOMY    (tskIDLE_PRIORITY + 4) /* Autonomy_Interface */
#define PRIO_HOUSEKEEP   (tskIDLE_PRIORITY + 3) /* Energy_Housekeeping (Rear) */
#define PRIO_DIAG        (tskIDLE_PRIORITY + 2) /* Diagnostics_NVM */
#define PRIO_TERM		 (tskIDLE_PRIORITY + 1) /* Terminal */

/* ---------- task periods (ms) ---------- */
#define PER_SAFETY_MS     5
#define PER_CROSSVCU_MS  10
#define PER_PROP_MS      10
#define PER_STEER_MS     10
#define PER_BRAKE_MS     10
#define PER_SUSP_MS      20
#define PER_AUTONOMY_MS  20
#define PER_HOUSEKEEP_MS 100
#define PER_DIAG_MS      100

/* ---------- stack sizes (words) ---------- */
#define STK_SMALL   (configMINIMAL_STACK_SIZE + 128)
#define STK_MED     (configMINIMAL_STACK_SIZE + 256)

/* ---------- watchdog alive-checkpoint bits ---------- */
typedef enum {
    WDG_SAFETY   = 1u << 0,
    WDG_CANRX    = 1u << 1,
    WDG_CROSSVCU = 1u << 2,
    WDG_PROP     = 1u << 3,
    WDG_STEER    = 1u << 4,
    WDG_BRAKE    = 1u << 5,
    WDG_VMODE    = 1u << 6,
    WDG_SUSP     = 1u << 7,
    WDG_CANTX    = 1u << 8,
} wdg_bit_t;

/* Tasks that must check in every safety window for the HW watchdog to be kicked.
 * CANTX is included: it is the single transmit path, so its loss must trip safe state. */
#define WDG_REQUIRED_MASK  (WDG_SAFETY|WDG_CANRX|WDG_CANTX|WDG_CROSSVCU|WDG_PROP|WDG_STEER|WDG_BRAKE)

/* ---------- vehicle safe-state ---------- */
typedef enum { ST_INIT, ST_NORMAL, ST_DEGRADED, ST_SAFE, ST_FAILSAFE } veh_state_t;

/* ---------- a decoded CAN frame carried on the RX queue ---------- */
typedef struct { uint8_t ch; uint32_t id; uint8_t dlc; uint8_t data[8]; } can_frame_t;

/* ---------- task handle list ---------- */
extern TaskHandle_t Term_Task_Handle;
extern TaskHandle_t CanTx_Task_Handle;


#endif /* APP_H */
