/************  Copyright 2004-2012 FICOSA CORPORATIVE ELECTRONICS  ************
| Language:       |  MISRA C
| Controller:     |  dsPIC33
| Requirements:  
|-----------------|------------------------------------------------------------
| Project:        |   TCU SERVICES
|------------------------------------------------------------------------------
| HISTORY OF MODIFICATIONS
|   Date  -  Coder -                   Description
| 24/03/2014   APV    Creation of the file. 
|------------------------------------------------------------------------------
| FILE DESCRIPTION:
| Api for Power Management 
******************************************************************************/

#ifndef __POWERMNG_H__
#define __POWERMNG_H__

/* -------------------------------- Includes -------------------------------- */
#include "Global.h"
#include "InternComOsek.h"
#include "InfrastructureCFG.h"

/* -------------------------------- Defines --------------------------------- */
/* List of services allowed to wake:  ACC OUT */
#define SERV_ECALL_ACCOUT              ((UI_8)0)

/* Total number of services (must be equal to SERV_LAST + 1) */
#define N_SERVICES_ACCOUT              ((UI_8)1)

/* Number of bytes needed to monitor all services requests */
#define N_ACCOUT_SERV_FLAGS     (((N_SERVICES_ACCOUT-(UI_8)1)/(UI_8)8)+(UI_8)1) /* 1 bit for each flag */


/* IntMix Physical Signal Defines */
#define INT_MIX_PHY_ON                     ((UI_8)1)
#define INT_MIX_PHY_OFF                    ((UI_8)0)
#define INT_MIX_PHY_UNKNOWN                ((UI_8)2)

/* Ignition Physical Signal Defines */
#define IGNITION_PHY_ON                     ((UI_8)1)
#define IGNITION_PHY_OFF                    ((UI_8)0)

/* Wakeup Physical Signal Defines */
#define WAKEUP_PHY_ON                       ((UI_8)1)
#define WAKEUP_PHY_OFF                      ((UI_8)0)

/* ------------------------------- Data Types ------------------------------- */
/* Data type for check rtc status type */
typedef enum {
    PM_RTC_TCU_OFF    = 0x00,  /* PMC OFF & TELEM OFF, IMPOSIBLE STATUS */
    PM_RTC_PMC_ON     = 0x01,  /* PMC ON & TELEM OFF, PMC CHECKING WAKEUP CAUSES */
    PM_RTC_TCU_ON     = 0x02,  /* PMC ON 6 TELEM ON, NORMAL STATUS */
    PM_RTC_DRX        = 0x03,  /* PMC OFF & TELEM DRX, IMPOSIBLE STATUS */
    PM_RTC_DRX_PMC_ON = 0x04,  /* PMC ON & TELEM DRX, PMC CHECKING WAKEUP CAUSES */
    PM_RTC_UNKNOWN    = 0x05   /* UNKNOWN STATUS IN RTC */  
} t_pm_rtc_status;

/* PMC power modes */
typedef enum {
    PMC_PM_INIT             = 0x00,
    PMC_PM_ON_TX            = 0x01,
    PMC_PM_ON               = 0x02,
    PMC_PM_OFF_STARTED      = 0x03,
    PMC_PM_OFF_STORE_DTCS   = 0x04,
    PMC_PM_OFF_CONFIRMED    = 0x05,
    PMC_PM_MONITORING       = 0x06
} t_pmc_power_mode;           /* PMC power modes data type */

/* drx request mode */
typedef enum {
    DRX_DEACTIVATE    = 0, /* Deactivation of DRX      */
    DRX_ACTIVATE      = 1, /* Activation of DRX        */
    SUDO_DRX_ACTIVATE = 2  /* Forced activation of DRX  */
} t_drx_status;

typedef enum {
    MICRO_PMC   = 0,    /* PMC id */
    MICRO_TELEM = 1     /* telem id*/
} t_pm_micro_requester; /* Micros accpeted for order a change status to rtc */

typedef enum {
    PM_STATUS_CURRENT = 0, /* leave the last status */
    PM_STATUS_ON      = 1, /* set on status         */
    PM_STATUS_OFF     = 2, /* set off status        */
    PM_STATUS_DRX     = 3  /* set drx status        */
} t_pm_req_status; /* Possibles status requested to RTC */

typedef enum {
    INTERFACE_TOTAL_ON      = 0x00, /* Interface on and 100% operative */
    INTERFACE_PARTIAL_ON    = 0x01, /* Interface comunication ok but peripherical not 100% operative */
    INTERFACE_ON_DEGRADED   = 0x02, /* Interface on with some errors */
    INTERFACE_OFF           = 0x03, /* Interface off */
    INTERFACE_KO            = 0x04, /* Interface unoperative, there is no response */
    INTERFACE_STARTING      = 0x05, /* Start order sended, waiting confirmation */
    INTERFACE_SHUTTING_DOWN = 0x06, /* Off order sended, waiting confirmation */
    INTERFACE_RESETING      = 0x07  /* Reset in course */
} t_interface_status;

typedef enum {
    POWERMNG_CAN_STS_SLEEP = 0x01,
    POWERMNG_CAN_STS_AWAKE = 0x02,
    POWERMNG_CAN_STS_WAKING_UP = 0x03,
    POWERMNG_CAN_STS_ERROR = 0x04
} t_powermng_can_wakeup_sts;

typedef enum {
    POWERMNG_PVT_REQ_SLEEP = 0x01,
    POWERMNG_PVT_REQ_MONITORING = 0x02,
    POWERMNG_PVT_REQ_OTHERS = 0x03
} t_powermng_pvt_req;


/* ---------------------------- Global Variables ---------------------------- */

/* Status of the physical input wakeup signal */
extern UI_8 wakeup_phy_sts;

/* --------------------------- Routine prototypes --------------------------- */

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine to initialize the power management related FSM
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/ 
void PowerMngInicialitza(void); 

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Task that executes the power management related FSM
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void PowerMng(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of seting service TCU ON request.
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
/---------------------------------------------------------------------------*/
void UpdateServiceOnRequest(UI_8 service_id,BOOL status,UI_16 tout_secs);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of updating navi ON status
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
|  status:      Status of the request
/---------------------------------------------------------------------------*/
void UpdateServiceNaviOnRequest(UI_8 service_id, BOOL status);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of updating service monitoring on requestion
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: service id 
|  req: monitoting requestion on or off
/---------------------------------------------------------------------------*/
void UpdateServiceMonitoringRequest( UI_8 service_id,BOOL req);
/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of updating service DRX mode request in next off conditions
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
/---------------------------------------------------------------------------*/
void UpdateServiceDrxRequest(UI_8 service_id,t_drx_status status);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of updating service DRX mode request in next off conditions
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
/---------------------------------------------------------------------------*/
void UpdateServiceGpsHotstartRequest(UI_8 service_id,BOOL status);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of updating service Disable Telematic Reset 
| 	mode request in wake up from DRX
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
/---------------------------------------------------------------------------*/
void UpdateServiceDisableTelemDrxResetRequest(UI_8 service_id,BOOL status);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  Routine to request an update of a service ID CAN_V wakeup status request. The 
|  different services that will use this API shall use it with an ID from 0 to 
|  (WAKEUP_RQS_MONITOR_BYTES - 1).
|  The call of this routine will be ignored if the Sleep Management selected 
|  mode is different from "Complete mode".
|---------------------------------------------------------------------------
| Parameters description:
|   service_id: identifier of the service that requests wakeup. These service IDs 
|               are defined by the user of the SleepManagement service
|   status: flag that indicates if this service is needing the CAN active or not
| Outputs:
/---------------------------------------------------------------------------*/
void PowerMngCanVWakeupRequest(UI_8 service_id, BOOL status);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  Routine to request an update of a service ID CAN_M wakeup status request. The 
|  different services that will use this API shall use it with an ID from 0 to 
|  (WAKEUP_RQS_MONITOR_BYTES - 1).
|  The call of this routine will be ignored if the Sleep Management selected 
|  mode is different from "Complete mode".
|---------------------------------------------------------------------------
| Parameters description:
|   service_id: identifier of the service that requests wakeup. These service IDs 
|               are defined by the user of the SleepManagement service
|   status: flag that indicates if this service is needing the CAN active or not
| Outputs:
/---------------------------------------------------------------------------*/
void PowerMngCanMWakeupRequest(UI_8 service_id, BOOL status);

#ifdef NISSAN_VARIANT
/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  Routine to request an update of a service ID CAN_EV wakeup status request. The 
|  different services that will use this API shall use it with an ID from 0 to 
|  (WAKEUP_RQS_MONITOR_BYTES - 1).
|  The call of this routine will be ignored if the Sleep Management selected 
|  mode is different from "Complete mode".
|---------------------------------------------------------------------------
| Parameters description:
|   service_id: identifier of the service that requests wakeup. These service IDs 
|               are defined by the user of the SleepManagement service
|   status: flag that indicates if this service is needing the CAN active or not
| Outputs:
/---------------------------------------------------------------------------*/
void PowerMngCanEVWakeupRequest(UI_8 service_id, BOOL status);
#endif

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  Routine to check the wakeup status of the CAN_V bus
|---------------------------------------------------------------------------
| Parameters description:
| Outputs:
|    wakeup_sts: POWERMNG_CAN_STS_SLEEP = CAN_V bus is sleep
|                POWERMNG_CAN_STS_AWAKE = CAN_V bus is awake
|                POWERMNG_CAN_STS_WAKING_UP = TCU is trying to awake CAN_V bus
|                POWERMNG_CAN_STS_ERROR = CAN_V bus could not be awaken by TCU
/---------------------------------------------------------------------------*/
t_powermng_can_wakeup_sts PowerMngCanVWakeupStatus(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  Routine to check the wakeup status of the CAN_M bus
|---------------------------------------------------------------------------
| Parameters description:
| Outputs:
|    wakeup_sts: POWERMNG_CAN_STS_SLEEP = CAN_M bus is sleep
|                POWERMNG_CAN_STS_AWAKE = CAN_M bus is awake
|                POWERMNG_CAN_STS_WAKING_UP = TCU is trying to awake CAN_M bus
|                POWERMNG_CAN_STS_ERROR = CAN_M bus could not be awaken by TCU
/---------------------------------------------------------------------------*/
t_powermng_can_wakeup_sts PowerMngCanMWakeupStatus(void);

#ifdef NISSAN_VARIANT
/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  Routine to check the wakeup status of the CAN_M bus
|---------------------------------------------------------------------------
| Parameters description:
| Outputs:
|    wakeup_sts: POWERMNG_CAN_STS_SLEEP = CAN_M bus is sleep
|                POWERMNG_CAN_STS_AWAKE = CAN_M bus is awake
|                POWERMNG_CAN_STS_WAKING_UP = TCU is trying to awake CAN_M bus
|                POWERMNG_CAN_STS_ERROR = CAN_M bus could not be awaken by TCU
/---------------------------------------------------------------------------*/
t_powermng_can_wakeup_sts PowerMngCanEVWakeupStatus(void);
#endif

/*---------------------------------------------------------------------------
| Portability:
|----------------------------------------------------------------------------
| Routine description:
|  * Checks if wakeup was done by movement sensor
|---------------------------------------------------------------------------
| Parameters description:
|  result: TRUE if RFOFF mode is enabled (TODO:RLD:move to powerMngFunctions)
/---------------------------------------------------------------------------*/
BOOL IsWakeupCauseByMove(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Write in ISO 15765_3 buffer the service drx status table
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void PMWriteServicesDrxStatus(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Write in ISO 15765_3 buffer the service on tx status table
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void PMWriteServicesOnTxStatus(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Return TCU Active power mode
|---------------------------------------------------------------------------
| Parameters description:
| return: Power mode from this list
|   PMC_PM_INIT            
|   PMC_PM_ON_TX           
|   PMC_PM_ON              
|   PMC_PM_OFF_STARTED     
|   PMC_PM_OFF_STORE_DTCS  
|   PMC_PM_OFF_CONFIRMED   
/---------------------------------------------------------------------------*/
t_pmc_power_mode PMGetPowerMode(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Set the RTC power mode 
|---------------------------------------------------------------------------
| Parameters description:
|   -micro: identifier of microcontroller who wants to change his status
|           MICRO_PMC
|           MICRO_TELEM
|   -status: new status for this microcontroller, not all status are 
|            available for all microcontrollers
|           STATUS_CURRENT (avalilabe only for MICRO_PMC)
|           STATUS_ON      (available for both micros)
|           STATUS_OFF     (available for both micros)
|           STATUS_DRX     (available only for MICRO_PMC)
|      
/---------------------------------------------------------------------------*/
void PMSetRtcPowerMode(t_pm_micro_requester micro, t_pm_req_status status);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Read power status for rtc.
|---------------------------------------------------------------------------
| Parameters description:
|     return: 
|      PM_RTC_TCU_OFF,     PMC OFF & TELEM OFF, IMPOSIBLE STATUS
|      PM_RTC_PMC_ON,      PMC ON & TELEM OFF, PMC CHECKING WAKEUP CAUSES
|      PM_RTC_TCU_ON,      PMC ON 6 TELEM ON, NORMAL STATUS
|      PM_RTC_DRX,         PMC OFF & TELEM DRX, IMPOSIBLE STATUS
|      PM_RTC_DRX_PMC_ON,  PMC ON & TELEM DRX, PMC CHECKING WAKEUP CAUSES
|      PM_RTC_UNKNOWN,     UNKNOW STATUS IN RTC 
/---------------------------------------------------------------------------*/
t_pm_rtc_status PMReadRtcStatus(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  Read telematic interfac status
|---------------------------------------------------------------------------
| Parameters description:
|   return:
|    Interface status suported:
|    --------------------------
|    INTERFACE_TOTAL_ON,         Interface on and 100% operative
|    INTERFACE_PARTIAL_ON        Interface on but Telematic SO not 100% operative
|    INTERFACE_ON_DEGRADED,      Interface on with some errors
|    INTERFACE_OFF,              Interface off
|    INTERFACE_KO,               Interface unoperative, there is no response
|    INTERFACE_STARTING,         Start order sended, waiting confirmation
|    INTERFACE_SHUTTING_DOWN,    Off order sended, waiting confirmation
|    INTERFACE_RESETING          Reset in course
/---------------------------------------------------------------------------*/
t_interface_status PMReadTelemStatus(void);

/*---------------------------------------------------------------------------
| Portability: Generic
|----------------------------------------------------------------------------
| Routine description:
| 
|  Routine responsible of putting services_on_re, drx_req, hotstart_req and
|  drx_rst_req into PMC_COMMON message to debug its values.
|  
|  Telematic module doesn't use this information. Only for debug purposes.
|---------------------------------------------------------------------------
| Parameters description:
|  param_name: none
|  result: none.
/---------------------------------------------------------------------------*/
void UpdateDebugPowerSignals(void);


BOOL PowerMngGetVBatInEmergencySts( void );




//DEPRECATED by PowerMngCanWakeupRequest
void RequestBCMWakeUp(UI_8 service_id);

//DEPRECATED by PowerMngCanWakeupRequest
void EndRequestBCMWakeUp(UI_8 service_id);

//DEPRECATED by PMGetPowerMode
BOOL PowerMngIsGoingToOff(void);

#ifdef NISSAN_VARIANT
//DEPRECATED
BOOL EvNissanIsAwake(void);
//DEPRECATED
void EvNissanWakeUpRequest(UI_8 service_id);
//DEPRECATED
void EndEvNissanWakeUpRequest(UI_8 service_id);
#endif

UI_8 ReadVehicleIgnitionStatus (void);

UI_8 ReadIntMixStatus (void);

#endif/* __POWERMNG_H__ */





