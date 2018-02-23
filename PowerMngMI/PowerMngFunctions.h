/************  Copyright 2004-2012 FICOSA CORPORATIVE ELECTRONICS  ************
| Language:       |  MISRA C
| Controller:     |  dsPIC33
| Requirements:  
|-----------------|------------------------------------------------------------
| Project:        |   TCU SERVICES
|------------------------------------------------------------------------------
| HISTORY OF MODIFICATIONS
|   Date  -  Coder -                   Description
| 20/06/2012   JVR    Creation of the file. 
|------------------------------------------------------------------------------
| FILE DESCRIPTION:
| Functions for Power Management 
******************************************************************************/

#ifndef __POWERMNGCONTROLLERFUNCTIONS_H__
#define __POWERMNGCONTROLLERFUNCTIONS_H__

/* -------------------------------- Includes -------------------------------- */
#include "Global.h"
#include "ProjectCFG.h"
#include "InternComOsek.h"
#include "PowerMng.h"
#include "PowerMngFunctions.h"
#include "InfrastructureCFG.h"

/* -------------------------------- Defines --------------------------------- */

/* ------------------------------- Data Types ------------------------------- */

/* ---------------------------- Global Variables ---------------------------- */

/* Vector of flags to notify which service requires TCU ON */
extern UI_16 services_tcuon_sts[N_SERVICES];

/* Vector of flags to notify which service requires Navi ON */
extern UI_8 services_navion_sts[N_ACCOUT_SERV_FLAGS];

/* Vector of flags to notify which service requires DRX mode */
extern UI_8 services_drx_sts[N_SERV_DRX_FLAGS];

/* Vector of flags to notify which service requires GPS hotstart mode */
extern UI_8 services_hotstart_sts[N_SERV_FLAGS];

/* Vector of flags to notify which service requires the telematic drx reset */
extern UI_8 services_telem_drx_reset_request[N_SERV_FLAGS];

/* rtc power mode last request */
extern t_sig_pmc_tx_req_power_mode pm_request;

/* Boolean with the status request of CAN EV */
extern BOOL can_ev_wakeup_req;

/* Flag to notify that DRX status has been updated in eeprom */
extern BOOL drx_status_updated;
extern t_pm_req_status rtc_off_pm_request;

/* Tcu current power mode */
extern t_pmc_power_mode pmc_power_mode;

/* Wakeup status of CAN EV */
extern t_powermng_can_wakeup_sts can_ev_wakeup_sts;

/* Output on request wakeup status requested */
extern BOOL outputonreq_requested;

/* Output on request waking status flag */
extern BOOL outputonreq_waking;

/* Output on request error flag */
extern BOOL outputonreq_error;

/* --------------------------- Routine prototypes --------------------------- */

#ifdef NISSAN_VARIANT
/*---------------------------------------------------------------------------
| Portability: Generic
|----------------------------------------------------------------------------
| Routine description:
|  * Function to check if EV CAN is awaken
|---------------------------------------------------------------------------
| Parameters description:
|  result: TRUE if CAN EV is detected awaken, FALSE otherwise
/---------------------------------------------------------------------------*/
BOOL CheckEvCanAwaken(void);
#endif

/* TASKERS */
void PowerMngAccOutControlInicialitza(void); /* Function simple not neccesary */
void PowerMngAccOutControl(void);
void PowerMngDrxMngInicialitza(void);
void PowerMngDrxMng(void);
void PowerMngWakeupPhyStsInicialitza(void);
void PowerMngWakeupPhySts(void);
void PowerMngOutputOnReq(void);
void PowerMngOutputOnReqInicialitza(void);
void PowerMngCanEvWakeupMngInicialitza(void);
void PowerMngCanEvWakeupMng(void);


/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking different WAKE-UP causes to maintain TCU_ON
|---------------------------------------------------------------------------
| Parameters description:
|  
/---------------------------------------------------------------------------*/
BOOL CheckTcuOnUpdateSleepMngConditions(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking the posible ON_TX startup causes
|    and return TRUE if any of them is active
|---------------------------------------------------------------------------
| Parameters description:
|  
/---------------------------------------------------------------------------*/
BOOL CheckRtcWakeupCauseConditions(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting MonitoringOnRequest from services to initial value
|---------------------------------------------------------------------------
| Parameters description:
|  
/---------------------------------------------------------------------------*/
void InitServiceMonitoringRequest(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking service monitoring requestion
|    and return it
|---------------------------------------------------------------------------
| Parameters description:
|   return  TRUE, if any service requests monitoring on;
|   return  FALSE, if all service requests monitoring off;
/---------------------------------------------------------------------------*/
BOOL CheckServiceMonitoringRequest(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of updating service monitoring on requestion
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: service id 
|  req: monitoring requestion on or off
/---------------------------------------------------------------------------*/
void PowerMngUpdateServiceMonitoringRequest( UI_8 service_id,BOOL req);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting TcuOnRequest from services to initial value
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
/---------------------------------------------------------------------------*/
void InitServiceTcuOnRequest(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting DRX mode from services to initial value
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
/---------------------------------------------------------------------------*/
void InitServiceDrxRequest(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting GPS hotstart mode from services to initial 
|    value
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void InitGpsHotstartRequest(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting Disable Telematic Drx Reset mode from 
|    services to initial value
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void InitServiceDisableTelemDrxResetRequest(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting no wakeup requests as initial value for 
|    different services
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void InitCanWakeupRequests(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking if any service have requested TCU ON.
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested TCU ON
/---------------------------------------------------------------------------*/
BOOL CheckServiceTcuOnRequest(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of decrease of timer for tcu on request 
|---------------------------------------------------------------------------
| Parameters description: NA
/---------------------------------------------------------------------------*/
void DecServiceTcuOnTout(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking if any service have requested DRX.
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested DRX
/---------------------------------------------------------------------------*/
t_drx_status CheckServiceDrxRequest(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking if any service have requested GPS 
|    hotstart mode.
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested GPS hotstart
/---------------------------------------------------------------------------*/
BOOL CheckServiceGpsHotstartRequest(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking if any service have requested to avoid
| the reset of the telematic module when comming from DRX.
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested to avoid the reset.
/---------------------------------------------------------------------------*/
BOOL CheckServiceDisableTelemDrxResetRequest( void );

/******************************************************************************
| Routine: TrackerBlockingBlankingCallback
| ----------------------------------------------------------------------------
| Operations contract:
|   * Callback of the EepromProgStart that notify that the DRX status of the EEPROM 
|     has finished. Raises the flag drx_status_updated.
|---------------------------------------------------------------------------
| Parameters explanation: 
|---------------------------------------------------------------------------
| Execution Time:
| Tmax: ?? cycles cpu | O(n): CTE 
/----------------------------------------------------------------------------*/
void DrxUpdateStatusCallback(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of waking up the external periphereals
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested TCU ON
/---------------------------------------------------------------------------*/
void ExtControllersWakeUp(void);

void GPSControllerWakeUp(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of Monitoring the external periphereals
|---------------------------------------------------------------------------
| Parameters description:
|   
/---------------------------------------------------------------------------*/
void ExtControllersMonitoring(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of sleeping the external periphereals
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested TCU ON
/---------------------------------------------------------------------------*/
void ExtControllersSleep(void);

void GPSControllerSleep(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine to check if accelerometer is in sleep mode
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if accelerometer confirmed sleep mode
/---------------------------------------------------------------------------*/
BOOL CheckExtControllersSleep(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Function to modify core mode wota pin
|    IT'S ONLY FOR SL8! 
|---------------------------------------------------------------------------
| Parameters description:
|   -flag: TRUE for set pin to high
|          FALSE for set pin to low
|      
/---------------------------------------------------------------------------*/
void PMSetFlagExitDrxSL8(BOOL flag);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of reinit the tcu_on max time in ONTX state
|---------------------------------------------------------------------------
| Parameters description:
|  
/---------------------------------------------------------------------------*/
void InitTcuOnMaxTime(void);
#ifdef DEBUG_COMP
void CheckOutputOnTx(void);
#endif
UI_8 IntMixRtcStatus (void);


/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Get flag for forcing telematic going to off
|---------------------------------------------------------------------------
| Parameters description:
|
/---------------------------------------------------------------------------*/
BOOL PowerMngGetForceTelemOffFlag(void);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Set flag for forcing telematic going to off
|---------------------------------------------------------------------------
| Parameters description:
|
/---------------------------------------------------------------------------*/
void PowerMngSetForceTelemOffFlag(BOOL force_off);

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Set time to request when requesting Update Service On Request
|---------------------------------------------------------------------------
| Parameters description:
|  value:   value to be set (in seconds)
|
/---------------------------------------------------------------------------*/
void PowerMngSetDelayedShutdownValue( UI_16 new_value );

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Get time to request when requesting Update Service On Request
|---------------------------------------------------------------------------
| Parameters description:
|  return:   value to be set (in seconds)
|
/---------------------------------------------------------------------------*/
UI_16 PowerMngGetDelayedShutdownValue( void );

#endif/* __POWERMNGCONTROLLERFUNCTIONS_H__ */

