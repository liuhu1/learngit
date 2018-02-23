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
| Api for Power Management 
******************************************************************************/

/* -------------------------------- Includes -------------------------------- */
#include "Global.h"
#include "ProjectCFG.h"
#include "PowerMngFunctions.h"
#include "PowerMngController.h"
#include "PowerMng.h"
#include "InternComOsek.h"
#include "FicOsekCom.h"
#include "TcuInOutFunctions.h"
#include "TelemMng.h"
#include "GeneralDiag.h"
#include "IgnitionFilterFunctions.h"

/* -------------------------------- Defines --------------------------------- */     

/* ------------------------------- Data Types ------------------------------- */

/* ---------------------------- Global Variables ---------------------------- */

/* RTC last known status */
static t_pm_rtc_status last_rtc_status = PM_RTC_UNKNOWN;

/* Status of the physical input wakeup signal */
UI_8 wakeup_phy_sts = WAKEUP_PHY_OFF;


/* --------------------------- Routine prototypes --------------------------- */

//static void RequestCanVWakeup(UI_8 service_id);
//static void RequestCanVSleep(void);

/* -------------------------------- Routines -------------------------------- */


/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine to initialize the power management related FSM
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/ 
void PowerMngInicialitza(void)
{
    PowerMngControllerInitialize();
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Task that executes the power management related FSM
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void PowerMng(void)
{
    t_ignition_status ign_state_new = ReadVehicleIgnitionStatus();
    UI_16 delayed_shutdown_value = PowerMngGetDelayedShutdownValue();
    UI_8 EOL_state_new = GetTCUEOL();

    /* Cancel update service on request when TCU is not in EOL mode */
    if (EOL_state_new != TCU_EOL_MODE) {
        PowerMngSetDelayedShutdownValue( (UI_16) 0 );
        UpdateServiceOnRequest( SERV_POWERMANAGER, FALSE, (UI_16) 0 );
    }
    else {
        /* Refresh power on if ignition state is still ON */
        if ((ign_state_new == IGNITION_ON) && (delayed_shutdown_value > (UI_16) 0)) {
            UpdateServiceOnRequest( SERV_POWERMANAGER, TRUE, delayed_shutdown_value );
        }
    }

    PowerMngController();
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting service TCU ON request.
|---------------------------------------------------------------------------
| Parameters description:
|  service_id:  Requestor service id
|  status:      Status of the request
|  tout_secs:   Max timeout for the request
/---------------------------------------------------------------------------*/
void UpdateServiceOnRequest(UI_8 service_id,BOOL status, UI_16 tout_secs)
{
    if(service_id < N_SERVICES){
        /* Enable service ON */
        if(status == TRUE) {
            /* Set flag of specified service */
            services_tcuon_sts[service_id] = tout_secs;
        }
        /* Disable Service DRX */
        else {
            /* Clear flag of specified service */
            services_tcuon_sts[service_id] = (UI_16)0;
        }
    }
}

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
void UpdateServiceNaviOnRequest(UI_8 service_id, BOOL status)
{
    UI_8 n_byte,n_bit;

    /* Calculate byte index in flag array */
    n_byte = service_id / 8;
    /* calculate bit position inside byte */
    n_bit = service_id % 8;

    /* Request Navi ON */
    if(status == TRUE) {
        /* Set flag of specified service */
        services_navion_sts[n_byte] |= (UI_8)(((UI_8)0x01)<<n_bit);
    }
    /* Service no longer needs Navi ON */
    else {
        /* Clear flag of specified service */
        services_navion_sts[n_byte] &= (UI_8)(~((UI_8)(((UI_8)0x01)<<n_bit)));
    }
}

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
void UpdateServiceMonitoringRequest( UI_8 service_id,BOOL req)
{
    PowerMngUpdateServiceMonitoringRequest(service_id,req);
}
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  * Routine responsible of updating service DRX mode request in next off conditions
//|---------------------------------------------------------------------------
//| Parameters description:
//|  service_id: Requestor service id
///---------------------------------------------------------------------------*/
//void UpdateServiceDrxRequest(UI_8 service_id,t_drx_status status)
//{
//    UI_8 n_byte,n_bit;
//
//    /* Calculate byte index in flag array */
//    n_byte = service_id / (UI_8)4;
//    /* calculate bit position inside byte */
//    n_bit = ((service_id % (UI_8)4) * (UI_8)2);
//
//    /* Enable service DRX */
//    if(status == DRX_ACTIVATE) {
//        /* Clear flag of specified service */
//        services_drx_sts[n_byte] &= (UI_8)(~((UI_8)(((UI_8)0x03)<<n_bit)));
//        /* Set flag of specified service */
//        services_drx_sts[n_byte] |= (UI_8)(((UI_8)0x01)<<n_bit);
//
//    }/* Enable service DRX */
//    else if(status == SUDO_DRX_ACTIVATE) {
//        /* Set flag of specified service */
//        services_drx_sts[n_byte] |= (UI_8)(((UI_8)0x03)<<n_bit);
//    }/* Disable Service DRX */
//    else {
//
//        /* Clear flag of specified service */
//        services_drx_sts[n_byte] &= (UI_8)(~((UI_8)(((UI_8)0x03)<<n_bit)));
//    }
//}
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  * Routine responsible of updating service DRX mode request in next off conditions
//|---------------------------------------------------------------------------
//| Parameters description:
//|  service_id: Requestor service id
///---------------------------------------------------------------------------*/
void UpdateServiceGpsHotstartRequest(UI_8 service_id,BOOL status)
{
    UI_8 n_byte,n_bit;

    /* Calculate byte index in flag array */
    n_byte = service_id / 8;
    /* calculate bit position inside byte */
    n_bit = service_id % 8;

    /* Enable service DRX */
    if(status == TRUE) {
        /* Set flag of specified service */
        services_hotstart_sts[n_byte] |= (UI_8)(((UI_8)0x01)<<n_bit);
    }
    /* Disable Service DRX */
    else {
        /* Clear flag of specified service */
        services_hotstart_sts[n_byte] &= (UI_8)(~((UI_8)(((UI_8)0x01)<<n_bit)));
    }
}
//
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  * Routine responsible of updating service Disable Telematic Reset
//|     mode request in wake up from DRX
//|---------------------------------------------------------------------------
//| Parameters description:
//|  service_id: Requestor service id
///---------------------------------------------------------------------------*/
//void UpdateServiceDisableTelemDrxResetRequest(UI_8 service_id,BOOL status)
//{
//    UI_8 n_byte,n_bit;
//
//    /* Calculate byte index in flag array */
//    n_byte = service_id / 8;
//    /* calculate bit position inside byte */
//    n_bit = service_id % 8;
//
//    /* Enable service DRX */
//    if(status == TRUE) {
//        /* Set flag of specified service */
//        services_telem_drx_reset_request[n_byte] |= (UI_8)(((UI_8)0x01)<<n_bit);
//    }
//    /* Disable Service DRX */
//    else {
//        /* Clear flag of specified service */
//        services_telem_drx_reset_request[n_byte] &= (UI_8)(~((UI_8)(((UI_8)0x01)<<n_bit)));
//    }
//}
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  Routine to request an update of a service ID CAN_V wakeup status request. The
//|  different services that will use this API shall use it with an ID from 0 to
//|  (WAKEUP_RQS_MONITOR_BYTES - 1).
//|  The call of this routine will be ignored if the Sleep Management selected
//|  mode is different from "Complete mode".
//|---------------------------------------------------------------------------
//| Parameters description:
//|   service_id: identifier of the service that requests wakeup. These service IDs
//|               are defined by the user of the SleepManagement service
//|   status: flag that indicates if this service is needing the CAN active or not
//| Outputs:
///---------------------------------------------------------------------------*/
//void PowerMngCanVWakeupRequest(UI_8 service_id, BOOL status)
//{
//    UI_8 n_byte,n_bit;
//    BOOL wakeup_still_requested = FALSE;
//    UI_8 i;
//
//    /* Calculate byte index in flag array */
//    n_byte = service_id / 8;
//    /* calculate bit position inside byte */
//    n_bit = service_id % 8;
//
//    /* Check if requested ID is inside the valid range defined */
//    if(n_byte < WAKEUP_RQS_MONITOR_BYTES) {
//        if(status == TRUE) {
//            /* Check if bit of the service is still not set */
//            if((can_v_wake_up[n_byte] & (UI_8)(((UI_8)0x01)<<n_bit)) == 0) {
//                /* Set the bit of the request */
//                can_v_wake_up[n_byte] |= (UI_8)(((UI_8)0x01)<<n_bit);
//                /* Request wakeup */
//                RequestCanVWakeup(service_id);
//            }
//            else {
//                /* Do nothing, bit already set */
//            }
//        }
//        else {
//            /* Clear the bit of the request */
//            can_v_wake_up[n_byte]&= (UI_8)(~(UI_8)(((UI_8)0x01)<<n_bit));
//            /* Check if all the services request sleep */
//            for(i=0; i<WAKEUP_RQS_MONITOR_BYTES; i++) {
//                /* Check if there is any bit still set */
//                if(can_v_wake_up[i] != 0) {
//                    wakeup_still_requested = TRUE;
//                }
//            }
//            /* Check if all the services that could request wakeup are not */
//            /* requesting wakeup */
//            if(wakeup_still_requested == FALSE) {
//                /* Request CAN_V sleep */
//                RequestCanVSleep();
//            }
//            else {
//                /* Do nothing */
//            }
//        }
//    }
//    else {
//        /* Invalid service_id, do nothing */
//    }
//}
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  Routine to request an update of a service ID CAN_M wakeup status request. The
//|  different services that will use this API shall use it with an ID from 0 to
//|  (WAKEUP_RQS_MONITOR_BYTES - 1).
//|  The call of this routine will be ignored if the Sleep Management selected
//|  mode is different from "Complete mode".
//|---------------------------------------------------------------------------
//| Parameters description:
//|   service_id: identifier of the service that requests wakeup. These service IDs
//|               are defined by the user of the SleepManagement service
//|   status: flag that indicates if this service is needing the CAN active or not
//| Outputs:
///---------------------------------------------------------------------------*/
//void PowerMngCanMWakeupRequest(UI_8 service_id, BOOL status)
//{
//    UI_8 n_byte,n_bit;
//    BOOL wakeup_still_requested;
//    UI_8 i;
//
//    /* Calculate byte index in flag array */
//    n_byte = service_id / 8;
//    /* calculate bit position inside byte */
//    n_bit = service_id % 8;
//
//    /* Check if requested ID is inside the valid range defined */
//    if(n_byte < WAKEUP_RQS_MONITOR_BYTES) {
//        if(status == TRUE) {
//            /* Check if bit of the service is still not set */
//            if((can_m_wake_up[n_byte] & (UI_8)(((UI_8)0x01)<<n_bit)) == 0) {
//                /* Set the bit of the request */
//                can_m_wake_up[n_byte] |= (UI_8)(((UI_8)0x01)<<n_bit);
//                /* Request CAN sleep management slave wakeup */
//                SleepMngSlaveWakeupStsRequest(CAN_M_SMNG_HANDLER, TRUE);
//            }
//            else {
//                /* Do nothing, bit already set */
//            }
//        }
//        else {
//            /* Clear the bit of the request */
//            can_m_wake_up[n_byte]&= (UI_8)(~(UI_8)(((UI_8)0x01)<<n_bit));
//            /* Check if all the services request sleep */
//            for(i=0; i<WAKEUP_RQS_MONITOR_BYTES; i++) {
//                /* Check if there is any bit still set */
//                if(can_m_wake_up[i] != 0) {
//                    wakeup_still_requested = TRUE;
//                }
//            }
//            /* Check if all the services that could request wakeup are not */
//            /* requesting wakeup */
//            if(wakeup_still_requested == FALSE) {
//                /* Request CAN sleep management slave wakeup */
//                SleepMngSlaveWakeupStsRequest(CAN_M_SMNG_HANDLER, FALSE);
//            }
//            else {
//                /* Do nothing */
//            }
//        }
//    }
//    else {
//        /* Invalid service_id, do nothing */
//    }
//}
//
//#ifdef NISSAN_VARIANT
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  Routine to request an update of a service ID CAN_M wakeup status request. The
//|  different services that will use this API shall use it with an ID from 0 to
//|  (WAKEUP_RQS_MONITOR_BYTES - 1).
//|  The call of this routine will be ignored if the Sleep Management selected
//|  mode is different from "Complete mode".
//|---------------------------------------------------------------------------
//| Parameters description:
//|   service_id: identifier of the service that requests wakeup. These service IDs
//|               are defined by the user of the SleepManagement service
//|   status: flag that indicates if this service is needing the CAN active or not
//| Outputs:
///---------------------------------------------------------------------------*/
//void PowerMngCanEVWakeupRequest(UI_8 service_id, BOOL status)
//{
//    UI_8 n_byte,n_bit;
//    BOOL wakeup_still_requested;
//    UI_8 i;
//
//    /* Calculate byte index in flag array */
//    n_byte = service_id / 8;
//    /* calculate bit position inside byte */
//    n_bit = service_id % 8;
//
//    /* Check if requested ID is inside the valid range defined */
//    if(n_byte < WAKEUP_RQS_MONITOR_BYTES) {
//        if(status == TRUE) {
//            /* Check if bit of the service is still not set */
//            if((can_ev_wake_up[n_byte] & (UI_8)(((UI_8)0x01)<<n_bit)) == 0) {
//                /* Set the bit of the request */
//                can_ev_wake_up[n_byte] |= (UI_8)(((UI_8)0x01)<<n_bit);
//
//                /* Request CAN EV wakeup */
//                can_ev_wakeup_req = TRUE;
//
//                /* Check if current EV CAN status is sleep or error*/
//                if((can_ev_wakeup_sts == POWERMNG_CAN_STS_SLEEP) ||
//                   (can_ev_wakeup_sts == POWERMNG_CAN_STS_ERROR)) {
//                    /* Update the CAN EV status to awaking */
//                    can_ev_wakeup_sts = POWERMNG_CAN_STS_WAKING_UP;
//                }
//                else {
//                    /* Do nothing */
//                }
//            }
//            else {
//                /* Do nothing, bit already set */
//            }
//        }
//        else {
//            /* Clear the bit of the request */
//            can_ev_wake_up[n_byte]&= (UI_8)(~(UI_8)(((UI_8)0x01)<<n_bit));
//            /* Check if all the services request sleep */
//            for(i=0; i<WAKEUP_RQS_MONITOR_BYTES; i++) {
//                /* Check if there is any bit still set */
//                if(can_ev_wake_up[i] != 0) {
//                    wakeup_still_requested = TRUE;
//                }
//            }
//            /* Check if all the services that could request wakeup are not */
//            /* requesting wakeup */
//            if(wakeup_still_requested == FALSE) {
//                /* Request CAN EV sleep */
//                can_ev_wakeup_req = FALSE;
//            }
//            else {
//                /* Do nothing */
//            }
//        }
//    }
//    else {
//        /* Invalid service_id, do nothing */
//    }
//}
//#endif
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  Routine to check the wakeup status of the CAN_V bus
//|---------------------------------------------------------------------------
//| Parameters description:
//| Outputs:
//|    wakeup_sts: POWERMNG_CAN_STS_SLEEP = CAN_V bus is sleep
//|                POWERMNG_CAN_STS_AWAKE = CAN_V bus is awake
//|                POWERMNG_CAN_STS_WAKING_UP = TCU is trying to awake CAN_V bus
//|                POWERMNG_CAN_STS_ERROR = CAN_V bus could not be awaken by TCU
///---------------------------------------------------------------------------*/
//t_powermng_can_wakeup_sts PowerMngCanVWakeupStatus(void)
//{
//    t_powermng_can_wakeup_sts power_can_sts;
//    t_smng_sts sleepmng_sts;
//
//    /* Get status of sleep management */
//    sleepmng_sts = SleepMngGetSts(CAN_V_SMNG_HANDLER);
//
//    /* Check if platform wakes up by Output ON request or by CAN */
//    if(GetDynamicSWConfig(DYN_SERV_WUOOR)==TRUE){
//        if(outputonreq_waking == TRUE) {
//            power_can_sts = POWERMNG_CAN_STS_WAKING_UP;
//        }
//        else if(outputonreq_error == TRUE) {
//            power_can_sts = POWERMNG_CAN_STS_ERROR;
//        }
//        else if(sleepmng_sts == SMNG_STS_AWAKE) {
//            power_can_sts = POWERMNG_CAN_STS_AWAKE;
//        }
//        else {
//            power_can_sts = POWERMNG_CAN_STS_SLEEP;
//        }
//    }
//    else {
//        /* Check status returned by sleep management */
//        if((sleepmng_sts == SMNG_STS_SLEEP_CONFIRMED) || (sleepmng_sts == SMNG_STS_SLEEP_TRANSIENT)) {
//            power_can_sts = POWERMNG_CAN_STS_SLEEP;
//        }
//        else if(sleepmng_sts == SMNG_STS_AWAKE) {
//            power_can_sts = POWERMNG_CAN_STS_AWAKE;
//        }
//        else if(sleepmng_sts == SMNG_STS_WAKING_UP) {
//            power_can_sts = POWERMNG_CAN_STS_WAKING_UP;
//        }
//        else {
//            power_can_sts = POWERMNG_CAN_STS_ERROR;
//        }
//    }
//
//    return power_can_sts;
//}
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  Routine to check the wakeup status of the CAN_M bus
//|---------------------------------------------------------------------------
//| Parameters description:
//| Outputs:
//|    wakeup_sts: POWERMNG_CAN_STS_SLEEP = CAN_M bus is sleep
//|                POWERMNG_CAN_STS_AWAKE = CAN_M bus is awake
//|                POWERMNG_CAN_STS_WAKING_UP = TCU is trying to awake CAN_M bus
//|                POWERMNG_CAN_STS_ERROR = CAN_M bus could not be awaken by TCU
///---------------------------------------------------------------------------*/
//t_powermng_can_wakeup_sts PowerMngCanMWakeupStatus(void)
//{
//    t_powermng_can_wakeup_sts power_can_sts;
//    t_smng_sts sleepmng_sts;
//
//    /* Get status of sleep management */
//    sleepmng_sts = SleepMngGetSts(CAN_M_SMNG_HANDLER);
//
//    /* Check status returned by sleep management */
//    if((sleepmng_sts == SMNG_STS_SLEEP_CONFIRMED) || (sleepmng_sts == SMNG_STS_SLEEP_TRANSIENT)) {
//        power_can_sts = POWERMNG_CAN_STS_SLEEP;
//    }
//    else if(sleepmng_sts == SMNG_STS_AWAKE) {
//        power_can_sts = POWERMNG_CAN_STS_AWAKE;
//    }
//    else if(sleepmng_sts == SMNG_STS_WAKING_UP) {
//        power_can_sts = POWERMNG_CAN_STS_WAKING_UP;
//    }
//    else {
//        power_can_sts = POWERMNG_CAN_STS_ERROR;
//    }
//
//    return power_can_sts;
//}
//
//#ifdef NISSAN_VARIANT
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  Routine to check the wakeup status of the CAN_M bus
//|---------------------------------------------------------------------------
//| Parameters description:
//| Outputs:
//|    wakeup_sts: POWERMNG_CAN_STS_SLEEP = CAN_M bus is sleep
//|                POWERMNG_CAN_STS_AWAKE = CAN_M bus is awake
//|                POWERMNG_CAN_STS_WAKING_UP = TCU is trying to awake CAN_M bus
//|                POWERMNG_CAN_STS_ERROR = CAN_M bus could not be awaken by TCU
///---------------------------------------------------------------------------*/
//t_powermng_can_wakeup_sts PowerMngCanEVWakeupStatus(void)
//{
//    return can_ev_wakeup_sts;
//}
//#endif
//
///*---------------------------------------------------------------------------
//| Portability:
//|----------------------------------------------------------------------------
//| Routine description:
//|  * Checks if wakeup was done by movement sensor
//|---------------------------------------------------------------------------
//| Parameters description:
//|  result: TRUE if RFOFF mode is enabled (TODO:RLD:move to powerMngFunctions)
///---------------------------------------------------------------------------*/
//BOOL IsWakeupCauseByMove(void)
//{
//    t_sig_rtc_tx_aco_int_sts sig_rtc_tx_aco_int_sts;
//    t_sig_pmc_tx_update_wakeup_causes wakeup_cause;
//    /* RLD: This signal seems to work by level instead by interrupt acivation*/
//    InternComOsekReceiveMessage(SIG_RTC_TX_ACO_INT_STS,(t_application_data_ref) &sig_rtc_tx_aco_int_sts);
//    InternComOsekReceiveMessage(SIG_RTC_TX_WAKE_UP_CAUSE,(t_application_data_ref)&wakeup_cause);
//
//    return (sig_rtc_tx_aco_int_sts == SIG_RTC_TX_ACO_INT_STS_NOT_ACTIVE);
//}
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  * Write in ISO 15765_3 buffer the service drx status table
//|---------------------------------------------------------------------------
//| Parameters description:
///---------------------------------------------------------------------------*/
//void PMWriteServicesDrxStatus(void)
//{
//    tp_uds_read_data_by_identifier_resp resp;
//    UI_8 i;
//
//    if(N_SERV_DRX_FLAGS <= TP_DIAG_RX_TX_LEN){
//        /* Get the var that gives access to the answer data buffer	*/
//        resp = ISO15765_3_GET_RESP_DATA(tp_uds_read_data_by_identifier_resp);
//
//        for(i = (UI_8)0;i<N_SERV_DRX_FLAGS; i++) {
//            resp->buffer_dades[i] = services_drx_sts[i];
//        }
//    }
//}
//
/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Write in ISO 15765_3 buffer the service on tx status table
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void PMWriteServicesOnTxStatus(void)
{
    tp_uds_read_data_by_identifier_resp resp;
    UI_8 i;

    if(N_SERV_FLAGS <= TP_DIAG_RX_TX_LEN){
        /* Get the var that gives access to the answer data buffer	*/
        resp = ISO15765_3_GET_RESP_DATA(tp_uds_read_data_by_identifier_resp);

        for(i = (UI_8)0;i<N_SERV_FLAGS; i++) {
            resp->buffer_dades[i] = services_tcuon_sts[i];
        }
    }
}

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
t_pmc_power_mode PMGetPowerMode(void)
{
    return pmc_power_mode;
}

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
|           PM_STATUS_CURRENT (avalilabe only for MICRO_PMC)
|           PM_STATUS_ON      (available for both micros)
|           PM_STATUS_OFF     (available for both micros)
|           PM_STATUS_DRX     (available only for MICRO_PMC)
|
/---------------------------------------------------------------------------*/
void PMSetRtcPowerMode(t_pm_micro_requester micro, t_pm_req_status status)
{
    t_pm_rtc_status rtc_status = PM_RTC_UNKNOWN;

    /* Order from PMC - MASTER */
    if(micro == (t_pm_micro_requester)MICRO_PMC){
        switch (status){
            case (t_pm_req_status)PM_STATUS_CURRENT:   /* Current req */
                pm_request = SIG_PMC_TX_REQ_POWER_MODE_KEEP_CUR;
                break;
            case (t_pm_req_status)PM_STATUS_ON:    /* ON req */
                /* Read rtc status */
                rtc_status = PMReadRtcStatus();
                if((rtc_status == PM_RTC_DRX)        ||
                   (rtc_status == PM_RTC_DRX_PMC_ON) ){

                    pm_request = SIG_PMC_TX_REQ_POWER_MODE_TCU_ON;
                }
                else if(pm_request != SIG_PMC_TX_REQ_POWER_MODE_TCU_ON){
                    pm_request = SIG_PMC_TX_REQ_POWER_MODE_TCU_ON;
                }
                else{
                    /* Do Nothing */
                }
                break;
            case (t_pm_req_status)PM_STATUS_OFF: /* OFF req */
                pm_request = SIG_PMC_TX_REQ_POWER_MODE_TCU_OFF;
                break;
            case (t_pm_req_status)PM_STATUS_DRX: /*  DRX req */
                pm_request = SIG_PMC_TX_REQ_POWER_MODE_DRX;
                break;
            default:
                /* NOT ACCEPTED */
                break;
        }
    }
    /* Order from Telematic - SLAVE */
    else if(micro == (t_pm_micro_requester)MICRO_TELEM){
        switch (status){
            case (t_pm_req_status)PM_STATUS_ON:  /* On req */
                if((pm_request ==SIG_PMC_TX_REQ_POWER_MODE_PMC_ON) || (pm_request == SIG_PMC_TX_REQ_POWER_MODE_KEEP_CUR)){
                    pm_request = SIG_PMC_TX_REQ_POWER_MODE_TCU_ON;
                }
                break;
            case (t_pm_req_status)PM_STATUS_OFF: /* Off req */
                if((pm_request == SIG_PMC_TX_REQ_POWER_MODE_TCU_ON) || (pm_request == SIG_PMC_TX_REQ_POWER_MODE_KEEP_CUR)){
                    pm_request = SIG_PMC_TX_REQ_POWER_MODE_PMC_ON;
                }
                break;
            case (t_pm_req_status)PM_STATUS_CURRENT: /* Not Accepted */
            case (t_pm_req_status)PM_STATUS_DRX:   /* Not Accepted */
            default:
                /* NOT ACCEPTED */
                break;
        }
    }
    else{
        /* ERROR */
    }

    /* Send request to RTC */
    InternComOsekSendMessage(SIG_PMC_TX_REQ_POWER_MODE,
                            (t_application_data_ref) &pm_request);
}

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
t_pm_rtc_status PMReadRtcStatus(void)
{
    t_sig_rtc_tx_cur_power_mode pm_status;

    /* Read signal from RTC */
    InternComOsekReceiveMessage(SIG_RTC_TX_CUR_POWER_MODE,(t_application_data_ref) &pm_status);

    /* Check signal */
    switch (pm_status){
        case SIG_RTC_TX_CUR_POWER_MODE_TCU_OFF:
            last_rtc_status = PM_RTC_TCU_OFF;
            break;
        case SIG_RTC_TX_CUR_POWER_MODE_PMC_ON:
            last_rtc_status = PM_RTC_PMC_ON;
            break;
        case SIG_RTC_TX_CUR_POWER_MODE_TCU_ON:
            last_rtc_status = PM_RTC_TCU_ON;
            break;
        case SIG_RTC_TX_CUR_POWER_MODE_DRX:
            last_rtc_status = PM_RTC_DRX;
            break;
        case SIG_RTC_TX_CUR_POWER_MODE_DRX_PMC_ON:
            last_rtc_status = PM_RTC_DRX_PMC_ON;
            break;
        default:
            last_rtc_status = PM_RTC_UNKNOWN;
            break;
    }

    return last_rtc_status;
}

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
t_interface_status PMReadTelemStatus(void)
{
    return TMReadStatus();
}

BOOL PowerMngGetVBatInEmergencySts( void )
{
	t_sig_rtc_tx_bat_emergency_sts v_bat_emergency_sts;
	BOOL ret = FALSE;

	(void) InternComOsekReceiveMessage( SIG_RTC_TX_BAT_EMERGENCY_STS, (t_application_data_ref) &v_bat_emergency_sts );

	if ( v_bat_emergency_sts == SIG_RTC_TX_BAT_EMERGENCY_STS_EMERGENCY ) {
		ret = TRUE;
	}
	else if ( v_bat_emergency_sts == SIG_RTC_TX_BAT_EMERGENCY_STS_NO_EMERGENCY ) {
		ret = FALSE;
	}
	else {
		/* Do Nothing */
		/* This should never happen */
	}
	return ret;
}

///*---------------------------------------------------------------------------
//| Portability: Generic
//|----------------------------------------------------------------------------
//| Routine description:
//|
//|  Routine responsible of putting services_on_re, drx_req, hotstart_req and
//|  drx_rst_req into PMC_COMMON message to debug its values.
//|
//|  Telematic module doesn't use this information. Only for debug purposes.
//|---------------------------------------------------------------------------
//| Parameters description:
//|  param_name: none
//|  result: none.
///---------------------------------------------------------------------------*/
//void UpdateDebugPowerSignals(void)
//{
//    UI_8 i = 0;
//    t_sig_pmc_common_dbg_service_on_req serv;
//    t_sig_pmc_common_dbg_service_drx_req drx;
//    t_sig_pmc_common_dbg_service_drx_rst_req rst;
//    t_sig_pmc_common_dbg_service_hotstart_req hot;
//
//    /**/
//    for(i=0;i<N_SERV_FLAGS;i++)
//    {
//        serv[i] = services_tcuon_sts[i];
//        rst[i] = services_telem_drx_reset_request[i];
//        hot[i] = services_hotstart_sts[i];
//    }
//
//    for(i=N_SERV_FLAGS;i<10;i++)
//    {
//        serv[i] = 0xFF;
//        rst[i] = 0xFF;
//        hot[i] = 0xFF;
//    }
//
//    for(i=0;i<N_SERV_DRX_FLAGS;i++)
//    {
//        drx[i] = services_drx_sts[i];
//    }
//    for(i=N_SERV_DRX_FLAGS;i<10;i++)
//    {
//        drx[i] = 0xFF;
//    }
//
//    InternComOsekSendMessage(SIG_PMC_COMMON_DBG_SERVICE_ON_REQ,(t_application_data_ref)&serv);
//    InternComOsekSendMessage(SIG_PMC_COMMON_DBG_SERVICE_DRX_REQ,(t_application_data_ref)&drx);
//    InternComOsekSendMessage(SIG_PMC_COMMON_DBG_SERVICE_DRX_RST_REQ,(t_application_data_ref)&rst);
//    InternComOsekSendMessage(SIG_PMC_COMMON_DBG_SERVICE_HOTSTART_REQ,(t_application_data_ref)&hot);
//}
//
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  Routine to request a wakeup on CAN V
//|---------------------------------------------------------------------------
//| Parameters description:
//| Outputs:
///---------------------------------------------------------------------------*/
//static void RequestCanVWakeup(UI_8 service_id)
//{
//    /* Check if platform wakes up by Output ON request or by CAN */
//    if(GetDynamicSWConfig(DYN_SERV_WUOOR)==TRUE){
//        /* Request output on request activation */
//        outputonreq_requested = TRUE;
//        /* Clear any previous wakeup error in order to force a new trial */
//        outputonreq_error = FALSE;
//        /* Request CAN sleep management slave wakeup in order to send the */
//        /* refuse sleep signal on CAN bus to REFUSE_TO_SLEEP value */
//        SleepMngSlaveWakeupStsRequest(CAN_V_SMNG_HANDLER, TRUE);
//    }
//    /* Platform wakes up BCM by CAN sleep management request */
//    else{
//        /* Request CAN sleep management slave wakeup */
//        SleepMngSlaveWakeupStsRequest(CAN_V_SMNG_HANDLER, TRUE);
//    }
//}
//
///*****************************************************************************
//| Portability: General
//|----------------------------------------------------------------------------
//| Routine description:
//|  Routine to request sleep on CAN V
//|---------------------------------------------------------------------------
//| Parameters description:
//| Outputs:
///---------------------------------------------------------------------------*/
//static void RequestCanVSleep(void)
//{
//    /* In case of Wake up by OOR, nothing to do cause the */
//    /* flag is managed in OORMngr */
//    if(GetDynamicSWConfig(DYN_SERV_WUOOR)==TRUE){
//        /* Clear CAN-V output on request wakeup */
//        outputonreq_requested = FALSE;
//        /* Request CAN sleep management slave wakeup in order to send the */
//        /* refuse sleep signal on CAN bus to ACCEPT_SLEEP value */
//        SleepMngSlaveWakeupStsRequest(CAN_V_SMNG_HANDLER, FALSE);
//    }
//    /* Platform wakes up BCM by CAN */
//    else {
//        SleepMngSlaveWakeupStsRequest(CAN_V_SMNG_HANDLER, FALSE);
//    }
//}
//
//
//
//
//
////DEPRECATED by PowerMngCanVWakeupRequest
//void RequestBCMWakeUp(UI_8 service_id)
//{
//    PowerMngCanVWakeupRequest(service_id, TRUE);
//}
//
////DEPRECATED by PowerMngCanVWakeupRequest
//void EndRequestBCMWakeUp(UI_8 service_id)
//{
//    PowerMngCanVWakeupRequest(service_id, FALSE);
//}
//
////DEPRECATED by PMGetPowerMode
//BOOL PowerMngIsGoingToOff(void)
//{
//    BOOL PMIGTO_ret = FALSE;
//
//    if((pmc_power_mode == PMC_PM_OFF_STARTED)
//        ||(pmc_power_mode == PMC_PM_OFF_STORE_DTCS)
//        ||(pmc_power_mode == PMC_PM_OFF_CONFIRMED)){
//        PMIGTO_ret = TRUE;
//    }
//
//    return PMIGTO_ret;
//}
//
//#ifdef NISSAN_VARIANT
////DEPRECATED by PowerMngCanEVWakeupStatus
//BOOL EvNissanIsAwake(void)
//{
//    BOOL ev_nissan_is_awake;
//
//    if(can_ev_wakeup_sts == POWERMNG_CAN_STS_AWAKE) {
//        ev_nissan_is_awake = TRUE;
//    }
//    else {
//        ev_nissan_is_awake = FALSE;
//    }
//
//    return ev_nissan_is_awake;
//}
//
////DEPRECATED by PowerMngCanEVWakeupRequest
//void EvNissanWakeUpRequest(UI_8 service_id)
//{
//    PowerMngCanEVWakeupRequest(service_id, TRUE);
//}
//
////DEPRECATED by PowerMngCanEVWakeupRequest
//void EndEvNissanWakeUpRequest(UI_8 service_id)
//{
//    PowerMngCanEVWakeupRequest(service_id, FALSE);
//}
//#endif

UI_8 ReadVehicleIgnitionStatus (void)
{
    return IgnitionGetStatus();
}

UI_8 ReadIntMixStatus (void)
{

    return IntMixRtcStatus();

}
