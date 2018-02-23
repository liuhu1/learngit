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
#include "PowerMng.h"
#include "InternComOsek.h"
#include "FicOsekCom.h"
#include "TcuGeneralFunctions.h"
#include "InfrastructureCFG.h"
#include "TcuInOutFunctions.h"
#include "SleepMngSimple.h"
#include "Iso15765_3_SpecC.h"
#include "DiagComMng.h"

#include "GpsNMEA.h"
#include "ACODriver.h"
#include "IgnitionFilterFunctions.h"
#include "PowerMngController.h"

#ifdef DEBUG_COMP
#include "debug.h"
#endif
/* -------------------------------- Defines --------------------------------- */
#define N_SERVICES_MAX      (UI_8)10

#define SERVICES_TCU_ON_MAX_TIME        (UI_16)600
/* ------------------------------- Data Types ------------------------------- */

/* ---------------------------- Global Variables ---------------------------- */

/* Vector of flags to notify which service requires TCU ON */
UI_16 services_tcuon_sts[N_SERVICES];
UI_16 services_tcuon_max_time = (UI_16)0;

/* Vector of flags to notify which service requires Navi ON */
UI_8 services_navion_sts[N_ACCOUT_SERV_FLAGS];

/* Vector of flags to notify which service requires DRX mode */
UI_8 services_drx_sts[N_SERV_DRX_FLAGS];

/* Vector of flags to notify which service requires GPS hotstart mode */
UI_8 services_hotstart_sts[N_SERV_FLAGS];

/* Vector of flags to notify which service requires the telematic drx reset */
UI_8 services_telem_drx_reset_request[N_SERV_FLAGS];

/* Flag to notify that DRX status has been updated in eeprom */
BOOL drx_status_updated = FALSE;

t_pm_req_status rtc_off_pm_request = PM_STATUS_CURRENT;

/* Flags for wakeup signal reception notification */
static BOOL flag_wakeup_causes_rx = FALSE;

/* Tcu current power mode */
t_pmc_power_mode pmc_power_mode = PMC_PM_INIT;

/* rtc power mode last request */
t_sig_pmc_tx_req_power_mode pm_request = SIG_PMC_TX_REQ_POWER_MODE_KEEP_CUR;

#if 0
/* Management of the wake up CAN_V requests */
UI_8 can_v_wake_up[WAKEUP_RQS_MONITOR_BYTES];

/* Management of the wake up CAN_M requests */
UI_8 can_m_wake_up[WAKEUP_RQS_MONITOR_BYTES];

/* Management of the wake up CAN_EV requests */
UI_8 can_ev_wake_up[WAKEUP_RQS_MONITOR_BYTES];
#endif

/* Current CAN EV wakeup requested status */
BOOL can_ev_wakeup_req = FALSE;

/* Current CAN EV bus status */
t_powermng_can_wakeup_sts can_ev_wakeup_sts = POWERMNG_CAN_STS_SLEEP;

/* Output on request wakeup status requested */
BOOL outputonreq_requested = FALSE;

/* Output on request waking status flag */
BOOL outputonreq_waking = FALSE;

/* Output on request error flag */
BOOL outputonreq_error = FALSE;

/* Force Telem Off flag */
BOOL powermng_force_telem_off_flag = FALSE;

/* Value to set on Update Service on Request When Going to Ignition OFF */
static UI_16 power_delayed_shutdown_value = (UI_16)0;

/* Vector of flags to notify which service requires monitoring ON */
UI_8 service_monitoring_on_req[N_SERV_FLAGS];

/* --------------------------- Routine prototypes --------------------------- */
static void PowerMngWakeCanTransceiver(void);
/* -------------------------------- Routines -------------------------------- */

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking different WAKE-UP causes and request
|    power management to be in ON in case any CAN is awakened. It is also
|    responsible of requesting to sleep management the current status of
|    master_can_wakeup_req.
|---------------------------------------------------------------------------
| Parameters description:
|  
/---------------------------------------------------------------------------*/
BOOL CheckTcuOnUpdateSleepMngConditions(void)
{

    BOOL tcu_on_req;
    
#ifdef CAN_V_SMNG_HANDLER
    
    BOOL wake_can_v;

#endif /* CAN_V_SMNG_HANDLER */

#ifdef CAN_M_SMNG_HANDLER

    BOOL wake_can_m;

#endif /* CAN_M_SMNG_HANDLER */

    UI_8 ignition_physical = IGNITION_PHY_OFF;
    /*t_wakeup_by_can sleepmng_wakeup_v;
    t_wakeup_by_can sleepmng_wakeup_m;*/

    /*t_sig_pmc_tel_crtl_car_wake_up sig_pmc_tel_crtl_car_wake_up;*/
#ifdef NISSAN_VARIANT
    UI_8 can_map_config;
    UI_8 acc_sts;
    BOOL acc_err;
#endif
    BOOL monitoring_on_req;

    /* Check if any service requested monitoring ON */
    monitoring_on_req = CheckServiceMonitoringRequest();
    
#if 0
    /* Get CAN sleep management master wakeup requested status of CAN_V */
    sleepmng_wakeup_v = SleepMngGetMasterWakeupByCanStsReq(CAN_V_SMNG_HANDLER);
    /* Get CAN sleep management master wakeup requested status of CAN_M */
    sleepmng_wakeup_m = SleepMngGetMasterWakeupByCanStsReq(CAN_M_SMNG_HANDLER);
#endif
#if (defined(VOLVO_VARIANT) || defined(HONDA_VARIANT))
    UI_8 diag_session;
    /* Get diagnostic session */
    diag_session = GetSessionIdentifier();
#endif

#if (defined(NISSAN_VARIANT) ||  defined(VOLVO_VARIANT) || defined(HONDA_VARIANT))
    /* Get CAN map configuration */
    /*can_map_config = GetNissanVCanConfig();*/

    ignition_physical = IgnitionGetStatus();

    /* Check Tcu ON Conditions */
    tcu_on_req = (/*(sleepmng_wakeup_v == WAKEUP_BY_CAN_TRUE) || */
                  (diag_session != UDS_DEFAULT_SESSION) ||
                  (ignition_physical == IGNITION_PHY_ON) /*||
                  (wake_up_by_can == TRUE)||
                  (wakeup_phy_sts == WAKEUP_PHY_ON)*/);
    
#ifdef CAN_V_SMNG_HANDLER

    /* CAN_V only depend of CAN_V wakeup frame */
    wake_can_v = (/*(sleepmng_wakeup_v == WAKEUP_BY_CAN_TRUE) || */
                  (diag_session != UDS_DEFAULT_SESSION) ||
                  (ignition_physical == IGNITION_PHY_ON) /*||
                  (wake_up_by_can == TRUE)||
                  (wakeup_phy_sts == WAKEUP_PHY_ON)*/||
                  (monitoring_on_req == TRUE));

#endif /* CAN_V_SMNG_HANDLER */

#ifdef CAN_M_SMNG_HANDLER

	/* Set CAN-M wakeup according to CAN-V wakeup status */
    wake_can_m = wake_can_v;

#endif /* CAN_M_SMNG_HANDLER */

#if 0
    if ((pmc_manual_ecall == SIG_PMC_TEL_CRTL_MANUAL_ECALL_ACTIVATED) ||
       (pmc_automatic_ecall == SIG_PMC_TEL_CRTL_AUTOMATIC_ECALL_ACTIVATED)) {
        wake_can_m = TRUE;
    }
    else {
        /* Check if CAN map 3 generation is configured */
        if((can_map_config == GEN3_CAN_MAP) || 
          ((can_map_config == GEN5_CAN_MAP) && (wake_can_v == TRUE)) ) {
            /* CAN-M wakeup deppend on physical ACC wire status */
            acc_err = ReadAcc(&acc_sts);
            /* Check if ACC is received correctly*/
            if((acc_err == TRUE) || (acc_sts == SIG_AUTO_ACC_STA_OFF)) {
                /* Set that CAN-M should be OFF due to error in reception of the signal or received to OFF */
                wake_can_m = FALSE;
            }
            else {
                /* Set that CAN-M should be ON */
                wake_can_m = TRUE;
            }
        }
        else{
            /* Set CAN_M wakeup to off */
            wake_can_m = FALSE;
        }
    }

#endif

#elif defined(RENAULT_GEN2_VARIANT)

    
    /* Check if current variant is the EOL */
    if(curr_tcu_variant == VARIANT_EOL_RENAULT_GEN2) {
        /* Set as wakeup cause the ignition, CAN_V and a diagnostic session */
        wake_can_v = ((sleepmng_wakeup_v == WAKEUP_BY_CAN_TRUE) ||
                      (wakeup_phy_sts == WAKEUP_PHY_ON) ||
                      (diag_session != UDS_DEFAULT_SESSION));
        /* Set CAN-M wakeup as CAN-V */
        wake_can_m = wake_can_v;
    }
    else {
        /* Set as wakeup cause only the CAN_V and a diagnostic session different than default */
        wake_can_v = ((sleepmng_wakeup_v == WAKEUP_BY_CAN_TRUE) ||
                      (diag_session != UDS_DEFAULT_SESSION));
        /* Check if current variant is 15_40 which is the only one that have wakeup by CAN-M */
        if((curr_tcu_variant == VARIANT_JFC)||(curr_tcu_variant == VARIANT_JFC_SL8)||(curr_tcu_variant == VARIANT_XFA)||(curr_tcu_variant == VARIANT_XFB)||(curr_tcu_variant == VARIANT_HFE_GEN2) || (TCU_SW_VARIANT == VARIANT_HFE_SL8)) {
            wake_can_m = (sleepmng_wakeup_m == WAKEUP_BY_CAN_TRUE);
        } 
		else if((curr_tcu_variant == VARIANT_X07_D)) {
            if(ReadPhyWakeup() == PORT_ACTIVATED){
                wake_can_m = TRUE;
            } else {
                wake_can_m = wake_can_v;
            }                            
        } 
		else {
			/* Set CAN-M wakeup according to CAN-M wakeup status */
            wake_can_m = wake_can_v;
		}
    }

#elif defined(RENAULT_LC_VARIANT)
    
    /* Check if current variant is the EOL */
    if(curr_tcu_variant == VARIANT_EOL_RENAULT_LC) {
        /* Set as wakeup cause the ignition and CAN */
        wake_can_v = ((sleepmng_wakeup_v == WAKEUP_BY_CAN_TRUE) || 
                      (ignition_phy_sts == IGNITION_PHY_ON) ||
                      (diag_session != UDS_DEFAULT_SESSION));
        /* Set CAN-M wakeup as CAN-V */
        wake_can_m = wake_can_v; 
    }
    else {
        /* Set as wakeup cause only the CAN_V and a diagnostic session different than default */
        wake_can_v = ((sleepmng_wakeup_v == WAKEUP_BY_CAN_TRUE) ||
                      (diag_session != UDS_DEFAULT_SESSION));
        /* Set CAN-M wakeup as CAN-V */
        wake_can_m = wake_can_v; 
    }
#else
    #error "Define correctly the variant wakeup sources"
#endif

#ifdef CAN_V_SMNG_HANDLER

    /* Request to sleep management the current awaken status for CAN_V */
    SleepMngMasterWakeupStsRequest(CAN_V_SMNG_HANDLER, wake_can_v);

#endif /* CAN_V_SMNG_HANDLER */

#ifdef CAN_M_SMNG_HANDLER

    /* Request to sleep management the current awaken status for CAN_M */
    SleepMngMasterWakeupStsRequest(CAN_M_SMNG_HANDLER, wake_can_m);

#endif

    /* Check if any CAN is awaken to notify the status of the car to telematic */
#if 0
    if((wake_can_v == TRUE) || (wake_can_m == TRUE)) {
        /* Notify to GMV that BCM has awaken the CAN bus */
        sig_pmc_tel_crtl_car_wake_up = (UI_8)1;
        /* Notify telematic that car is waken up*/
        InternComOsekSendMessage(SIG_PMC_TEL_CRTL_CAR_WAKE_UP, (t_application_data_ref)&sig_pmc_tel_crtl_car_wake_up);
    }
    else {
        /* Do nothing */
    }
#endif

#if defined(CAN_V_SMNG_HANDLER) & defined (CAN_M_SMNG_HANDLER)

    return (wake_can_v | wake_can_m);

#else

#ifdef CAN_V_SMNG_HANDLER

    return tcu_on_req;

#endif /* CAN_V_SMNG_HANDLER */

#ifdef CAN_M_SMNG_HANDLER

    return wake_can_m;

#endif /* CAN_M_SMNG_HANDLER */

#endif /* defined(CAN_V_SMNG_HANDLER) & defined (CAN_M_SMNG_HANDLER) */
}

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
BOOL CheckRtcWakeupCauseConditions(void)
{
    t_sig_rtc_tx_wake_up_cause wake_up_cause;
    BOOL ret = TRUE;

    /* Check if the wakeup cause flag to see if signal has been received at least once*/
    if( flag_wakeup_causes_rx==FALSE ) {
        /* If it was not received yet, check now */
        if( InternComOsekReadFlagRxSig(SIG_RTC_TX_WAKE_UP_CAUSE_ZERO1)==COM_TRUE ) {
            /* Raise the flag if already received */
            flag_wakeup_causes_rx = TRUE;
        }
        else {
            /* Keep the flag */
            flag_wakeup_causes_rx = FALSE;
        }
    }
    else {
        /* Do Nothing */
    }
    
    if( flag_wakeup_causes_rx==TRUE ) {
        /* Get the RTC wakeup cause */
        InternComOsekReceiveMessage(SIG_RTC_TX_WAKE_UP_CAUSE, (t_application_data_ref)&wake_up_cause);

        /* Check if wakeup cause is due to an alarm expired or because telematic */
        /* requested ON being in DRX mode */
        ret = (((wake_up_cause & SIG_RTC_TX_WAKE_UP_CAUSE_TELEM) == SIG_RTC_TX_WAKE_UP_CAUSE_TELEM) ||
            ((wake_up_cause & SIG_RTC_TX_WAKE_UP_CAUSE_INT_MOV) == SIG_RTC_TX_WAKE_UP_CAUSE_INT_MOV) ||
            ((wake_up_cause & SIG_RTC_TX_WAKE_UP_CAUSE_ALARM0) == SIG_RTC_TX_WAKE_UP_CAUSE_ALARM0) ||
            ((wake_up_cause & SIG_RTC_TX_WAKE_UP_CAUSE_ALARM1) == SIG_RTC_TX_WAKE_UP_CAUSE_ALARM1) ||
            ((wake_up_cause & SIG_RTC_TX_WAKE_UP_CAUSE_ALARM2) == SIG_RTC_TX_WAKE_UP_CAUSE_ALARM2));
    }
    else {
        ret = TRUE;
    }
    
    return ret;
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting MonitoringOnRequest from services to initial value
|---------------------------------------------------------------------------
| Parameters description:
| 
/---------------------------------------------------------------------------*/
void InitServiceMonitoringRequest(void)
{
    UI_8 i = (UI_8)0;
    
	/*All the bytes defined for monitoring ON request initial*/
    for(i=0;i<N_SERV_FLAGS;i++) {
        service_monitoring_on_req[i] = (UI_8)0;
    }    
}

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
BOOL CheckServiceMonitoringRequest(void)
{
    UI_8 i = (UI_8)0;
    BOOL request = FALSE;
	static UI_32 temp_cnt = (UI_32)0;
	UI_8 buf[100];

    /* Check all the bytes defined for monitoring ON request*/
    for(i=0;i<N_SERV_FLAGS;i++) {
    	/* Check if any flag of the current byte is set */
        if(service_monitoring_on_req[i] != 0)
        {
            /* On has been request by some service */
	    request = TRUE;
		if(STATE_POWERMNGCONTROLLER_PMCMONITORING==PowerMngControllerGetState())
		{
				temp_cnt++;
				if(temp_cnt>=100000)
				{
					sprintf((char *)buf,"service_monitoring_on_req[%u] = %u\r\n", i, service_monitoring_on_req[i]);
					temp_cnt=0;
					PrintDebugString(buf);

				}
		}
	
	}
	else{
	    /* Do Nothing */
	}
    }

    return request;
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
void PowerMngUpdateServiceMonitoringRequest( UI_8 service_id,BOOL req)
{
    UI_8 n_byte,n_bit;

    /* Calculate byte index in flag array */
    n_byte = service_id / 8;
    /* calculate bit position inside byte */
    n_bit = service_id % 8;

    /* Service monitoring requestion on */
    if(req == TRUE) {
    /* Set flag of specified service */
        service_monitoring_on_req[n_byte] |= (UI_8)(((UI_8)0x01)<<n_bit);
    }
    /* Service monitoring requestion off */
    else {
    /* Clear flag of specified service */
        service_monitoring_on_req[n_byte] &= (UI_8)(~((UI_8)(((UI_8)0x01)<<n_bit)));
    }
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting TcuOnRequest from services to initial value
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
/---------------------------------------------------------------------------*/
void InitServiceTcuOnRequest(void)
{
    UI_8 i;
    
    for(i=0;i<N_SERVICES;i++) {
        services_tcuon_sts[i] = (UI_16)0;
    }
    
    services_tcuon_max_time = (UI_16)SERVICES_TCU_ON_MAX_TIME;
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of reinit the tcu_on max time in ONTX state
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
/---------------------------------------------------------------------------*/
void InitTcuOnMaxTime(void)
{
    services_tcuon_max_time = (UI_16)SERVICES_TCU_ON_MAX_TIME;
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting DRX mode from services to initial value
|---------------------------------------------------------------------------
| Parameters description:
|  service_id: Requestor service id
/---------------------------------------------------------------------------*/
void InitServiceDrxRequest(void)
{
    UI_8 i;
    
    for(i=0;i<N_SERV_DRX_FLAGS;i++) {
        services_drx_sts[i] = (UI_8)0;
    }    
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting GPS hotstart mode from services to initial 
|    value
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void InitGpsHotstartRequest(void)
{
    UI_8 i;
    
    for(i=0;i<N_SERV_FLAGS;i++) {
        services_hotstart_sts[i] = (UI_8)0;
    }    
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting Disable Telematic Drx Reset mode from 
|    services to initial value
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void InitServiceDisableTelemDrxResetRequest(void)
{
    UI_8 i;
    
    for(i=0;i<N_SERV_FLAGS;i++) {
        services_telem_drx_reset_request[i] = (UI_8)0;
    }    
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of setting no wakeup requests as initial value for 
|    different services
|---------------------------------------------------------------------------
| Parameters description:
/---------------------------------------------------------------------------*/
void InitCanWakeupRequests(void)
{
/*    UI_8 i;

    for(i=0;i<WAKEUP_RQS_MONITOR_BYTES;i++) {
        can_v_wake_up[i] = (UI_8)0;
        can_m_wake_up[i] = (UI_8)0;
        can_ev_wake_up[i] = (UI_8)0;
    }*/
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking if any service have requested TCU ON.
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested TCU ON
/---------------------------------------------------------------------------*/
BOOL CheckServiceTcuOnRequest(void)
{    
    UI_8 i;
    BOOL request = FALSE;
    
    if(services_tcuon_max_time > 0){
        for(i = 0; i < N_SERVICES; i++){
            if(services_tcuon_sts[i] != 0){
                request = TRUE;
            }
        }
    }
    else{
        /* Do nothing */
    }
    return request;
}


/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of decrease of timer for tcu on request 
|---------------------------------------------------------------------------
| Parameters description: NA
/---------------------------------------------------------------------------*/
void DecServiceTcuOnTout(void)
{
    UI_8 i;
    t_sig_pmc_common_services_on_request services_on;
    t_sig_pmc_common_num_services n_services = N_SERVICES;
    
    if(services_tcuon_max_time > 0){
        services_tcuon_max_time--;
    }
    
    for(i = 0; i < N_SERVICES; i++){
        if(i < N_SERVICES_MAX){
            services_on[i * 2] = services_tcuon_sts[i];
            services_on[(i * 2) + 1] = services_tcuon_sts[i] >> 8;
        }
        if(services_tcuon_sts[i] > 0){
            services_tcuon_sts[i]--;
        }
        else{
            /* Do nothing */
        }
    }
    
    /* Send to telematic the state of services */
    InternComOsekSendMessage(SIG_PMC_COMMON_NUM_SERVICES, (t_application_data_ref)&n_services);
    InternComOsekSendMessage(SIG_PMC_COMMON_SERVICES_ON_REQUEST, (t_application_data_ref)&services_on[0]);
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of checking if any service have requested DRX.
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested DRX
/---------------------------------------------------------------------------*/
t_drx_status CheckServiceDrxRequest(void)
{    
    UI_8 i,j;
    t_drx_status drx_request = DRX_DEACTIVATE;
    t_drx_status aux;

    /* Check all the bytes defined for ON services request*/
    for(i = (UI_8)0; (i < N_SERV_DRX_FLAGS) && (drx_request != (t_drx_status)SUDO_DRX_ACTIVATE); i++) {
        for (j = (UI_8)0; (j < (UI_8)8) && (drx_request != (t_drx_status)SUDO_DRX_ACTIVATE); j+=(UI_8)2){
            aux = (UI_8)(services_drx_sts[i] >> (UI_8)j) & (UI_8)0x03;
            if (aux == (t_drx_status)SUDO_DRX_ACTIVATE){
                drx_request = (t_drx_status)SUDO_DRX_ACTIVATE;
            }
            else if((aux == (t_drx_status)DRX_ACTIVATE) &&
                     (drx_request != (t_drx_status)SUDO_DRX_ACTIVATE)){
                drx_request = (t_drx_status)DRX_ACTIVATE;
            }
            else{
                /* Do Nothing */
            }
        }
    }
    
    return drx_request;
}

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
BOOL CheckServiceGpsHotstartRequest(void)
{    
    UI_8 i = (UI_8)0;
    BOOL hotstart_request = FALSE;

    /* Check all the bytes defined for ON services request*/
    for(i=0; i<N_SERV_FLAGS; i++) {
        /* Check if any flag of the current byte is set */
        if(services_hotstart_sts[i] != 0) {
            /* On has been request by some service */
            hotstart_request = TRUE;
        }
        else {
            /* Do nothing */
        }
    }
    
    return hotstart_request;
}


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
BOOL CheckServiceDisableTelemDrxResetRequest( void )
{    
    UI_8 i = (UI_8)0;
    BOOL disable_telem_drx_reset_request = FALSE;

    /* Check all the bytes defined for ON services request*/
    for(i=0; i<N_SERV_FLAGS; i++) {
        /* Check if any flag of the current byte is set */
        if(services_telem_drx_reset_request[i] != 0) {
            /* On has been request by some service */
            disable_telem_drx_reset_request = TRUE;
        }
        else {
            /* Do nothing */
        }
    }
    
    return disable_telem_drx_reset_request;
}

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
void DrxUpdateStatusCallback(void)
{
    drx_status_updated = TRUE;
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of waking up the external periphereals
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested TCU ON
/---------------------------------------------------------------------------*/
void ExtControllersWakeUp(void)
{
     /*ACF s'han de posar en les tasques corresponents*/
    ControlEnVAux(1);
    /* Activa GPS */
	gps_enable = TRUE;
    gps_hotstart = TRUE;
    PowerMngWakeCanTransceiver();

    /* Reset ACODriver */
    ACODriverReinit();

}

void GPSControllerWakeUp(void)
{
   /* Activa GPS */
	gps_enable = TRUE;
	gps_hotstart = FALSE;
}

static void PowerMngWakeCanTransceiver(void)
{
    UI_8 temp;
    /* Enable transceiver POWER ON */
    ControlNstbCanV(PORT_ACTIVATED);
    /* Enable the transceiver */
    ControlEnableCanV(PORT_ACTIVATED);

    /* Provoke a transition on wakeup pin */
    ControlWakeCanV(PORT_DEACTIVATED);
    for(temp=(UI_8)0;temp<(UI_8)255;temp++) {
    }
    ControlWakeCanV(PORT_ACTIVATED);
    for(temp=(UI_8)0;temp<(UI_8)255;temp++) {
    }
    ControlWakeCanV(PORT_DEACTIVATED);
}
/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine responsible of sleeping the external periphereals
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if any service has requested TCU ON
/---------------------------------------------------------------------------*/
void ExtControllersSleep(void)
{

    /* Request turn off of ACODriver */
    ACODriverStop();

#ifdef GEN2_PLATFORM
#if ((TCU_SW_PLATFORM == PLATFORM_2G) || (TCU_SW_PLATFORM == PLATFORM_3G_TCU3_SL8))
//    BOOL hotstart_requested;
    
    /* Case in which power mode requested will be DRX */
//    if(rtc_off_pm_request == (t_pm_req_status)PM_STATUS_DRX) {
//        /* Check if any service requested GPS in hotstart mode during DRX */
//        hotstart_requested = CheckServiceGpsHotstartRequest();
//        /* Check if hotstart has been requested */
//        if(hotstart_requested == TRUE) {
//            /* Request GPS hotstart mode */
//            gps_enable = TRUE;
//            gps_hotstart = TRUE;
//        }
//        else {
//            /* Request GPS power off */
      //      gps_enable = FALSE;
      //      gps_hotstart = FALSE;
//        }
//    }
//    else {
//    }
#endif
    /* Apaga spk */
    spk_hu_enable = FALSE;
     //ACF s'han de posar en les tasques corresponents
        ControlEnVAux(0);
#endif
}

void GPSControllerSleep(void)
{
	/* Request GPS power off */
	gps_enable = TRUE;
	gps_hotstart = TRUE;
}

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  * Routine to check if accelerometer is in sleep mode
|---------------------------------------------------------------------------
| Parameters description:
|   returns true if accelerometer confirmed sleep mode
/---------------------------------------------------------------------------*/
BOOL CheckExtControllersSleep(void)
{

    /* Check if ACODriver is stopped */
    return ACODriverIsStopped();

}


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
void PMSetFlagExitDrxSL8(BOOL flag)
{
    t_sig_rtc_tx_cur_power_mode pm_status;
    
    /* Read signal from RTC */
    InternComOsekReceiveMessage(SIG_RTC_TX_CUR_POWER_MODE,(t_application_data_ref) &pm_status);
    
    /* ONLY FOR SL8 */
#if (TCU_SW_PLATFORM == PLATFORM_3G_TCU3_SL8)
    /* Set pin high */
    if((flag == TRUE) && (pm_status == SIG_RTC_TX_CUR_POWER_MODE_TCU_ON)) {    
        ControlCoreModeWota(PORT_ACTIVATED);
        ControlEnUsb(PORT_ACTIVATED);
    }
    /*Set pin low */
    else{
        ControlCoreModeWota(PORT_DEACTIVATED);
        ControlEnUsb(PORT_DEACTIVATED);
    }
#endif
}

#ifdef DEBUG_COMP
void CheckOutputOnTx(void)
{
    UI_8 bufferDebug[100];
    UI_8 i;

    for (i = 0; i < N_SERVICES; i++) {
        if (services_tcuon_sts[i] > 0) {
            //PrintDebugService(index, buf);
            sprintf( (char *)bufferDebug, (const char *)"Service Req. Power ON = %d time = %d \r\n", i, services_tcuon_sts[i] );
            PrintDebugString( bufferDebug );
        }
        else {
            /* Do nothing */
        }
#if 0
        if(services_telem_drx_reset_request[i] & (0x01 << j)) {
            index = (i * 8) + j;
            //PrintDebugService(index, buf);
            sprintf(bufferDebug,"Service Req. DRX = %d \r\n", index);
            PrintDebugString(bufferDebug);
        }
        else {
            /* Do nothing */
        }
        if(services_hotstart_sts[i] & (0x01 << j)) {
            index = (i * 8) + j;
            //PrintDebugService(index, buf);
            sprintf(bufferDebug,"Service Req. hotStart = %d \r\n", index);
            PrintDebugString(bufferDebug);
        }
        else {
            /* Do nothing */
        }
#endif
    }

}
#endif

/*****************************************************************************
| Portability: General
|----------------------------------------------------------------------------
| Routine description:
|  Update service request
|---------------------------------------------------------------------------
| Parameters description:
|    request_type:
|           DRX_REQUEST
|           ONTX_REQUEST
|           GPS_HOTSTART_REQUEST
|    service_id: a service id, it must be defined at infrastructureCFG.h
|    aux: pointer to struct with data needed for the request.
|           structs suported: 
|                   t_drx_parameters
|                   t_ontx_parameters
|                   t_gps_hotstart_parameters
|
|   return:
|    TRUE Update accepted
|    FALSE Update not accepted
/---------------------------------------------------------------------------*/
/*
BOOL PMUpdateServiceRequest(UI_8 request_type, UI_8 service_id, void* aux)
{
    BOOL accepted = TRUE;
    
    switch (request_type){
        case DRX_REQUEST:
            accepted = call_drx_request(service_id, (t_drx_parameters *)aux);
            break;
        case ONTX_REQUEST:
            accepted = call_ontx_request(service_id, (t_ontx_parameters *)aux);
            break;
        case GPS_HOTSTART_REQUEST:
            accepted = call_gps_hotstart_request(service_id, (t_gps_hotstart_parameters *)aux);
            break;
        default:
            accepted = FALSE;
            break;
    }
    return accpeted;
}
*/

typedef enum {
    CAUSE_ALARM0,
    CAUSE_ALARM1,
    CAUSE_ALARM2,
    CAUSE_TELEMATIC,
    CAUSE_INT_MOVE,
    CAUSE_INT_MIX,
    CAUSE_POR
} t_wake_up_cause;

BOOL PMCheckWakeUpCauseOccurred(t_wake_up_cause wake_up_cause)
{
    BOOL occurred = FALSE;
    t_sig_rtc_tx_wake_up_cause         wake_up = 0;
    
    InternComOsekReceiveMessage(SIG_RTC_TX_WAKE_UP_CAUSE,(t_application_data_ref) &wake_up);
        switch (wake_up_cause){
            case CAUSE_ALARM0:
                occurred = ((wake_up & SIG_RTC_TX_WAKE_UP_CAUSE_ALARM0) == SIG_RTC_TX_WAKE_UP_CAUSE_ALARM0);
                break;
            case CAUSE_ALARM1:
                occurred = ((wake_up & SIG_RTC_TX_WAKE_UP_CAUSE_ALARM1) == SIG_RTC_TX_WAKE_UP_CAUSE_ALARM1);
                break;
            case CAUSE_ALARM2:
                occurred = ((wake_up & SIG_RTC_TX_WAKE_UP_CAUSE_ALARM2) == SIG_RTC_TX_WAKE_UP_CAUSE_ALARM2);
                break;
            case CAUSE_TELEMATIC:
                occurred = ((wake_up & SIG_RTC_TX_WAKE_UP_CAUSE_TELEM) == SIG_RTC_TX_WAKE_UP_CAUSE_TELEM);
                break;
            case CAUSE_INT_MOVE:
                occurred = ((wake_up & SIG_RTC_TX_WAKE_UP_CAUSE_INT_MOV) == SIG_RTC_TX_WAKE_UP_CAUSE_INT_MOV);
                break;
            case CAUSE_INT_MIX:
                occurred = ((wake_up & SIG_RTC_TX_WAKE_UP_CAUSE_INT_MIX) == SIG_RTC_TX_WAKE_UP_CAUSE_INT_MIX);
                break;
            case CAUSE_POR:
                occurred = ((wake_up & SIG_RTC_TX_WAKE_UP_CAUSE_POR) == SIG_RTC_TX_WAKE_UP_CAUSE_POR);
                break;
            default:
                occurred = FALSE;
                break;

        }
    
    return occurred;
}

UI_8 IntMixRtcStatus (void)
{
    t_sig_rtc_tx_int_mix_sts rtc_int_mix = SIG_RTC_TX_INT_MIX_STS_INTERRUPT_NOT_GENERATE;
    /* ignition variable */
    UI_8 int_mix_phy = INT_MIX_PHY_OFF;
    static BOOL received_first_comm = FALSE;

    if (received_first_comm == FALSE){
        /* Check if signal received from RTC */
        if(InternComOsekReadFlagRxSig(SIG_RTC_TX_INT_MIX_STS) == COM_TRUE){
            /* Read signal from RTC InternCom */
            InternComOsekReceiveMessage(SIG_RTC_TX_INT_MIX_STS,(t_application_data_ref) &rtc_int_mix);
            /* Indicate that the first frame has been received */
            received_first_comm = TRUE;
        }
    }
    else{
        /* Read signal from RTC InternCom */
        InternComOsekReceiveMessage(SIG_RTC_TX_INT_MIX_STS,(t_application_data_ref) &rtc_int_mix);
    }

    /* Update variable */
    if (rtc_int_mix == SIG_RTC_TX_INT_MIX_STS_INTERRUPT_GENERATE){
        int_mix_phy = INT_MIX_PHY_ON;
    }
    else if (rtc_int_mix == SIG_RTC_TX_INT_MIX_STS_INTERRUPT_NOT_GENERATE){
        int_mix_phy = INT_MIX_PHY_OFF;
    }
    else{
    	int_mix_phy = INT_MIX_PHY_UNKNOWN;
    }

    return int_mix_phy;
}

BOOL PowerMngGetForceTelemOffFlag(void){

    return powermng_force_telem_off_flag;

}

void PowerMngSetForceTelemOffFlag(BOOL force_off){

    powermng_force_telem_off_flag = force_off;

}

void PowerMngSetDelayedShutdownValue( UI_16 new_value )
{
    power_delayed_shutdown_value = new_value;
}

UI_16 PowerMngGetDelayedShutdownValue( void )
{
    return power_delayed_shutdown_value;
}


