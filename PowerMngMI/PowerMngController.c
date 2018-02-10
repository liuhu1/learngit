
/*************************   FICOSA INTERNATIONAL ****************************
| File:                   | PowerMngController.c
|-----------------------------------------------------------------------------
| Author:                 | ACF, APV, DRP
|-----------------------------------------------------------------------------
| Project:                | 08X_TCU2X_F4_RN_2013
| System:                 | PMC_APP
| Diagram:                | PowerMngController
| Model Version:          | 1.0
| Model Date:             | 2014_03_26
|-----------------------------------------------------------------------------
| FSM Generator Version:  | Generator_Ver_10/11/2016
| EDI Attributes Version: | Attributes_Ver_15_12_2014
|-----------------------------------------------------------------------------
| Description:
|    Automatic codification of a states diagram
|
|    ++++ States Diagram Information +++++
|
|    Number of states:       7
|    Number of transitions: 26
|    Initial states:       1
|  
|    Input variables:
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus()	
|    Output variables:
|       TMSetNewStatus(UI_8 power_mode)	
|    Input/Output variables:	
|    Local variables:
|           telem_off_pm_request = 0 (UI_8)
|           technical_wakeup_present = 0 (BOOL)
|           initial_rtc_pm = 0 (t_sig_rtc_tx_cur_power_mode)
|           vehicle_awaken = 0 (BOOL)
|
|    ++++ Implemented process description ++++
|
|    Module that manages and commands the different system 
power modes that the PMC can decide (TCU_OFF, TCU_ON).
|
****************************   END OF HEADER  ******************************/

/*------------------------------- Includes --------------------------------*/

#include "Global.h"
#ifdef DEBUG_TRACES
#include "Debug.h"
#endif
#ifdef DEBUG_UART
#include "DebugMng.h"
#endif  /* DEBUG_UART */
#include "Timer.h"
#include "PowerMngController.h"
#include "ProjectCfg.h"
#include "PowerMngFunctions.h"
#include "PowerMng.h"
#include "InternComOsek.h"
#include "Eeprom.h"
#include "TcuSensingFunctions.h"
#include "TcuGeneralFunctions.h"
#include "TcuInOutFunctions.h"
#include "TelemMng.h"
#include "CANMapsMI.h"
#include "NvM.h"

#ifdef DEBUG_COMP
#include "Debug.h"
#endif

/*-------------------------------- Defines ---------------------------------*/

#define DTCS_EXT_CONTROLLER_TIMEOUT ((UI_16)2000)
#define MAX_SECONDS_OFF_TIMEOUT ((UI_16)300)
#define ONE_SECOND  ((UI_16)1000)
#define MIN_INIT_TIME ((UI_16)5000)
#define DELAY_END_CAN ((UI_16)2000)

/* Max value of timer */
#define MAX_COMPTADOR_TEMPS ((UI_16)65000)

/* Value for clearing all the RTC wakeup causes */
#define CLEAR_ALL_RTC_WAKE_UP_CAUSES (SIG_PMC_TX_CLEAR_WAKE_UP_CAUSE_POR | \
                                      SIG_PMC_TX_CLEAR_WAKE_UP_CAUSE_INT_MIX | \
                                      SIG_PMC_TX_CLEAR_WAKE_UP_CAUSE_INT_MOV | \
                                      SIG_PMC_TX_CLEAR_WAKE_UP_CAUSE_TELEM | \
                                      SIG_PMC_TX_CLEAR_WAKE_UP_CAUSE_ALARM0 | \
                                      SIG_PMC_TX_CLEAR_WAKE_UP_CAUSE_ALARM1 | \
                                      SIG_PMC_TX_CLEAR_WAKE_UP_CAUSE_ALARM2)



/*------------------------------ Variables --------------------------------*/
/* -- state variables -- */
static t_state_powermngcontroller state_powermngcontroller = STATE_0_POWERMNGCONTROLLER;

/* -- implicit timers -- */
static t_timer_time time_powermngcontroller = (t_timer_time)0;
static t_timer_tick last_tick_powermngcontroller = (t_timer_tick)0;

/* -- diagram specific variables -- */
static UI_8 telem_off_pm_request = 0;
static BOOL technical_wakeup_present = 0;
static t_sig_rtc_tx_cur_power_mode initial_rtc_pm = 0;
static BOOL vehicle_awaken = 0;


/*------------------------------ User Code --------------------------------*/


static void NotifyVehicleAwaken(void)
{
    t_sig_pmc_tx_reinit_drx_timer reinit_drx_timer = SIG_PMC_TX_REINIT_DRX_TIMER_REINIT;
    if(technical_wakeup_present == FALSE){
        /* No technical Wakeup, restart drx timer */
        InternComOsekSendMessage(SIG_PMC_TX_REINIT_DRX_TIMER,(t_application_data_ref) &reinit_drx_timer); 
    }else{
        /* DO nothing */
    }
    /* Set flag to notify that vehicle has been awaken*/
    vehicle_awaken = TRUE;
}
static void CheckTechnicalWakeup(void)
{
#if defined(RENAULT_GEN2_VARIANT) || defined(RENAULT_LC_VARIANT)
    BOOL tech_rx_ok = FALSE;
    t_sig_technicalwakeuptype wakeup_type = SIG_TECHNICALWAKEUPTYPE_UNAVAILABLE;
    /* Check TechnicalWakeup reception status */
    tech_rx_ok  = ReadFlagRxSig(SIG_TECHNICALWAKEUPTYPE);
    /* Read CAN value */
    if(tech_rx_ok == TRUE){
        ReceiveMessage(SIG_TECHNICALWAKEUPTYPE,(t_application_data_ref)&wakeup_type);
    }else{
        /* Do nothing */
    }
    /* Renit DRX timer */
    if( wakeup_type == SIG_TECHNICALWAKEUPTYPE_PERIODICAL_TECHNICAL_WAKE_UP){
        technical_wakeup_present = TRUE;
    }
    else{
        /* Do nothing */
    }
#endif
}
static BOOL GetRtcInitialPowerMode(void) 
{
    if(InternComOsekReadFlagRxSig(SIG_RTC_TX_CUR_POWER_MODE) == COM_TRUE) {
        /* Get the RTC current power mode */
        InternComOsekReceiveMessage(SIG_RTC_TX_CUR_POWER_MODE,(t_application_data_ref) &initial_rtc_pm);        
    }

    return ((initial_rtc_pm != SIG_RTC_TX_CUR_POWER_MODE_TCU_OFF) ? TRUE : FALSE);
}
static BOOL GetRtcInitialPowerModeError(void)
{
    return InternComOsekReadFlagRxErrorSig(SIG_RTC_TX_CUR_POWER_MODE);
}
static void SetPmcOffPowerMode(void)
{
    rtc_off_pm_request = (t_pm_req_status)PM_STATUS_OFF;
    telem_off_pm_request = TELEM_TCU_MODE_OFF;
}

/*---------------------------- Routine headers ----------------------------*/

static void PowerMngController0(void);
static void OnTx(void);
static void TcuOn(void);
static void PmcInit(void);
static void OffStarted(void);
static void OffConfirmedStoreDtcs(void);
static void PmcOff(void);
static void PmcMonitoring(void);


/*------------------------ Initialization Routines ------------------------*/
 
/**************************************************************************** 
| Functionality:                                                               
|   States diagram initialization PowerMngController 
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
void PowerMngControllerInitialize(void)
{
    /* -- state variable initialization -- */
    state_powermngcontroller = STATE_0_POWERMNGCONTROLLER;

    /* -- implicit timer time initialization -- */
    time_powermngcontroller= (t_timer_time)0;

    /* -- implicit timer tick initialization -- */
    last_tick_powermngcontroller= TimerGetCurrentTick();

    /* -- sates machine initial cycle execution -- */
    PowerMngController();
}


/*------------------------- Task Routines ---------------------------*/
 
/**************************************************************************** 
| Functionality:                                                               
|   Main routine of the states diagram PowerMngController.
|   Calls the corresponding function state. 
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
void PowerMngController(void)
{
    /* -- auxiliary timer variable -- */
    t_timer_time time_diff;
    /* -- get the time difference compared to last FSM execution -- */
    time_diff = TimerDeltaTime(&last_tick_powermngcontroller);
    /* -- check if implicit FSM timer will overflow -- */
    if ((T_TIMER_TIME_MAX_VALUE - time_diff) < time_powermngcontroller) {
        /* -- timer reached maximum allowed value -- */
        time_powermngcontroller = T_TIMER_TIME_MAX_VALUE;
    }
    else {
        /* -- increment of the FSM implicit timer -- */
        time_powermngcontroller += time_diff;
    }
    /* -- check the FSM current state -- */
    switch (state_powermngcontroller){
    case STATE_0_POWERMNGCONTROLLER:
        PowerMngController0();
        break;
    case STATE_POWERMNGCONTROLLER_ONTX:
        OnTx();
        break;
    case STATE_POWERMNGCONTROLLER_TCUON:
        TcuOn();
        break;
    case STATE_POWERMNGCONTROLLER_PMCINIT:
        PmcInit();
        break;
    case STATE_POWERMNGCONTROLLER_OFFSTARTED:
        OffStarted();
        break;
    case STATE_POWERMNGCONTROLLER_OFFCONFIRMEDSTOREDTCS:
        OffConfirmedStoreDtcs();
        break;
    case STATE_POWERMNGCONTROLLER_PMCOFF:
        PmcOff();
        break;
    case STATE_POWERMNGCONTROLLER_PMCMONITORING:
        PmcMonitoring();
        break;
    /* polyspace<RTE:UNR:Not a defect:Justify with annotations> Default case defined for defensive programming */
    default:
        state_powermngcontroller = STATE_0_POWERMNGCONTROLLER;
        break;
    }

}

/*---------------------------- Get State Routine ---------------------------*/
         
/**************************************************************************** 
| Functionality:                                                               
|   Function that returns the current state of the FSM PowerMngController. 
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
t_state_powermngcontroller PowerMngControllerGetState(void)
{
    return state_powermngcontroller;
}


/*--------------------------- State Routines -----------------------------*/

 
/**************************************************************************** 
| Functionality:                                                                  
|   Function corresponding to the initial state of the PowerMngController.   
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
static void PowerMngController0(void)
{
    /* -- action of the transition initial -- */
    ExtControllersWakeUp();
    InitServiceTcuOnRequest();
    InitServiceMonitoringRequest();
    //InitServiceDrxRequest();
    //InitGpsHotstartRequest();
    //InitServiceDisableTelemDrxResetRequest();
    //InitCanWakeupRequests();
    
    #ifdef DEBUG_COMP
    PrintDebugString((UI_8 *)"PowerMngController:PowerMngController0 from initial transition to PMCINIT\r\n");
    #endif

    /* -- initialization of the implicit timer time -- */
    time_powermngcontroller= (t_timer_time)0;
    /* -- we change the initial state -- */
    state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCINIT;
}
 
/**************************************************************************** 
| Functionality:                                                               
|   Function corresponding to the state OnTx.   
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
static void OnTx(void)
{
    /* -- code of the current state -- */
    BOOL tcu_on_conditions, service_on_req,monitoring_on_req;
    /* Set current power mode */
    pmc_power_mode = PMC_PM_ON_TX;
    /* Set RTC power mode to TCU_ON */
    PMSetRtcPowerMode( (t_pm_micro_requester)MICRO_PMC,
                                         (t_pm_req_status)PM_STATUS_ON);
    /* Notify to telematic to enter in TCU_ON */
    TMSetNewStatus(TELEM_TCU_MODE_ON_TX);
    /* Check TCU ON conditions */
    tcu_on_conditions = CheckTcuOnUpdateSleepMngConditions();
    /* Check if any service requested ON */
    service_on_req = CheckServiceTcuOnRequest();
    /* Check if any service requested monitoring ON */
    monitoring_on_req = CheckServiceMonitoringRequest();
    /* Update controllers power mode */
    ExtControllersWakeUp();
    /* GPS Controllers Wake Up */
    GPSControllerWakeUp();
    /* WARNING READ FUNCTION HEAD */
    PMSetFlagExitDrxSL8(TRUE);

    if (tcu_on_conditions == TRUE) {
        /* -- case where transition is executed WakeUpConditionsFromOnTx -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OnTx from WakeUpConditionsFromOnTx transition to TCUON\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_TCUON;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONSFROMONTX_11_12);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == FALSE) && (tcu_on_conditions == FALSE) && (monitoring_on_req == FALSE)) {
        /* -- case where transition is executed ServicesFinishedOnTx -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OnTx from ServicesFinishedOnTx transition to OFFSTARTED\r\n");
        #endif

        /* -- entry of the state OffStarted -- */        
        /* Set the RTC power mode requested in PMC OFF */
        SetPmcOffPowerMode();
        /* Set the max DRX time to RTC */
        //SetMaxDrxTime();
        
        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_OFFSTARTED;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_SERVICESFINISHEDONTX_11_21);
#endif /* DEBUG_UART */ 
    }

    /* polyspace<MISRA-C:19.7:Not a defect:Justify with annotations> Function like macro is used for efficiency */    else if ((time_powermngcontroller >= TIMER_MS_TO_TIME(ONE_SECOND))) {
        /* -- case where transition is executed IncrementSeconds -- */

        /* -- action of the transition -- */
        DecServiceTcuOnTout();
        
        #ifdef DEBUG_COMP
        CheckOutputOnTx();
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_ONTX;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_INCREMENTSECONDS_11_11);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == FALSE) && (tcu_on_conditions == FALSE) && (monitoring_on_req == TRUE)) {
        /* -- case where transition is executed MonitoringOnConditions -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OnTx from MonitoringConditionsFromOnTx transition to PmcMonitoring\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCMONITORING;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_11_44);
#endif /* DEBUG_UART */ 
    }
    else {
        /* -- no transition has been changed: keep current state -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_ONTX;
    }
}
 
/**************************************************************************** 
| Functionality:                                                               
|   Function corresponding to the state TcuOn.   
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
static void TcuOn(void)
{
    /* -- code of the current state -- */
    BOOL tcu_on_conditions, service_on_req, monitoring_on_req;
    /* Set current power mode */
    pmc_power_mode = PMC_PM_ON;
    
    if (PowerMngGetForceTelemOffFlag() == FALSE){
    
        /* Set RTC power mode to TCU_ON */
        PMSetRtcPowerMode( (t_pm_micro_requester)MICRO_PMC,
                                         (t_pm_req_status)PM_STATUS_ON);
        /* Notify to telematic to enter in TCU_ON */
        TMSetNewStatus(TELEM_TCU_MODE_ON);
    
    } else {
    
        /* Set RTC power mode to TCU_ON */
        PMSetRtcPowerMode( (t_pm_micro_requester)MICRO_TELEM,
                                        (t_pm_req_status)PM_STATUS_OFF);
        /* Notify to telematic to enter in TCU_OFF */
        TMSetNewStatus(TELEM_TCU_MODE_OFF);
    
    }
    
    /* Check TCU ON conditions */
    tcu_on_conditions = CheckTcuOnUpdateSleepMngConditions();
    /* Check if it is a technical wakeup */
     CheckTechnicalWakeup();
    /* Check if any service requested ON */
    service_on_req = CheckServiceTcuOnRequest();
    /* Check if any service requested monitoring ON */
    monitoring_on_req = CheckServiceMonitoringRequest();
    /* Update controllers power mode */
    ExtControllersWakeUp();
    /* GPS Controllers Wake Up */
    GPSControllerWakeUp();
    /* WARNING READ FUNCTION HEAD */
    PMSetFlagExitDrxSL8(TRUE);

    if ((service_on_req == FALSE)&& (tcu_on_conditions == FALSE) && (monitoring_on_req == FALSE)) {
        /* -- case where transition is executed StartTcuOff -- */

        /* -- action of the transition -- */
        /* Notify that tcu has been awaken by vehicle wakeup */
        NotifyVehicleAwaken();
        
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:TcuOn from StartTcuOff transition to OFFSTARTED\r\n");
        #endif

        /* -- entry of the state OffStarted -- */        
        /* Set the RTC power mode requested in PMC OFF */
        SetPmcOffPowerMode();
        /* Set the max DRX time to RTC */
        //SetMaxDrxTime();
        
        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_OFFSTARTED;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_STARTTCUOFF_12_21);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == TRUE)&& (tcu_on_conditions == FALSE)) {
        /* -- case where transition is executed TimeoutServiceOn -- */

        /* -- action of the transition -- */
        /* Notify that tcu has been awaken by vehicle wakeup */
        NotifyVehicleAwaken();
        
        InitTcuOnMaxTime();
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:TcuOn from TimeoutServiceOn transition to ONTX\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_ONTX;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_TIMEOUTSERVICEON_12_11);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == TRUE) || (tcu_on_conditions == TRUE)) {
        /* -- case where transition is executed  -- */

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_TCUON;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER__12_12);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == FALSE) && (tcu_on_conditions == FALSE) && (monitoring_on_req == TRUE)) {
        /* -- case where transition is executed MonitoringOnConditions -- */

        /* -- action of the transition -- */
        /* Notify that tcu has been awaken by vehicle wakeup */
        NotifyVehicleAwaken();
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:TCUON from MonitoringConditionsFromTCUON transition to PmcMonitoring\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCMONITORING;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_12_44);
#endif /* DEBUG_UART */ 
    }
    else {
        /* -- no transition has been changed: keep current state -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_TCUON;
    }
}
 
/**************************************************************************** 
| Functionality:                                                               
|   Function corresponding to the state PmcInit.   
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
static void PmcInit(void)
{
    /* -- code of the current state -- */
    BOOL tcu_on_conditions, rtc_wakeup_conditions, service_on_req,monitoring_on_req;//, canv_canm_sleeping;
    BOOL rtc_initial_power_mode, rtc_pwr_mode_err;
    /* Set PMC current power mode status */
    pmc_power_mode = PMC_PM_INIT;
    /* Keep RTC power mode to current one */
    PMSetRtcPowerMode( (t_pm_micro_requester)MICRO_PMC,
                                         (t_pm_req_status)PM_STATUS_CURRENT);
    /* Check TCU ON conditions */
    tcu_on_conditions = CheckTcuOnUpdateSleepMngConditions();
    /* Check RTC wakeup causes conditions */
    rtc_wakeup_conditions = CheckRtcWakeupCauseConditions();
    /* Check if CAN-V and CAN-M are in sleeping state */
    //canv_canm_sleeping = CheckSleepMngSleeping();
    /* Check if any service requested ON */
    service_on_req = CheckServiceTcuOnRequest();
    /* Check if any service requested monitoring ON */
    monitoring_on_req = CheckServiceMonitoringRequest();
    rtc_initial_power_mode = GetRtcInitialPowerMode();
    rtc_pwr_mode_err = GetRtcInitialPowerModeError();
    /* Update controllers power mode */
    ExtControllersWakeUp();
    /* WARNING READ FUNCTION HEAD */
    PMSetFlagExitDrxSL8(FALSE);

    if ((rtc_initial_power_mode==TRUE) && (tcu_on_conditions == TRUE)) {
        /* -- case where transition is executed WakeupConditions -- */

        /* -- action of the transition -- */
        /* Start the DTCs monitoring */
        //dtc_mng = START_DTCS_MONITORING;
        
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:PmcInit from WakeupConditions transition to TCUON\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_TCUON;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONS_17_12);
#endif /* DEBUG_UART */ 
    }

    /* polyspace<MISRA-C:19.7:Not a defect:Justify with annotations> Function like macro is used for efficiency */    else if ((time_powermngcontroller >= TIMER_MS_TO_TIME(MIN_INIT_TIME))&& (rtc_initial_power_mode==TRUE) && (tcu_on_conditions == FALSE) && /* (canv_canm_sleeping == TRUE) && */ (service_on_req == FALSE) && (monitoring_on_req == FALSE)) {
        /* -- case where transition is executed NoWakeupConditionsRtcPmReceived -- */

        /* -- action of the transition -- */
        /* Set the RTC power mode for switching PMC OFF */
        SetPmcOffPowerMode();
        /* Clear all the RTC wakeup causes */
        ClearWakeUpCause(CLEAR_ALL_RTC_WAKE_UP_CAUSES,TRUE);
        
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:PmcInit from NoWakeupConditionsRtcPmReceived transition to OFFCONFIRMEDSTOREDTCS\r\n");
        #endif

        /* -- entry of the state OffConfirmedStoreDtcs -- */        
        /* Set external controllers to sleep state */
        ExtControllersSleep();
        /* Set GPS controller to sleep state */
        GPSControllerSleep();
        /* DEM Shutdown */
        Dem_Shutdown();
        
        /* NvM_WriteAll */
        (void) NvM_WriteAll();
        
        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_OFFCONFIRMEDSTOREDTCS;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_NOWAKEUPCONDITIONSRTCPMRECEIVED_17_26);
#endif /* DEBUG_UART */ 
    }
    else if ((rtc_initial_power_mode==TRUE) && (service_on_req == TRUE) && (tcu_on_conditions == FALSE)) {
        /* -- case where transition is executed OnTxConditions -- */

        /* -- action of the transition -- */
        /* Start the DTCs monitoring */
        //dtc_mng = START_DTCS_MONITORING;
        #ifdef DEBUG_COMP
        CheckOutputOnTx();
        #endif
        
        InitTcuOnMaxTime();
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:PmcInit from OnTxConditions transition to ONTX\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_ONTX;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_ONTXCONDITIONS_17_11);
#endif /* DEBUG_UART */ 
    }

    /* polyspace<MISRA-C:19.7:Not a defect:Justify with annotations> Function like macro is used for efficiency */    else if ((time_powermngcontroller >= TIMER_MS_TO_TIME(MIN_INIT_TIME))&& (rtc_pwr_mode_err == TRUE) && (tcu_on_conditions == FALSE) && /* (canv_canm_sleeping == TRUE) && */ (service_on_req == FALSE) && (monitoring_on_req == FALSE)) {
        /* -- case where transition is executed NoWakeupConditionsRtcPmNotReceived -- */

        /* -- action of the transition -- */
        /* Request TCU_OFF as there is not RX COM with RTC */
        rtc_off_pm_request = (t_pm_req_status)PM_STATUS_OFF;
        telem_off_pm_request = TELEM_TCU_MODE_OFF;
        /* Clear all the RTC wakeup causes */
        ClearWakeUpCause(CLEAR_ALL_RTC_WAKE_UP_CAUSES,TRUE);
        
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:PmcInit from NoWakeupConditionsRtcPmNotReceived transition to OFFCONFIRMEDSTOREDTCS\r\n");
        #endif

        /* -- entry of the state OffConfirmedStoreDtcs -- */        
        /* Set external controllers to sleep state */
        ExtControllersSleep();
        /* Set GPS controller to sleep state */
        GPSControllerSleep();
        /* DEM Shutdown */
        Dem_Shutdown();
        
        /* NvM_WriteAll */
        (void) NvM_WriteAll();
        
        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_OFFCONFIRMEDSTOREDTCS;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_NOWAKEUPCONDITIONSRTCPMNOTRECEIVED_17_26);
#endif /* DEBUG_UART */ 
    }
    else if ((rtc_wakeup_conditions == FALSE) && (rtc_initial_power_mode == TRUE) && (service_on_req == FALSE) && (tcu_on_conditions == FALSE) /* && (canv_canm_sleeping == TRUE) */ && (monitoring_on_req == FALSE)) {
        /* -- case where transition is executed WakeupCausesClearedNoServiceOnOrWakeupRequested -- */

        /* -- action of the transition -- */
        /* Set the RTC power mode for switching PMC OFF */
        SetPmcOffPowerMode();
        
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:PmcInit from WakeupCausesClearedNoServiceOnOrWakeupRequested transition to OFFCONFIRMEDSTOREDTCS\r\n");
        #endif

        /* -- entry of the state OffConfirmedStoreDtcs -- */        
        /* Set external controllers to sleep state */
        ExtControllersSleep();
        /* Set GPS controller to sleep state */
        GPSControllerSleep();
        /* DEM Shutdown */
        Dem_Shutdown();
        
        /* NvM_WriteAll */
        (void) NvM_WriteAll();
        
        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_OFFCONFIRMEDSTOREDTCS;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_WAKEUPCAUSESCLEAREDNOSERVICEONORWAKEUPREQUESTED_17_26);
#endif /* DEBUG_UART */ 
    }
    else if ((rtc_initial_power_mode==TRUE) && (service_on_req == FALSE) && (tcu_on_conditions == FALSE) && (monitoring_on_req == TRUE)) {
        /* -- case where transition is executed MonitoringOnConditions -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:PmcInit from MonitoringConditionsFromPmcInit transition to PmcMonitoring\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCMONITORING;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_17_44);
#endif /* DEBUG_UART */ 
    }
    else {
        /* -- no transition has been changed: keep current state -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCINIT;
    }
}
 
/**************************************************************************** 
| Functionality:                                                               
|   Function corresponding to the state OffStarted.   
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
static void OffStarted(void)
{
    /* -- code of the current state -- */    
    BOOL tcu_on_conditions, /* canv_canm_sleeping, */ service_on_req,monitoring_on_req;
    t_interface_status telem_status;
    /* Set PMC current power mode */
    pmc_power_mode = PMC_PM_OFF_STARTED;
    PMSetRtcPowerMode( (t_pm_micro_requester)MICRO_PMC,
                                         (t_pm_req_status)PM_STATUS_CURRENT);
    /* Notify to telematic to enter in TCU_OFF */
    TMSetNewStatus(telem_off_pm_request);
    /* Check TCU ON conditions */
    tcu_on_conditions = CheckTcuOnUpdateSleepMngConditions();
    /* Check if CAN-V and CAN-M are in sleeping state */
    //canv_canm_sleeping = CheckSleepMngSleeping();
    /* Check if any service requested ON */
    service_on_req = CheckServiceTcuOnRequest();
    /* Check if any service requested monitoring ON */
    monitoring_on_req = CheckServiceMonitoringRequest();
    /* Check telematic status status */
    telem_status = TMReadStatus();

    if (/* (canv_canm_sleeping == TRUE) && */ (tcu_on_conditions == FALSE) && (telem_status == INTERFACE_OFF) && (service_on_req == FALSE) && (monitoring_on_req == FALSE)) {
        /* -- case where transition is executed SleepMngAndTelematicOffConfirmed -- */

        /* -- action of the transition -- */
        /* Request to store eeprom data in flash */
        //dtc_mng = STORE_DTCS_IN_FLASH;
        
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OffStarted from SleepMngAndTelematicOffConfirmed transition to OFFCONFIRMEDSTOREDTCS\r\n");
        #endif

        /* -- entry of the state OffConfirmedStoreDtcs -- */        
        /* Set external controllers to sleep state */
        ExtControllersSleep();
        /* Set GPS controller to sleep state */
        GPSControllerSleep();
        /* DEM Shutdown */
        Dem_Shutdown();
        
        /* NvM_WriteAll */
        (void) NvM_WriteAll();
        
        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_OFFCONFIRMEDSTOREDTCS;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_SLEEPMNGANDTELEMATICOFFCONFIRMED_21_26);
#endif /* DEBUG_UART */ 
    }
    else if ((tcu_on_conditions == TRUE)) {
        /* -- case where transition is executed WakeupConditions -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OffStarted from WakeupConditions transition to TCUON\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_TCUON;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONS_21_12);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == TRUE) && (tcu_on_conditions == FALSE)) {
        /* -- case where transition is executed OnTxConditions -- */

        /* -- action of the transition -- */
        InitTcuOnMaxTime();
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OffStarted from OnTxConditions transition to ONTX\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_ONTX;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_ONTXCONDITIONS_21_11);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == FALSE) && (tcu_on_conditions == FALSE) && (monitoring_on_req == TRUE)) {
        /* -- case where transition is executed MonitoringOnConditions -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OffStarted from MonitoringConditionsFromOffStarted transition to PmcMonitoring\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCMONITORING;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_21_44);
#endif /* DEBUG_UART */ 
    }
    else {
        /* -- no transition has been changed: keep current state -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_OFFSTARTED;
    }
}
 
/**************************************************************************** 
| Functionality:                                                               
|   Function corresponding to the state OffConfirmedStoreDtcs.   
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
static void OffConfirmedStoreDtcs(void)
{
    /* -- code of the current state -- */    
    NvM_RequestResultType write_all_status = NVM_REQ_PENDING;
    
    BOOL ext_controllers_sleep, service_on_req, tcu_on_conditions,monitoring_on_req;
    /* Set current PMC power mode */
    pmc_power_mode = PMC_PM_OFF_STORE_DTCS;
    /* Keep RTC power mode to current one */
    PMSetRtcPowerMode( (t_pm_micro_requester)MICRO_PMC,
                                         (t_pm_req_status)PM_STATUS_CURRENT);
    /* Check if controllers are correctly slept*/
    ext_controllers_sleep = CheckExtControllersSleep();
    /* Check TCU ON conditions */
    tcu_on_conditions = CheckTcuOnUpdateSleepMngConditions();
    /* Check if any service requested ON */
    service_on_req = CheckServiceTcuOnRequest();
    /* Check if any service requested monitoring ON */
    monitoring_on_req = CheckServiceMonitoringRequest();
    /* Notify to telematic to enter in TCU_OFF */
    TMSetNewStatus(telem_off_pm_request);
    /* WARNING READ FUNCTION HEAD */
    PMSetFlagExitDrxSL8(FALSE);
    
    /* check status of WriteAll operation */
    (void) NvM_GetErrorStatus(0, &write_all_status);

    /* polyspace<MISRA-C:19.7:Not a defect:Justify with annotations> Function like macro is used for efficiency */
    if ((time_powermngcontroller >= TIMER_MS_TO_TIME(DTCS_EXT_CONTROLLER_TIMEOUT))&& (service_on_req == FALSE) && (tcu_on_conditions == FALSE) && (monitoring_on_req == FALSE)) {
        /* -- case where transition is executed DtcsAndAcoOffErrorOffForcedByTimeout -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OffConfirmedStoreDtcs from DtcsAndAcoOffErrorOffForcedByTimeout transition to PMCOFF\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCOFF;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_DTCSANDACOOFFERROROFFFORCEDBYTIMEOUT_26_31);
#endif /* DEBUG_UART */ 
    }
    else if ((ext_controllers_sleep == TRUE) && (write_all_status != NVM_REQ_PENDING) && /* (dtcs_stored_in_flash == TRUE) && (EepromProgStatus()!=EEPROM_BUSY) && (rtime_task_mode != RTIME_SYNCHING) && */ (service_on_req == FALSE) && (tcu_on_conditions == FALSE) && (monitoring_on_req == FALSE)) {
        /* -- case where transition is executed DtcsStoredAndAccelerometerSleep -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OffConfirmedStoreDtcs from DtcsStoredAndAccelerometerSleep transition to PMCOFF\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCOFF;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_DTCSSTOREDANDACCELEROMETERSLEEP_26_31);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == TRUE) && (tcu_on_conditions == FALSE)) {
        /* -- case where transition is executed OnTxConditions -- */

        /* -- action of the transition -- */
        /* Start the DTCs monitoring */
        //dtc_mng = START_DTCS_MONITORING;
        
        /* DEM Init */
        Dem_Init(NULL_PTR);
        
        InitTcuOnMaxTime();
        
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OffConfirmedStoreDtcs from OnTxConditions transition to ONTX\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_ONTX;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_ONTXCONDITIONS_26_11);
#endif /* DEBUG_UART */ 
    }
    else if ((tcu_on_conditions == TRUE)) {
        /* -- case where transition is executed WakeupConditions -- */

        /* -- action of the transition -- */
        /* Start the DTCs monitoring */
        //dtc_mng = START_DTCS_MONITORING;
        
        /* DEM Init */
        Dem_Init(NULL_PTR);
        
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OffConfirmedStoreDtcs from WakeupConditions transition to TCUON\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_TCUON;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONS_26_12);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == FALSE) && (tcu_on_conditions == FALSE) && (monitoring_on_req == TRUE)) {
        /* -- case where transition is executed MonitoringOnConditions -- */

        /* -- action of the transition -- */
        /* Start the DTCs monitoring */
        //dtc_mng = START_DTCS_MONITORING;
        
        /* DEM Init */
        Dem_Init(NULL_PTR);
        
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:OffConfirmedStoreDtcs from MonitoringConditionsFromOffConfirmedStoreDtcs transition to PmcMonitoring\r\n");
        #endif

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCMONITORING;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_26_44);
#endif /* DEBUG_UART */ 
    }
    else {
        /* -- no transition has been changed: keep current state -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_OFFCONFIRMEDSTOREDTCS;
    }
}
 
/**************************************************************************** 
| Functionality:                                                               
|   Function corresponding to the state PmcOff.   
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
static void PmcOff(void)
{
    /* -- code of the current state -- */
    /* Set current PMC power mode */
    pmc_power_mode = PMC_PM_OFF_CONFIRMED;
    /* Set requested RTC power mode for switching PMC OFF */
    PMSetRtcPowerMode( (t_pm_micro_requester)MICRO_PMC, rtc_off_pm_request);
    
    ControlNstbCanV(PORT_DEACTIVATED);
    ControlEnableCanV(PORT_ACTIVATED);

    /* -- aquest es un state pou. Ja no es canviara d state -- */
    /* -- per redundancia es reassigna el mateix state -- */
    state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCOFF;
}
 
/**************************************************************************** 
| Functionality:                                                               
|   Function corresponding to the state PmcMonitoring.   
|---------------------------------------------------------------------------- 
| Interface:                                                                 
|   Inputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       drx_diag_config
|       drx_diag_time
|       TMReadStatus() 
|   Outputs:
|       state_powermngcontroller
|       time_powermngcontroller
|       last_tick_powermngcontroller
|       TMSetNewStatus(UI_8 power_mode) 
****************************************************************************/
static void PmcMonitoring(void)
{
    /* -- code of the current state -- */
    BOOL service_on_req, tcu_on_conditions,monitoring_on_req;    
    /* Set current PMC power mode */
    pmc_power_mode = PMC_PM_MONITORING;
    /* Set requested RTC power mode for PMC Monitoring */
    PMSetRtcPowerMode( (t_pm_micro_requester)MICRO_PMC, (t_pm_req_status)PM_STATUS_CURRENT);/* Keep last conditions */
    tcu_on_conditions = CheckTcuOnUpdateSleepMngConditions();
    /* Check if any service requested ON */
    service_on_req = CheckServiceTcuOnRequest();
    /* Check if any service requested monitoring ON */
    monitoring_on_req = CheckServiceMonitoringRequest();
    /* Wake up the external periphereals */
    ExtControllersWakeUp();
    /* Notify to telematic to enter in TCU_OFF */
    TMSetNewStatus(TELEM_TCU_MODE_OFF);
    /* WARNING READ FUNCTION HEAD */
    PMSetFlagExitDrxSL8(TRUE);

    if ((service_on_req == FALSE)&& (tcu_on_conditions == FALSE) && (monitoring_on_req == FALSE)) {
        /* -- case where transition is executed StartTcuOff -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:PmcMonitoring from StartTcuOff transition to OFFSTARTED\r\n");
        #endif   

        /* -- entry of the state OffStarted -- */        
        /* Set the RTC power mode requested in PMC OFF */
        SetPmcOffPowerMode();
        /* Set the max DRX time to RTC */
        //SetMaxDrxTime();
        
        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_OFFSTARTED;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_STARTTCUOFF_44_21);
#endif /* DEBUG_UART */ 
    }
    else if ((service_on_req == TRUE) && (tcu_on_conditions == FALSE)) {
        /* -- case where transition is executed OnTxConditions -- */

        /* -- action of the transition -- */
        InitTcuOnMaxTime();
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:PmcMonitoring OnTxConditions transition to ONTX\r\n");
        #endif  

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_ONTX;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_ONTXCONDITIONS_44_11);
#endif /* DEBUG_UART */ 
    }
    else if ((tcu_on_conditions == TRUE)) {
        /* -- case where transition is executed WakeupConditions -- */

        /* -- action of the transition -- */
        #ifdef DEBUG_COMP
        PrintDebugString((UI_8 *)"PowerMngController:PmcMonitoring from WakeUpConditionsFromPmcMonitoring transition to TCUON\r\n");
        #endif  

        /* -- initialization of the implicit timer time -- */
        time_powermngcontroller= (t_timer_time)0;

        /* -- state is changed -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_TCUON;

        /* -- send debug information -- */
#ifdef DEBUG_UART 
        PrintTrans(DEBUG_POWERMNGCONTROLLER_FSM, (UI_16)TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONS_44_12);
#endif /* DEBUG_UART */ 
    }
    else {
        /* -- no transition has been changed: keep current state -- */
        state_powermngcontroller = STATE_POWERMNGCONTROLLER_PMCMONITORING;
    }
}


/********************************** END **************************************/
