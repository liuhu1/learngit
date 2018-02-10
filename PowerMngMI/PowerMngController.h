
/*************************   FICOSA INTERNATIONAL ****************************
| File:              | PowerMngController.h
|-----------------------------------------------------------------------------
| Author:            | ACF, APV, DRP
|-----------------------------------------------------------------------------
| Project:           | 08X_TCU2X_F4_RN_2013
| System:            | PMC_APP
| Diagram:           | PowerMngController
| Model Version:     | 1.0
| Model Date:        | 2014_03_26
|-----------------------------------------------------------------------------
| CodeH Version:     | 1.01
| FSM Generator Version:  | Generator_Ver_10/11/2016
| EDI Attributes Version: | Attributes_Ver_15_12_2014
|-----------------------------------------------------------------------------
| Description:
|    Automatic codification of states diagram
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
|
|    ++++ Implemented process description ++++			
|
|    TCU 2013 SW design of the different FSM of the PMC
|
********************************   END OF HEADER  ****************************/
#ifndef _POWERMNGCONTROLLER_H
#define _POWERMNGCONTROLLER_H

/*------------------------------- Includes ---------------------------------*/

#include "Global.h"
    

/*------------------------------ Data types --------------------------------*/
typedef enum {
    STATE_POWERMNGCONTROLLER_ONTX = 1, /* 11 */
    STATE_POWERMNGCONTROLLER_TCUON = 2, /* 12 */
    STATE_POWERMNGCONTROLLER_PMCINIT = 3, /* 17 */
    STATE_POWERMNGCONTROLLER_OFFSTARTED = 4, /* 21 */
    STATE_POWERMNGCONTROLLER_OFFCONFIRMEDSTOREDTCS = 5, /* 26 */
    STATE_POWERMNGCONTROLLER_PMCOFF = 6, /* 31 */
    STATE_POWERMNGCONTROLLER_PMCMONITORING = 7, /* 44 */
    STATE_0_POWERMNGCONTROLLER = 0
} t_state_powermngcontroller;

#ifdef DEBUG_UART 
typedef enum {
    TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONSFROMONTX_11_12 = 1,
    TRANS_POWERMNGCONTROLLER_STARTTCUOFF_12_21 = 2,
    TRANS_POWERMNGCONTROLLER_TIMEOUTSERVICEON_12_11 = 3,
    TRANS_POWERMNGCONTROLLER__12_12 = 4,
    TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONS_17_12 = 5,
    TRANS_POWERMNGCONTROLLER_NOWAKEUPCONDITIONSRTCPMRECEIVED_17_26 = 6,
    TRANS_POWERMNGCONTROLLER_ONTXCONDITIONS_17_11 = 7,
    TRANS_POWERMNGCONTROLLER_SLEEPMNGANDTELEMATICOFFCONFIRMED_21_26 = 8,
    TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONS_21_12 = 9,
    TRANS_POWERMNGCONTROLLER_SERVICESFINISHEDONTX_11_21 = 10,
    TRANS_POWERMNGCONTROLLER_INCREMENTSECONDS_11_11 = 11,
    TRANS_POWERMNGCONTROLLER_NOWAKEUPCONDITIONSRTCPMNOTRECEIVED_17_26 = 12,
    TRANS_POWERMNGCONTROLLER_WAKEUPCAUSESCLEAREDNOSERVICEONORWAKEUPREQUESTED_17_26 = 13,
    TRANS_POWERMNGCONTROLLER_DTCSANDACOOFFERROROFFFORCEDBYTIMEOUT_26_31 = 14,
    TRANS_POWERMNGCONTROLLER_DTCSSTOREDANDACCELEROMETERSLEEP_26_31 = 15,
    TRANS_POWERMNGCONTROLLER_ONTXCONDITIONS_21_11 = 16,
    TRANS_POWERMNGCONTROLLER_ONTXCONDITIONS_26_11 = 17,
    TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONS_26_12 = 18,
    TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_17_44 = 19,
    TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_11_44 = 20,
    TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_12_44 = 21,
    TRANS_POWERMNGCONTROLLER_STARTTCUOFF_44_21 = 22,
    TRANS_POWERMNGCONTROLLER_ONTXCONDITIONS_44_11 = 23,
    TRANS_POWERMNGCONTROLLER_WAKEUPCONDITIONS_44_12 = 24,
    TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_26_44 = 25,
    TRANS_POWERMNGCONTROLLER_MONITORINGONCONDITIONS_21_44 = 26,
} t_trans_powermngcontroller;
#endif /* DEBUG_UART */ 


/*------------------------ Initialization Routines -------------------------*/
 
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
void PowerMngControllerInitialize(void);


/*------------------------------ Task Routines -----------------------------*/
 
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
void PowerMngController(void);


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
t_state_powermngcontroller PowerMngControllerGetState(void);

/********************************** END **************************************/
#endif

