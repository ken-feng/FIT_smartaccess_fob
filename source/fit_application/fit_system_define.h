//********************************************************************************
// Modification Tag
//********************************************************************************
//Modify :NXP-V0001 (Ken) -20240424

//********************************************************************************
// Version
//********************************************************************************
#define APP_VERSION_H	        0x00
#define APP_VERSION_L	        0x01

//********************************************************************************
// HW board
//********************************************************************************
//#define __HW_BLE_UWB_G2_H
//#define __HW_BLE_UWB_FOB_H
//#define __HW_BLE_UWB_EVT1_H
//#define __HW_VENUS_EVT1_H

//********************************************************************************
// Clock Source
//********************************************************************************
#define __SYS_CLOCK_EXTERNAL_H						// FLL Engaged External
//#define __SYS_CLOCK_INTERNAL_H						// FLL Engaged Internal

//********************************************************************************
// For Application Function
//********************************************************************************
//#define __FIT_UART_SETTING_H
//#define __FIT_UART_LOG_USING_TASK_H
//--------------------------------------------------------------------------------
#ifdef __HW_BLE_UWB_EVT1_H
  #define __FIT_BUTTON_SETTING_HIGH_TRIGGER_H
#endif
//#define __FIT_BLE_TIMEOUT_STOP_ADV_H
//#define __HW_TEST_PIN_H
//#define __FIT_UWB_RangingSpeed_TEST_H
//#define __FIT_UWB_NoBLERanging_TEST_H

//********************************************************************************
// For Test
//********************************************************************************
//#define __FIT_UART_TEST_H
//#define __NXP_EVB_TEST_PIN_H
//#define __FIT_UWB_RangingSpeed_TEST_H

//********************************************************************************
// For Log
//********************************************************************************
// Enable/disable RTT debug, log will be printed to Segger RTT viewer
//#define SEGGER_RTT_ENABLE                       1

//********************************************************************************
// For Application parameter
//********************************************************************************
#define __FIT_UWB_FEATURE_NCJ29D5_H
//#define __FIT_SECURITY_CHIP_H

//==============================================================================
// UWB - NCJ29D5 Configurations
//==============================================================================
#ifdef  __FIT_UWB_FEATURE_NCJ29D5_H
  //----------------------------------------------------------------------------
  // Role
  //----------------------------------------------------------------------------
  #define __UWB_ROLE_INITIATOR_H
//  #define __UWB_ROLE_RESPONDER_H
  //----------------------------------------------------------------------------
   /* Enable DCDC output 3V to provide 3V on GPIO input/output, UWB needs IO level as 3V */
  //----------------------------------------------------------------------------
  #define gDCDC_3p3V_Output_d                     1
  //----------------------------------------------------------------------------
  /* BLE already supported by default, if below feature not support, then this is only BLE auxiliary node */
  //----------------------------------------------------------------------------
  #define UWB_FEATURE_SUPPORT                     1
  //----------------------------------------------------------------------------
  #ifdef  __UWB_ROLE_RESPONDER_H
      #define NUM_ANCHORS                             1       //number of NCJ29D5 anchors, support 1-4 anchors in one session in this demo
      #define __InitSessionCfg_Size                   15
  #elif defined   __UWB_ROLE_INITIATOR_H
      #define __InitSessionCfg_Size                   13
  #else
      #error("The UWB configuration no role")
  #endif
  //----------------------------------------------------------------------------
  /* anchor slot index in UWB ranging session, if there are not only one anchor in a ranging session 
     then the ANCHOR_INDEX should not be same.
      0 is CAN center node
      other is CAN slave node
  */  
  //----------------------------------------------------------------------------
  #define ANCHOR_INDEX                            0u

  #if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
    #define NUM_OF_UWB_TASK                         1
    #define NUM_OF_UWB_EVENTS                       1
  #else
    #define NUM_OF_UWB_TASK                         0
    #define NUM_OF_UWB_EVENTS                       0
  #endif   
#endif
  //----------------------------------------------------------------------------
  // UWB Sync method
  //----------------------------------------------------------------------------
  #define	__FIT_UWB_RFIC_SYNC_H

//==============================================================================
// Security Chip
//==============================================================================


