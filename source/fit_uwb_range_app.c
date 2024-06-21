//***************************************************** 
// The KW38 serial head file define 
// Define the main program Ram section  
//***************************************************** 
#include "EmbeddedTypes.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "board.h"
#include "fsl_port.h"
#include "pin_mux.h"
#include "fit_hardware_initial.h"

#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
#include "Ranger4UciCmd.h"
#include "Ranger4_demo_task.h"
#include "fit_uwb_range_app.h"
#endif


//#include "NVM_Interface.h"
//#include "Reset.h"
//#include "Panic.h"
//#include "RNG_Interface.h"
//#include "TimersManager.h"
//#include "FunctionLib.h"
//#include "Messaging.h"
//#include "MemManager.h"
//***********************************************************************************
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/

#define UWB_INITIATION_TIME_DEMO                100u   // milliseconds
#define ANCHOR_UWB_INITIATION_TIME              500u

// Capability of UWB should achieve from NCJ29D5 via UCI command.
UwbCapabilityManagement_t UwbCapability;

// UWB Ranging Session
SessionManagement_t UwbSessions;

uint64_t TsUwbCurrentTimeStamp = 0;

#define mCharReadBufferLength_c     (13U)           /* length of the buffer */


#ifdef __HW_TEST_PIN_H
    #define __TEST_PTC_HIGH(x)			GPIO_PinWrite (GPIOC, x, 1)
    #define __TEST_PTC_LOW(x)			GPIO_PinWrite (GPIOC, x, 0)
    #define __TEST_TogglePTC(x)			GPIO_PortToggle (GPIOC, 1<<x)
#else
    #define __TEST_PTC_HIGH(x)
    #define __TEST_PTC_LOW(x)
    #define __TEST_TogglePTC(x)
#endif

#define gAppMaxConnections_c           				(1U)
#define gInvalidDeviceId_c                          (0xFFU)
#define gInvalidNvmIndex_c                          (0xFFU)
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef struct appCustomInfo_tag
{

    uint16_t    hUwbService; 			// Service UUID handle
    uint16_t    hUwb;				// characteristic ID/handle for Notification
    uint16_t    hUwbCccd;
    uint16_t    hUwbDesc;
    uint16_t    hNumAnchors;
    uint16_t    NumAnchorsValue;
}appCustomInfo_t;

typedef struct appUwbInfo_tag
{
    uint32_t            RangingSessionID;
    uint64_t            TsUwbLocalDeviceTime;
    uint64_t            TsLocalDeviceEventCount;
    uint32_t            TsBleEvtDelay;
    uint8_t             TsUwbDeviceTimeUncertainty;
    uint16_t            TsRetryDelay;
    bool_t              NeedsSendTimeSyncForPhyUpdate;
} appUwbInfo_t;

typedef struct appUwbTsPeerDevice_tag
{
    uint8_t head;
    uint8_t tail;
    uint8_t maxListSize;
    deviceId_t DeviceIdList[UWB_MAX_QUERY_TIMESTAMP_LIST_SIZE];
}appUwbTsPeerDevice_t;

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
static appUwbInfo_t mPeerInformation;

/* Timers */
//static tmrTimerID_t mAppTimerId;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
//******************************************************************************
// UWB (NCJ29D5)
//******************************************************************************
static appUwbTsPeerDevice_t     TsPeerDeviceList;
//static tmrTimerID_t             mTimeSyncTimerId;
//==============================================================================
/*** Ranger4 core configuration ***/
//==============================================================================
static appUwbTsPeerDevice_t TsPeerDeviceList;

static uint8_t UwbLowPowerMode[1] = {0x00};
static Ranger4Uci_ParameterStructure_t UwbCoreCfg[1] =
{
    {
        .ID = R4_PARAM_LOW_POWER_MODE,
        .length = 0x01,
        .paramValue = UwbLowPowerMode
    }
};

static uint8_t UwbCoreCfgSize = NumberOfElements(UwbCoreCfg);

static Ranger4Uci_DeviceStatusType_t mUwbDeviceStatus = R4_UCI_DEVICE_STATUS_UNDEFINE;

static bool mIsRanger4Init = false;
static bool mIsRangingStart = false;
static bool mIsRangingSessionInit = false;

//static tmrTimerID_t mUwbStartRangingTimerId;


uint8_t UWB_Api_Start_Ranging(void* pUsrkDst, uint16_t sessio_id)
{
	#if defined __FIT_UWB_RFIC_SYNC_H
	if(mIsRangingSessionInit == false)
	{
        UwbSessions.UwbSessionID = sessio_id;                  // because uwbSession=0x01 is test mode
        UwbSessions.Number_Responders_Nodes = 0x01;       // Initiator (needs get from anchor side)
        Ranger4UciCmd_MacSessionCfgInit(UwbSessions.UwbSessionID, UwbSessions.SessionType);
        mIsRangingSessionInit = true;
        return	0;
	}
	else{
		return	1;
	}
	#endif
}

uint8_t UWB_Api_Stop_Ranging(uint16_t sessio_id)
{
	if(mIsRangingSessionInit == true)
	{
		UwbSessions.UwbSessionID = sessio_id;                  // because uwbSession=0x01 is test mode
        Ranger4UciCmd_MacSessionCfgDeInit(UwbSessions.UwbSessionID);
        UwbSessions.SessionStatus = R4_SESSION_STATE_DEINIT;
        mIsRangingStart = false;
        mIsRangingSessionInit = false;
        return	0;
	}
	else{
		return	1;
	}
}

void UWB_Api_Reset(void)
{

}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : The byte at index i from the source buffer is copied to index ((n-1) - i)
 *         in the destination buffer (and vice versa).
 * Description   :
 *
 *
 * END ****************************************************************************************************************/
static void MemCpyReverseOrder (void* pDst, const void* pSrc, uint32_t cBytes)
{
    if(cBytes != 0UL)
    {
        pDst = (uint8_t*)pDst + (uint32_t)(cBytes-1UL);
        while (cBytes != 0UL)
        {
            *((uint8_t*)pDst) = *((const uint8_t*)pSrc);
            pDst = (uint8_t*)pDst-1U;
            pSrc = (const uint8_t*)pSrc+1U;
            cBytes--;
        }
    }
}



/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_Init
 * Description   : Calls debug console initialization functions
 * 
 *
 * END ****************************************************************************************************************/
void Init_UWB_Range(void)
{
	//--------------------------------------------------------------------------
	// [ Allcate Timer ] - for UWB
	//--------------------------------------------------------------------------
	//mUwbStartRangingTimerId = TMR_AllocateTimer();

	//--------------------------------------------------------------------------
	// [ Initial data ] - for UWB
	//--------------------------------------------------------------------------
	TsPeerDeviceList.head = 0;
	TsPeerDeviceList.tail = 0;
	TsPeerDeviceList.maxListSize = UWB_MAX_QUERY_TIMESTAMP_LIST_SIZE;
	(void)memset(TsPeerDeviceList.DeviceIdList, 0xFF, TsPeerDeviceList.maxListSize);

	//--------------------------------------------------------------------------
	// [ Initial function ]
	//--------------------------------------------------------------------------
	Ranger4App_RegisterCallback(Ranger4App_RspCallback, Ranger4App_NtfCallback);
	Ranger4App_task_Init();

    //--------------------------------------------------------------------------
    // Test Pin
    //--------------------------------------------------------------------------
	#ifdef __HW_TEST_PIN_H
	CLOCK_EnableClock(kCLOCK_PortC);
	PORT_SetPinMux(PORTC, 7u, kPORT_MuxAsGpio);
	GPIO_PinInit(GPIOC, 7u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0U});

	PORT_SetPinMux(PORTC, 3u, kPORT_MuxAsGpio);
	GPIO_PinInit(GPIOC, 3u, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0U});
	#endif
}

/* FUNCTION ************************************************************************************************************
 * \brief        Query Time Stamp from UWB
 *
 * \param[in]    deviceId   0xF0 & valid DeviceId: start ranging after get timestamp
                            valid DeviceId: get timestamp for peer device time sync
 * END ****************************************************************************************************************/
void QueryUwbTimeStamp(deviceId_t deviceId)
{
    uint8_t freeListSize = 0;

    /* record peer device id that needs time sync*/
    if(TsPeerDeviceList.head >= TsPeerDeviceList.tail)
    {
        freeListSize = TsPeerDeviceList.maxListSize - (TsPeerDeviceList.head - TsPeerDeviceList.tail);
    }
    else
    {
        freeListSize = TsPeerDeviceList.tail - TsPeerDeviceList.head;
    }

    if(freeListSize != 0 )
    {
        TsPeerDeviceList.DeviceIdList[TsPeerDeviceList.head] = deviceId;
        TsPeerDeviceList.head++;
        if(TsPeerDeviceList.head >= TsPeerDeviceList.maxListSize)
        {
            TsPeerDeviceList.head = 0;
        }

        /* Send command to get current timestamp from UWB*/
        Ranger4UciCmd_ProprietaryQueryTimeStamp();
    }
    else
    {
        /*list is full*/
    	NVIC_SystemReset();
    }
}
/* FUNCTION ************************************************************************************************************
 * \brief        App_SetRangingSessionCfg
 *
 * \param[in]

 * END ****************************************************************************************************************/
void App_SetRangingSessionCfg(deviceId_t deviceId, SessionManagement_t * pSessionPara)
{
    uint8_t CfgSize = 0;
//    static uint32_t uwbInitTime = 0;
    phscaR4CadsTypesIntf_ErrorCode_t en_Status = errorCode_Success;
    Ranger4Uci_SessionCfgStructure_t * pRangingSessionCfg;

    pRangingSessionCfg = (Ranger4Uci_SessionCfgStructure_t*)pvPortMalloc(sizeof(Ranger4Uci_SessionCfgStructure_t) * 30);
    if (pRangingSessionCfg == NULL) {
        // Handle error
        return;
    }
    (void)memset((uint8_t *)pRangingSessionCfg, 0x00, sizeof(Ranger4Uci_SessionCfgStructure_t) * 30);

    //add configurations here
    pRangingSessionCfg[CfgSize].ID = R4_SESSION_UWB_CONFIG_ID;
    pRangingSessionCfg[CfgSize].length = 0x02U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SelectedUwbConfigId;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_PULSESHAPE_COMBO;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SelectedPulseShapeCombo;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_RANGING_INTERVAL;
    pRangingSessionCfg[CfgSize].length = 0x04U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SessionRangingBlockT;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_CHANNEL_ID;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SelectedChannel;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_RANGING_SLOT_LENGTH;
    pRangingSessionCfg[CfgSize].length = 0x02U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SessionChapsperSlot;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_NUMBER_OF_ANCHORS;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->Number_Responders_Nodes;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_SLOTS_PER_RR;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->Number_Slots_per_Round;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_HOPPING_MODE;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->SessionHopingMode;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_PREAMBLE_ID;
    pRangingSessionCfg[CfgSize].length = 0x01U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->Selected_SYNC_Code_Index;
    CfgSize++;

    pRangingSessionCfg[CfgSize].ID = R4_SESSION_STS_INDEX0;
    pRangingSessionCfg[CfgSize].length = 0x04U;
    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&pSessionPara->STS_Index0;
    CfgSize++;
    //add fixed configurations
    (void)memcpy(&pRangingSessionCfg[CfgSize], pSessionPara->pSessionfixedCfg, sizeof(Ranger4Uci_SessionCfgStructure_t) * pSessionPara->SessionSetCfgSize);
    CfgSize += pSessionPara->SessionSetCfgSize;

    //==========================================================================
    // UWB Initial Time
    //==========================================================================
    //Modify (Ken):NXP-V0001 NO.7 -20240507
    #if defined __FIT_UWB_RFIC_SYNC_H || defined __FIT_UWB_NoBLERanging_TEST_H
    // Local UWB Ranging session start late, start Ranging session immediatly
    #else
    //add UWB init time parameter
    pRangingSessionCfg[CfgSize].ID = R4_SESSION_UWB_INITIATION_TIME;
    pRangingSessionCfg[CfgSize].length = 4;
    // convert to millisecond
    uwbInitTime = UWB_INITIATION_TIME_DEMO + 225; //225 milliseconds

    pRangingSessionCfg[CfgSize].paramValue = (uint8_t*)&uwbInitTime;
    CfgSize++;
    #endif

    //==========================================================================
    //
    //==========================================================================
    en_Status = Ranger4UciCmd_MacSessionConfigure(pSessionPara->UwbSessionID, CfgSize, pRangingSessionCfg);

    vPortFree(pRangingSessionCfg);
    pRangingSessionCfg = NULL;

    if( en_Status == errorCode_InvalidRange)
    {
//        //too many configurations, split configurations
//        Ranger4UciCmd_MacSessionConfigure(pSessionPara->UwbSessionID, CfgSize/2, pRangingSessionCfg);
//
//        Ranger4UciCmd_MacSessionConfigure(pSessionPara->UwbSessionID, (CfgSize/2 + (CfgSize%2)), &pRangingSessionCfg[CfgSize/2]);
//
    }
    else if(en_Status == errorCode_Undefined)
    {
        //Out of memory
    	NVIC_SystemReset();
    }
}


/* FUNCTION ************************************************************************************************************
 * \brief        Process Time Stamp get from UWB
 *
 * \param[in]    pPayload   point of UWB UCI response frame

 * END ****************************************************************************************************************/
void ProcessUwbTimeStamp(uint8_t * pPayload, phscaR4CadsTypesIntf_UciStatusCode_t StateCode)
{
    uint8_t PeerId = 0;
    union
    {
        uint8_t Buf[8];
        uint64_t TsValue;
    }TsUwbTime;

    (void)memcpy(TsUwbTime.Buf, &pPayload[0], 8);
    /* Get nearst Id of device that needs time sync  */
    PeerId  = TsPeerDeviceList.DeviceIdList[TsPeerDeviceList.tail];

    /* Update device list to next device ID */
    TsPeerDeviceList.tail++;
    if(TsPeerDeviceList.tail >= TsPeerDeviceList.maxListSize)
    {
        TsPeerDeviceList.tail = 0;
    }

    if(StateCode != uciStatusCode_Ok)
    {
        QueryUwbTimeStamp(PeerId);
        return;
    }

    /* Update current timestamp */
    TsUwbCurrentTimeStamp = TsUwbTime.TsValue;

    if(PeerId < gAppMaxConnections_c)
    {
        /* Subtract ble event delay */
        mPeerInformation.TsUwbLocalDeviceTime = TsUwbTime.TsValue - (uint64_t)mPeerInformation.TsBleEvtDelay;

        if( mPeerInformation.NeedsSendTimeSyncForPhyUpdate )
        {
            mPeerInformation.NeedsSendTimeSyncForPhyUpdate = false;
        }
    }
    else
    {
        PeerId = PeerId & 0x0F;
        if(PeerId < gAppMaxConnections_c)
        {
            /* start reference time is 100 milliseconds after the present time,give vehicle enought time to setup uwb */
            UwbSessions.UWB_Time0 = TsUwbCurrentTimeStamp + UWB_INITIATION_TIME_DEMO * 1000;
            /* add Uncertainty time as max 4 milliseconds */
            UwbSessions.UWB_Time0 = UwbSessions.UWB_Time0 + (4u * 1000u);
            App_SetRangingSessionCfg(PeerId, &UwbSessions);
        }
    }
    #if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
    log_debug("\r\n Ranger4App_RspCallback ---> Uwb Current TimeStamp is:");
    log_debug_hexBigend((uint8_t *)&TsUwbCurrentTimeStamp,8);
    log_debug(" \r\n");
    #endif
}

/* FUNCTION ************************************************************************************************************
 * \brief        Extraction capability required by CCC capability exchange process
 *
 * \param[in]    payload   point of UWB UCI response frame

 * END ****************************************************************************************************************/
void GetCapabilityData(uint8_t *payload)
{
    uint8_t i, j, TlvCnt, TlvOffset;

    TlvCnt = payload[1];
    TlvOffset = 2;
    for(i = 0; i < TlvCnt; i++)
    {
        switch (payload[TlvOffset])
        {
            case R4_DEVICE_CAP_SLOT_BITMASK:
            {
                UwbCapability.Slot_bitMask = payload[TlvOffset+2];
            }
            break;

            case R4_DEVICE_CAP_SYNC_CODE_INDEX_BITMASK:
            {
            	(void)memcpy(UwbCapability.SYNC_Code_Index_Bitmaks, &payload[TlvOffset+2], 4);
            }
            break;

            case R4_DEVICE_CAP_HOPPING_CONFIG_BITMASK:
            {
                UwbCapability.Hopping_Config_Bitmask = payload[TlvOffset+2];
            }
            break;

            case R4_DEVICE_CAP_CHANNEL_BITMASK: //Channel Bitmask
            {
                UwbCapability.Channel_BitMask = payload[TlvOffset+2];
            }
            break;

            case R4_DEVICE_CAP_SUPPORTED_PROTOCOL_VERSION: //Supported Protocol Version
            {
                UwbCapability.ProtoVer_Len = payload[TlvOffset + 1];
                for(j = 0; j < UwbCapability.ProtoVer_Len; j += 2)
                {
                	MemCpyReverseOrder(&UwbCapability.Supported_Protocol_Version[j],
                                            &payload[TlvOffset + 2 + j],
                                            2);
                }
            }
            break;

            case R4_DEVICE_CAP_SUPPORTED_UWB_CONFIG_ID: //Supported Configure Id
            {
                UwbCapability.CfgId_Len = payload[TlvOffset + 1];
                for(j = 0; j < UwbCapability.CfgId_Len; j += 2)
                {
                	(void)memcpy(&UwbCapability.Supported_Cfg_Id[j], &payload[TlvOffset + 2 + j], 2);
                }
            }
            break;

            case R4_DEVICE_CAP_SUPPORTED_PULSESHAPE_COMBO: //Supported Pluse shape Combination
            {
                UwbCapability.PluseshapeCombo_Len = payload[TlvOffset + 1];
                (void)memcpy(UwbCapability.Supported_Pluseshape_Combo, &payload[TlvOffset + 2], UwbCapability.PluseshapeCombo_Len);
            }
            break;

            default:
                /* Other capability parameters are not used in this demo, ignore  */
            break;
        }

        TlvOffset += payload[TlvOffset + 1];//next TLV start point
        TlvOffset += 2;
    }
}

/* FUNCTION ************************************************************************************************************
 * \brief        UCI response callback function, this function could be rework by
 *               customer application code
 *
 * \param[in]    frame   UCI response frame

 * END ****************************************************************************************************************/
void Ranger4App_RspCallback(phscaR4CadsTypesIntf_UciFrame_t * frame)
{
    phscaR4CadsTypesIntf_UciStatusCode_t StatusCode;
    uint8_t sGroupCode, sOpCode;

    //==========================================================================
    // Get Payload Data
    //==========================================================================
    StatusCode = (phscaR4CadsTypesIntf_UciStatusCode_t)frame->uciPacket.payload[0];
    sGroupCode = frame->uciPacket.header.groupId;
    sOpCode    = frame->uciPacket.header.opcodeId;

    //==========================================================================
    // State Error
    //==========================================================================
//    if(StatusCode != uciStatusCode_Ok){
//        return;
//    }

    //==========================================================================
    // Group Identifier (GID)
    //==========================================================================
    switch(sGroupCode)
    {
		case PHSCA_R4CADSTYPES_UCI_GID_CORE:
		{
			switch (sOpCode)
			{
				case R4_UCI_CORE_OID_RESET_DEVICE:
				{
					if(StatusCode == uciStatusCode_Ok)
					{
					}
				} break;

				case R4_UCI_CORE_OID_CORE_GET_DEVICE_INFO:
					break;

				case R4_UCI_CORE_OID_CORE_GET_CAPS_INFO:
				{
					if(StatusCode == uciStatusCode_Ok)
					{
						if( mIsRanger4Init == false )
						{
							//set core configuration, to prevent UWB(NCJ29D5D) enter Low power mode
							//just disable UWB enter low power mode
							Ranger4UciCmd_MacCoreSetConfiguration(UwbCoreCfgSize, UwbCoreCfg);
						}
						/* extract capability that required during CCC capability exchange */
						GetCapabilityData(frame->uciPacket.payload);
					}
				} break;

				case R4_UCI_CORE_OID_CORE_GET_CONFIG:
					break;

				case R4_UCI_CORE_OID_CORE_SET_CONFIG:
				{
					if(StatusCode == uciStatusCode_Ok)
					{
						/* Set core configuration complete */
						if( mIsRanger4Init == false )
						{
							/* Start UWB timer by calling query UWB timestamp command */
							QueryUwbTimeStamp(gInvalidDeviceId_c);
						}

						//Jia: Just for test UWB without BLE
						#if defined __FIT_UWB_NoBLERanging_TEST_H
						UwbSessions.UwbSessionID = 0x01;
						Ranger4UciCmd_MacSessionCfgInit(UwbSessions.UwbSessionID, UwbSessions.SessionType);
						#endif
					}
					else
					{}
				} break;

				default:
					break;
			}
		} break;


		case PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG:
		{
			switch(sOpCode)
			{
				case R4_UCI_SESSION_CFG_OID_INIT://session init complete
					if(StatusCode == uciStatusCode_Ok)
					{
						//Jia: Just for test UWB without BLE
//                    Ranger4UciCmd_MacSessionConfigure(UwbSessions.UwbSessionID,
//                                                      UwbSessions.SessionSetCfgSize,
//                                                      UwbSessions.pSessionCfg);

						#if defined __FIT_UWB_RFIC_SYNC_H ||  defined  __FIT_UWB_RangingSpeed_TEST_H || defined __FIT_UWB_NoBLERanging_TEST_H
						App_SetRangingSessionCfg(UwbSessions.UwbSessionID, &UwbSessions);
						mIsRangingStart = true;
						#endif
					}
					else
					{}
					break;
				case R4_UCI_SESSION_CFG_OID_SET_APP_CFG: //session config complete
					if(StatusCode == uciStatusCode_Ok)
					{
						Ranger4UciCmd_MacSessionStartRanging(UwbSessions.UwbSessionID);
						#if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
						log_debug("\r\n Ranger4App_RspCallback ---> ranging start, session ID is:\r\n");
						log_debug_hexBigend((uint8_t*)&UwbSessions.UwbSessionID,4);
						log_debug(" \r\n");
						#endif
						__TEST_TogglePTC(7);
					}
					else
					{

					} break;

				default:
					break;
			}
		} break;

		case PHSCA_R4CADSTYPES_UCI_GID_RANGING_SESSION_CONTROL:
		{
			switch(sOpCode)
			{
				case R4_UCI_RANGE_CTR_OID_START://session init complete
				  {
					__TEST_TogglePTC(7);            // Session Step 3
				  } break;
				case R4_UCI_RANGE_CTR_OID_STOP: //session config complete
				  {
					mIsRangingStart = false;
				  } break;
				default:
					break;
			}
		} break;

		case PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN:
		{
			switch(sOpCode)
			{
				case R4_UCI_PROPRIETARY_OID_QUERY_UWB_TIMESTAMP:
				{
					if(StatusCode == uciStatusCode_Ok)
					{
						if( mIsRanger4Init == false )
						{
							mIsRanger4Init = true;
						}
					}

					ProcessUwbTimeStamp(&frame->uciPacket.payload[1], StatusCode);
				} break;

				default:
					break;
			}
		} break;

		default:
			break;
    }
}

/* FUNCTION ************************************************************************************************************
 * \brief        UCI notification callback function, this function could be rework by
 *               customer application code
 *
 * \param[in]    frame   UCI response frame

 * END ****************************************************************************************************************/
void Ranger4App_NtfCallback(phscaR4CadsTypesIntf_UciFrame_t * frame)
{
	Ranger4Uci_SessionStatusType_t	SessionSTA_Tmp;
    //==========================================================================
    // Group Identifier (GID)
    //==========================================================================
    switch (frame->uciPacket.header.groupId)
    {
        case PHSCA_R4CADSTYPES_UCI_GID_CORE:
        {
            if(frame->uciPacket.header.opcodeId == R4_UCI_CORE_OID_CORE_DEVICE_STATUS_NTF)
            {
                //This state is the first state of UWBS after power ON/Reset or on receipt of DEVICE_RESET_CMD from host.
                mUwbDeviceStatus = (Ranger4Uci_DeviceStatusType_t)frame->uciPacket.payload[0];
                if(mUwbDeviceStatus >= R4_UCI_DEVICE_STATUS_NOT_SUPPORT)
                {
                    /* When the device type is not supported, only CORE_GET_DEVICE_INFO_CMD and CORE_RESET_DEVICE_CMD are
                       supported, it need to check if UWB device support UCI in hardware level*/
                    #if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
                    log_debug("\r\n Ranger4App_NtfCallback ---> Device respone not support UCI, please check UWB device firmware. \r\n");
                    #endif
                    __NOP();
                }
                else
                {
                    if(mUwbDeviceStatus == R4_UCI_DEVICE_STATUS_READY)
                    {
                        if(mIsRanger4Init == false)
                        {
                            /* UWB status is ready after ranger4 initialized, then read capability from UWB */
                            Ranger4UciCmd_MacCoreGetCapsInfo();

                            /* Session variable init*/
                            Ranger4App_InitDefaultSessionCfg(&UwbSessions);
                            UwbSessions.SessionStatus = R4_SESSION_STATE_DEINIT;
                        }
                        else
                        {
                            /* UWB status is ready, on other case */
                        }
                    }
                }
            }

            if(frame->uciPacket.header.opcodeId == R4_UCI_CORE_OID_CORE_GENERIC_ERROR_NTF)
            {
                /* This Notification is used in error situations when the error cannot
                   be notified using an error status in a Response Message*/
            }
            break;
        }

        case PHSCA_R4CADSTYPES_UCI_GID_SESSION_CONFIG:
        {
        	SessionSTA_Tmp = UwbSessions.SessionStatus;

            if(frame->uciPacket.header.opcodeId == R4_UCI_SESSION_CFG_OID_STATUS_NTF)
            {
                UwbSessions.UwbSessionID  = (uint32_t)frame->uciPacket.payload[0];
                UwbSessions.UwbSessionID |= ((uint32_t)frame->uciPacket.payload[1]) << 8;
                UwbSessions.UwbSessionID |= ((uint32_t)frame->uciPacket.payload[2]) << 16;
                UwbSessions.UwbSessionID |= ((uint32_t)frame->uciPacket.payload[3]) << 24;
                UwbSessions.SessionStatus = frame->uciPacket.payload[4];
                UwbSessions.SessionStatusChangeReason = frame->uciPacket.payload[5];

                if(SessionSTA_Tmp == R4_SESSION_STATE_ACTIVE && UwbSessions.SessionStatus==R4_SESSION_STATE_IDLE)
                {
                	UWB_Api_Stop_Ranging(UwbSessions.UwbSessionID);
                }
            }
            break;
        }
        case PHSCA_R4CADSTYPES_UCI_GID_RANGING_SESSION_CONTROL:
         {
             if(frame->uciPacket.header.opcodeId == R4_UCI_RANGE_CTR_OID_CCC_DATA_NTF)
             {
                __TEST_TogglePTC(7);

                switch ((uint8_t)frame->uciPacket.payload[4])
                {
                    case 0x00:
                      {
                        __NOP();
                      } break;

                    case 0x01:
                      {
                        __NOP();
                      } break;

                    case 0x02:
                      {
                        __TEST_TogglePTC(3);
                      } break;

                    case 0x03:
                      {
                        __NOP();
                      } break;
                }
             }
             break;
         }
         case PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN:
         {
             if(frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_LOG_NTF)
             {
             }
             if(frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_TEST_STOP_NTF)
             {
             }
             if(frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_TEST_LOOPBACK_NTF)
             {
             }
             if(frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_SET_TRIM_VALUE_NTF)
             {
             }
             break;
         }
         default:
             /*JIA: For other group notification, not sure how to handle it, just ignore it */
             break;

    }
}


/* FUNCTION ************************************************************************************************************
 *
 * Function Name :
 * Description   :
 * 
 *
 * END ****************************************************************************************************************/


#endif




