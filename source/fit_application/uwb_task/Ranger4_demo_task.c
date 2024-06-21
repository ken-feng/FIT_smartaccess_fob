//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
/*! *********************************************************************************
* Include
********************************************************************************** */
#include "fsl_common.h"
#include "EmbeddedTypes.h"

#include "phscaR4DriverHw.h"
#include "phscaR4UciSpi.h"
#include "Ranger4UciCmd.h"
#include "Ranger4_demo_task.h" 

//Modify (Ken):NXP-V0001 NO.3 -20240319
#if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
#include "segger_rtt_utils.h"
#endif

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "message_queue.h"

/*! *********************************************************************************
* Private macros
********************************************************************************** */
#define R4SWUP_UART_CMD_START_SWUP        0xF1
#define R4SWUP_UART_CMD_GET_NEW_BLOCK     0xF2
#define R4SWUP_UART_CMD_TRANSFER_COMPLETE 0xF3
#define R4SWUP_UART_CMD_STOP_TRANSFER     0xF4
#define R4SWUP_UART_CMD_ERROR_NOTIFY      0xF5

#define R4SWUP_UART_CMDTYPE_REQUEST       0x00
#define R4SWUP_UART_CMDTYPE_COMFIRM       0x01
#define R4SWUP_UART_CMDTYPE_INDECATION    R4SWUP_UART_CMDTYPE_REQUEST

/* Task priorities. */
#define Ranger4Apptask_PRIORITY 		  (configMAX_PRIORITIES - 1)
#define configRanger4AppTask_STACK_SIZE   ((unsigned short)1000)

#define MessageQueue_Pending(q) 		  (!MessageQueue_IsEmpty(q))
/*! *********************************************************************************
* Private function declarations
********************************************************************************** */
static void Ranger4App_task(void *pvParameters);
//static void Ranger4App_task
//(
//    osaTaskParam_t param
//);
static void Ranger4App_RunningHandler(void);
//static void Ranger4App_DeviceSWUPHandler(void);
static void Ranger4App_Interrupt (void *pParam);
static void Ranger4App_RcvDateFromR4Handle(phscaR4CadsTypesIntf_UciFrame_t * frame);

/*! *********************************************************************************
* Private memory declarations
********************************************************************************** */
//static bool                          mRanger4App_SwupRequire = false;
//static osaEventFlags_t               mRanger4App_event = 0U;
EventGroupHandle_t 			 		 mRanger4ThreadEventId = NULL;
EventBits_t 					 	 mRanger4App_event = 0U;
//TaskHandle_t    					 mRanger4ThreadId;

static bool                          mRanger4App_SwupRequire = false;
static Ranger4App_Status_t           mRanger4App_Status = Ranger4App_Init;
static Ranger4HandleUciRcvCallback_t handleUciNtfCallback= NULL;
static Ranger4HandleUciRcvCallback_t handleUciRspCallback= NULL;

//Ranger4Uci_SessionStatus_t mUwbSessionStatus;

/*! *********************************************************************************
* Public memory declarations
********************************************************************************** */
//OSA_TASK_DEFINE(Ranger4App_task, gRanger4TaskPriority_c, 1, gRanger4TaskStackSize_c, FALSE );
//osaEventId_t   mRanger4ThreadEventId;
//osaTaskId_t    mRanger4ThreadId;
Ranger4UciCmd_TransferStateManagement_t 	mRanger4TransSta;
MessageQueue                     			mRanger4SendPktMsgQueue;
MessageQueue                     			mRanger4RcvPktMsgQueue;
phscaR4CadsTypesIntf_UciFrame_t  			m_Ranger4UciRcvBuffer;

extern uint8_t gAppSerMgrIf;
extern Ranger4Uci_SessionCfgStructure_t UwbSessionCfg[__InitSessionCfg_Size];
/*! *********************************************************************************
* Public functions
********************************************************************************** */
void Ranger4App_InitDefaultSessionCfg(SessionManagement_t * pSessionCfg)
{
    pSessionCfg->UwbSessionID = 0x01;
    pSessionCfg->pSessionfixedCfg = UwbSessionCfg;
    pSessionCfg->SessionSetCfgSize = NumberOfElements(UwbSessionCfg);
    pSessionCfg->SessionType = R4_SESSION_CCC_RANGING_SESSION;
    pSessionCfg->RAN_Multiplier = 1;
    pSessionCfg->Number_Chaps_per_Slot = 3;
    pSessionCfg->Selected_Hopping_Config_Bitmask = 0x80;
    
    pSessionCfg->SelectedDkProtolVersion[0] = 0x01;
    pSessionCfg->SelectedDkProtolVersion[1] = 0x00;
    
    pSessionCfg->SelectedUwbConfigId  = 0x0000;
    
    pSessionCfg->SelectedPulseShapeCombo = 0x11;
    
    pSessionCfg->SessionRangingBlockT = pSessionCfg->RAN_Multiplier * CCC_MIN_BLOCK;
    
    pSessionCfg->SelectedChannel = 0x09;

    pSessionCfg->SessionChapsperSlot = pSessionCfg->Number_Chaps_per_Slot * CCC_TCHAP;

    //Modify (Ken):NXP-V0001 NO.5 -20240319
    #ifdef  __UWB_ROLE_RESPONDER_H
    pSessionCfg->Number_Responders_Nodes = NUM_ANCHORS;         //Responder
    //Modify (Ken):NXP-V0001 NO.4 -20240506
    #elif defined   __FIT_UWB_NoBLERanging_TEST_H
    pSessionCfg->Number_Responders_Nodes = 1;                   //Initiator (needs get from anchor side)
    #else
    pSessionCfg->Number_Responders_Nodes = 0;                   //Initiator
    #endif

    pSessionCfg->Number_Slots_per_Round = 12; 
    
    /* because we don't use hopping mode, so just set to no hopping mode */     
    pSessionCfg->SessionHopingMode = 0x00;    
    
    pSessionCfg->Selected_SYNC_Code_Index = 0x0A;

    pSessionCfg->STS_Index0 = 0x00000000;
    
    pSessionCfg->UWB_Time0 = 0;
}


//******************************************************************************
// Setting Callback function for Notification and Response from NCJ29D5
//******************************************************************************
void Ranger4App_RegisterCallback(Ranger4HandleUciRcvCallback_t  RspCallback,
                                 Ranger4HandleUciRcvCallback_t  NtfCallback)
{
    handleUciRspCallback = RspCallback;
    handleUciNtfCallback = NtfCallback;
}
//******************************************************************************


//******************************************************************************
// Range Task Initialize
//******************************************************************************
/*! -------------------------------------------------------------------------
 * \brief initialize the Ranger4 task
 *---------------------------------------------------------------------------*/
void Ranger4App_task_Init(void)
{
    static uint8_t R4Task_initialized = 0;

    if( !R4Task_initialized )
    {
        #if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
        log_debug("\r\n UWB task init.\r\n");        
        #endif
        //======================================================================
        /* Initialization of hardware*/
        //======================================================================
        (void)phscaR4UciSpi_InterfaceInit();
        phscaR4Driver_SetINTCallBack(Ranger4App_Interrupt, NULL);
        
        //======================================================================
        //First time runing this task
        //======================================================================
        mRanger4App_Status = Ranger4App_Init;
        //======================================================================
        /* Prepare callback input queue.*/
        //======================================================================
        MessageQueue_Init(&mRanger4SendPktMsgQueue);
        MessageQueue_Init(&mRanger4RcvPktMsgQueue);
        
        //======================================================================
        /* Event*/
        //======================================================================
        mRanger4ThreadEventId = xEventGroupCreate();

        if( NULL == mRanger4ThreadEventId )
        {
//            panic( 0, (uint32_t)Ranger4App_task_Init, 0, 0 );
        	__NOP();
        }
        else
        {
            //------------------------------------------------------------------
            /* Creat task*/
            //------------------------------------------------------------------
            if (xTaskCreate(Ranger4App_task, "Ranger4AppTask", configRanger4AppTask_STACK_SIZE, NULL, Ranger4Apptask_PRIORITY, NULL) != pdPASS)
            {
				#if (defined(SEGGER_RTT_ENABLE) && (SEGGER_RTT_ENABLE == 1U))
            	PRINTF("uart Task creation failed!.\r\n");
				#endif
                while (1)
                    ;
            }
        }
        
        //======================================================================
        /* Finish */
        //======================================================================
        R4Task_initialized = 1;
    }
}
//******************************************************************************


//******************************************************************************
// Range Task Process
//******************************************************************************
static void Ranger4App_task
(
//    osaTaskParam_t param
	void *pvParameters
)
{ 
    while(1)
    {
        switch (mRanger4App_Status)
        {
            case Ranger4App_Init:
            {
                mRanger4TransSta.state = TransferState_Idle;
                /* Switch to next state */
                mRanger4App_Status = Ranger4App_HardwareResetDevice;
                break;
            }
        
            case Ranger4App_HardwareResetDevice:
            {
                /* Reset device by pin*/
                phscaR4UciSpi_HardwareResetRanger4();
                /* Switch to running state*/
                if(mRanger4App_SwupRequire)
                {
                    mRanger4App_Status = Ranger4App_DeviceSWUP;
                }
                else
                {
                    mRanger4App_Status = Ranger4App_Running;
                }
                break;
            }
        
            case Ranger4App_Running:
            {
                Ranger4App_RunningHandler();
                break;
            }
//            case Ranger4App_DeviceSWUP:
//            {
//                Ranger4App_DeviceSWUPHandler();
//                break;
//            }        
            case Ranger4App_UciCommunicationError:
            {
                break;
            }
        
            case Ranger4App_CriticalError:
            {
                break;
            }
            case Ranger4App_UndefinedError:
            {
                break;
            }
            default:
            {
                /* Do nothing */
                break;
            }  
        }
    }
}
//******************************************************************************


//******************************************************************************
// Ranging Evet Proccess
//******************************************************************************
static void Ranger4App_RunningHandler(void)
{  
    phscaR4CadsTypesIntf_ErrorCode_t error = errorCode_Success;
    phscaR4CadsTypesIntf_UciFrame_t *pMsg;

    //==========================================================================
    // Check (Set Task uxStackDepth size #define INCLUDE_uxTaskGetStackHighWaterMark 1 in FreeRTOSConfig.h)
    //==========================================================================
//    UBaseType_t uxHighWaterMark;
//
//    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);		// size is 843
//    __NOP();

    //==========================================================================
    //
    //==========================================================================
    mRanger4App_event = xEventGroupWaitBits(mRanger4ThreadEventId, gR4EvtFlag_ApplicationALL_c, pdTRUE, pdFALSE, portMAX_DELAY);

    //==========================================================================
    // wait receive response after send data in Send Sync
    //==========================================================================
    if(mRanger4App_event & gR4EvtUwb2HostMsgReceivedEvent_c)
    {
    	pMsg = pvPortMalloc(sizeof(phscaR4CadsTypesIntf_UciFrame_t));
    	if (pMsg == NULL) {
    	    // Handle error
    		return;
    	}
        pMsg = MessageQueue_Get(&mRanger4RcvPktMsgQueue);
        /* process received data */
        Ranger4App_RcvDateFromR4Handle(pMsg);

        vPortFree(pMsg);
        pMsg = NULL;
    }

    //==========================================================================
    // when interrupt from ncj29d5
    //==========================================================================
    if(mRanger4App_event & gR4EvtUwb2HostMsgAvailable_c)
    {
        /* Receive data over UCI */
        error = phscaR4UciSpi_ReceiveData(&m_Ranger4UciRcvBuffer);
        if(error == errorCode_Success)
        {
            /* process receive data */
            Ranger4App_RcvDateFromR4Handle(&m_Ranger4UciRcvBuffer);
        }
        else if((error == errorCode_UciNoEvent))//Jia need compare with CADS
        {
            /* Do nothing, notification may already read in other branch */
        }
        else
        {}
    }

    //==========================================================================
    // Transfer packet to ncj29d5
    //==========================================================================
    if(mRanger4App_event & gR4EvtTransferCmdToR4Event_c)
    {
        if(MessageQueue_Pending(&mRanger4SendPktMsgQueue) && (mRanger4TransSta.state == TransferState_Idle))
        {
            pMsg = MessageQueue_Get(&mRanger4SendPktMsgQueue);
            error = phscaR4UciSpi_TransferData((phscaR4CadsTypesIntf_UciFrame_t *)pMsg, &m_Ranger4UciRcvBuffer);

            //------------------------------------------------------------------
            // Command Send complete
            //------------------------------------------------------------------
            if(error == errorCode_UciSendSuccess )
            {
                mRanger4TransSta.state = TransferState_Busy;
                /* Send success, save sent packet data. */
                memcpy(mRanger4TransSta.PreviousUciFrame.bytes, pMsg->bytes, pMsg->uciPacket.header.payloadLength + 4);
                /* Messages must be freed*/
//                (void)free(pMsg);
            }
            //------------------------------------------------------------------
            // Command message not send
            //------------------------------------------------------------------
            else
            {
                if(error == errorCode_UciRecvSuccess)
                {
                    //Process received data
                    Ranger4App_RcvDateFromR4Handle(&m_Ranger4UciRcvBuffer);
                    //Command message not send, not need to free Command msg, we need to
                    //send it in next loop, and it in queue head, wait for next send
                    MessageQueue_PutHead(&mRanger4SendPktMsgQueue, *pMsg);


                    //Re-call event to send pkg
                    (void)xEventGroupSetBits(mRanger4ThreadEventId, gR4EvtTransferCmdToR4Event_c);
                }
                else if(error == errorCode_UciR4NotReadyError)
                {
                    //Command message not send, not need to free Command msg, we need to
                    //send it in next loop, and it in queue head, wait for next send
                    MessageQueue_PutHead(&mRanger4SendPktMsgQueue, *pMsg);
                    //Re-call event to send pkg
                    (void)xEventGroupSetBits(mRanger4ThreadEventId, gR4EvtTransferCmdToR4Event_c);
                }
                else
                {
                    //errorCode_UciGenericError
                    mRanger4App_Status = Ranger4App_UndefinedError;
                    /*Send success, Messages must be freed. */
//                    (void)free(pMsg);
                }
            }
        }
    }

    //==========================================================================
    // check Send packet queue
    //==========================================================================
    if(MessageQueue_Pending(&mRanger4SendPktMsgQueue))
    {
        (void)xEventGroupSetBits(mRanger4ThreadEventId, gR4EvtTransferCmdToR4Event_c);
    }
}
//******************************************************************************


//******************************************************************************
// 
//******************************************************************************
static void Ranger4App_RcvDateFromR4Handle(phscaR4CadsTypesIntf_UciFrame_t * frame)
{
    phscaR4CadsTypesIntf_UciFrame_t * pUciCmdPkt;
    
    //==========================================================================
    // CCC Notification
    //==========================================================================
    if(frame->uciPacket.header.msgType == PHSCA_R4CADSTYPES_UCI_MT_CMD_NTF)
    {
        if(handleUciNtfCallback != NULL)
        {
            handleUciNtfCallback(frame);
        }
    }
    //==========================================================================
    // CCC Response
    //==========================================================================
    else if (frame->uciPacket.header.msgType == PHSCA_R4CADSTYPES_UCI_MT_CMD_RSP)
    {
        /* Both GID and OID received is same as previous send CMD, means response corresponding command */
        if((mRanger4TransSta.PreviousUciFrame.uciPacket.header.groupId == frame->uciPacket.header.groupId) 
           && (mRanger4TransSta.PreviousUciFrame.uciPacket.header.opcodeId == frame->uciPacket.header.opcodeId))
        {
            if(frame->uciPacket.payload[0] == uciStatusCode_Command_Retry)
            {
                //UWB request retransmission UCI frame, add previous frame in queue head
                pUciCmdPkt = (phscaR4CadsTypesIntf_UciFrame_t *)pvPortMalloc(PHSCAR4_UCI_HEADER_LENGTH + mRanger4TransSta.PreviousUciFrame.uciPacket.header.payloadLength);
                if (pUciCmdPkt == NULL) {
                    // Handle error
                    return;
                }
                memcpy(pUciCmdPkt, mRanger4TransSta.PreviousUciFrame.bytes, (PHSCAR4_UCI_HEADER_LENGTH + mRanger4TransSta.PreviousUciFrame.uciPacket.header.payloadLength));
                (void)MessageQueue_Put(&mRanger4SendPktMsgQueue, *pUciCmdPkt);
                //Re-call event to send pkg
                (void)xEventGroupSetBits(mRanger4ThreadEventId, gR4EvtTransferCmdToR4Event_c);
                vPortFree(pUciCmdPkt);
                pUciCmdPkt = NULL;
            }
            else
            {
                /*Received response, then next command can be send */
                mRanger4TransSta.state = TransferState_Idle;
                if(handleUciRspCallback != NULL)
                {
                    handleUciRspCallback(frame);
                }
            }
        }
        else
        {
            /* For R4Pkt_SendSync */
            if((frame->uciPacket.header.groupId == PHSCA_R4CADSTYPES_UCI_GID_PROPRIETARY_MIN) 
               && (frame->uciPacket.header.opcodeId == R4_UCI_PROPRIETARY_OID_QUERY_UWB_TIMESTAMP))
            {
                if(handleUciRspCallback != NULL)
                {
                    handleUciRspCallback(frame);
                }
            }
        }
    }
    //==========================================================================
    //
    //==========================================================================
    else
    {
        /*  Other msg type should not be receive on device, if it received, ignore that*/
    }
}
//******************************************************************************


//******************************************************************************
// GPIO INT Interrupt from NCJ29D5
//******************************************************************************
static void Ranger4App_Interrupt (void *pParam)
{
    (void)xEventGroupSetBits(mRanger4ThreadEventId, gR4EvtUwb2HostMsgAvailable_c);
}
//******************************************************************************

#endif  // for UWB_Support

