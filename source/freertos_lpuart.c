/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "board.h"

#include "fsl_lpuart_freertos.h"
#include "fsl_lpuart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "SEGGER_RTT.h"

#include "phscaR4UciSpi.h"
#include "fit_uwb_range_app.h"

#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
#define DEMO_LPUART LPUART1
#else
#define DEMO_LPUART LPUART0
#endif
#define DEMO_LPUART_CLKSRC BOARD_DEBUG_UART_CLKSRC
#define DEMO_LPUART_CLK_FREQ CLOCK_GetFreq(BOARD_DEBUG_UART_CLKSRC)
#define DEMO_LPUART_IRQn LPUART0_LPUART1_IRQn
#define DEMO_LPUART_IRQHandler LPUART0_LPUART1_IRQHandler

#define BOARD_LED_GPIO BOARD_LED_BLUE_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_BLUE_GPIO_PIN

#define BOARD_SW_GPIO BOARD_SW2_GPIO
#define BOARD_SW_PORT BOARD_SW2_PORT
#define BOARD_SW_GPIO_PIN BOARD_SW2_GPIO_PIN
#define BOARD_SW_IRQ BOARD_SW2_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW2_IRQ_HANDLER
#define BOARD_SW_NAME BOARD_SW2_NAME
#define APP_DBG_LOG(...)                     SEGGER_RTT_printf(0,__VA_ARGS__)


/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 2)
#define sw_task_PRIORITY (configMAX_PRIORITIES - 3)

#define configUartAppTask_STACK_SIZE	((unsigned short)500)
#define configSwAppTask_STACK_SIZE   	((unsigned short)500)

// RFIC Command format
#define __FIT_UART_CMD_H				0
#define __FIT_UART_LENGTH_H				1
#define __FIT_UART_DATA_START_H			2
#define __FIT_UART_DATA_MAX_SIZE_H		16

// RFIC State
#define __FIT_RFIC_PWRON_START			1
#define __FIT_RFIC_PWRON_COMPLETE		2

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void uart_task(void *pvParameters);
static void sw_task(void *pvParameters);
static uint8_t RFIC_cmd_compose_send(uint8_t);
static void RFIC_cmd_from_RFIC(uint8_t *fromstr);

/*******************************************************************************
 * Code
 ******************************************************************************/
#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
const char *to_send               = "FreeRTOS UWB_Fob_LPUART example!\r\n";
#else
const char *to_send               = "FreeRTOS LPUART driver example!\r\n";
#endif

const char *send_ring_overrun     = "\r\nRing buffer overrun!\r\n";
const char *send_hardware_overrun = "\r\nHardware buffer overrun!\r\n";
uint8_t background_buffer[20];
volatile bool g_ButtonPress = false;

//------------------------------------------------------------------------------
// RFIC Register
//------------------------------------------------------------------------------
static bool uart_sof = false;
static bool uart_eof = false;

uint8_t uart_recv_buffer[1];
uint8_t uart_recvstr[20];
uint8_t uart_recv_cnt=0;
uint8_t uart_recv_maxcnt=0;
uint8_t uart_recv_cs=0;

//------------------------------------------------------------------------------
// fob system Register
//------------------------------------------------------------------------------
static uint8_t fob_sys_rf_poweron_step = 0;			// The power-on sequence of an RFIC consists of 3 stages

//------------------------------------------------------------------------------
static uint8_t RKE_ARM =1;

lpuart_rtos_handle_t handle;
struct _lpuart_handle t_handle;

lpuart_rtos_config_t lpuart_config = {
    .baudrate    = 115200,
    .parity      = kLPUART_ParityDisabled,
    .stopbits    = kLPUART_OneStopBit,
    .buffer      = background_buffer,
    .buffer_size = sizeof(background_buffer),
};


typedef enum
{
    oACK_NAK,
	oSW_Reset_RFIC=0x01,
	oPowerOn_Info_Rpt,
	oSet_TX_Power,
	oStartStop_RFIC,
	oSet_RFIC_Interval_Time,
	oNotify_RFIC_Info,
	oRead_RFIC_Info,
	oRFIC_Rpt_from_RFIC,

	oRKE_command=0x20,

	oInto_Pairing=0x30,
	oPairing_Result,
	oRF_Comm_Auth_Result,
	oErase_Pairing,
	oPairing_Result_After_Erase,
	oRFIC_Test=80,


} RFIC_operate_t;

enum{
	lenSW_Reset_RFIC=4,
	lenPowerOn_Info_Rpt=9,
	lenSet_TX_Power=4,
	lenStart_RFIC=8,
	lenStop_RFIC=8,
	lenSet_RFIC_Int_Time=7,
	lenNotify_RFIC_Info=5,
	lenRead_RFIC_Info=4,

};

static uint8_t RFIC_pkt_len[] ={
		4,       //lenACK
		4,       //lenSW_Reset_RFIC=4,
		9,       //lenPowerOn_Info_Rpt=9,
		4,       //lenSet_TX_Power=4,
		8,       //lenStart_RFIC=8,
		8,       //lenStop_RFIC=8,
		7,       //lenSet_RFIC_Int_Time=7,
		5,       //lenNotify_RFIC_Info=5,
		4,        //lenRead_RFIC_Info=4,
		0,0,0,0,0,0,0,0,0,0,0,
		5,        //lenRKE_command=5
		0,0,0,0,0,0,0,0,0,
		4,       //lenIntoPairing


};

static uint8_t strdone[40]={0x24};
static uint8_t waitfor=0;
static uint8_t resendcnt=0;
static uint16_t         RFIC_433_Int_time =0x0BB8;
static uint16_t         RFIC_125_Int_time =0x08BB;
static uint8_t          RFIC_TX_power =3;

/*!
 * @brief Application entry point.
 */

//void BOARD_SW_IRQ_HANDLER(void)
//{
//    /* Clear external interrupt flag. */
//    GPIO_PortClearInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
//    /* Change state of button. */
//    g_ButtonPress = true;
//
//
///* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
//  exception return operation might vector to incorrect interrupt */
//#if defined __CORTEX_M && (__CORTEX_M == 4U)
//    __DSB();
//#endif
//}

int main(void)
{

    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput,
        0,
    };


    //====================================================================
    /* Init board hardware. */
    //====================================================================
    BOARD_InitPins();
    BOARD_BootClockRUN();
    //====================================================================
    // 0b00..Clock disabled
    // 0b01..MCGFLLCLK clock
    // 0b10..OSCERCLK clock (external reference clock)
    // 0b11..MCGIRCLK clock (internal reference clock)
    //====================================================================
	#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
    CLOCK_SetLpuart1Clock(0x2U);	//UART1 External
	#else
	CLOCK_SetLpuart0Clock(0x2U);	//UART0 External
	#endif


    //====================================================================
    // Switch pin trigger Setting
    //====================================================================
//    PORT_SetPinInterruptConfig(BOARD_SW_PORT, BOARD_SW_GPIO_PIN, kPORT_InterruptFallingEdge);
//    EnableIRQ(BOARD_SW_IRQ);
//    GPIO_PinInit(BOARD_SW_GPIO, BOARD_SW_GPIO_PIN, &sw_config);

    NVIC_SetPriority(DEMO_LPUART_IRQn, 5);
    //====================================================================
    //
    //====================================================================
    if (xTaskCreate(uart_task, "Uart_task", configUartAppTask_STACK_SIZE, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("uart Task creation failed!.\r\n");
        while (1)
            ;
    }
    //====================================================================
    //
    //====================================================================
    if (xTaskCreate(sw_task, "sw_task", configSwAppTask_STACK_SIZE, NULL, sw_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("sw Task creation failed!.\r\n");
        while (1)
            ;
    }
    //====================================================================
    // Initial UWB
    //====================================================================
    Init_UWB_Range();

    //====================================================================
    //
    //====================================================================
    vTaskStartScheduler();

    for (;;)
        ;

}

/*!
 * @brief Task responsible for loopback.
 */
static void uart_task(void *pvParameters)
{
    int error;
    size_t n = 0;

    lpuart_config.srcclk = DEMO_LPUART_CLK_FREQ;
    lpuart_config.base   = DEMO_LPUART;

    if (0 > LPUART_RTOS_Init(&handle, &t_handle, &lpuart_config))
    {
        vTaskSuspend(NULL);
    }

    /* Send introduction message. */
    if (0 > LPUART_RTOS_Send(&handle, (uint8_t *)to_send, strlen(to_send)))
    {
        vTaskSuspend(NULL);
    }

    /* Receive user input and send it back to terminal. */
    do
    {
        //========================================================================
        //
        //========================================================================
   		error = LPUART_RTOS_Receive(&handle, uart_recv_buffer, 1, &n);

        //========================================================================
        //
        //========================================================================
        if (error == kStatus_LPUART_RxHardwareOverrun)
        {

            /* Notify about hardware buffer overrun */
            if (kStatus_Success !=
                LPUART_RTOS_Send(&handle, (uint8_t *)send_hardware_overrun, strlen(send_hardware_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        //========================================================================
        //
        //========================================================================
        if (error == kStatus_LPUART_RxRingBufferOverrun)
        {
            /* Notify about ring buffer overrun */
            if (kStatus_Success != LPUART_RTOS_Send(&handle, (uint8_t *)send_ring_overrun, strlen(send_ring_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        //========================================================================
        //
        //========================================================================
		if (uart_eof==false)
		{
			//--------------------------------------------------------------------
			// [ start of flag ]
			//--------------------------------------------------------------------
			if (uart_sof==true)
			{
				//----------------------------------------------------------------
				// [ DATA ]
				//----------------------------------------------------------------
				uart_recvstr[uart_recv_cnt]=uart_recv_buffer[0];
				uart_recv_cnt++;
				uart_recv_cs += uart_recv_buffer[0];
				//----------------------------------------------------------------
				// [ Length ]
				//----------------------------------------------------------------
				if (uart_recv_cnt==2)
				{
					uart_recv_maxcnt += uart_recv_buffer[0];
					if (uart_recv_maxcnt > 20){
						uart_recv_maxcnt = 20;
					}
				}
				//----------------------------------------------------------------
				// [ EOF ]
				//----------------------------------------------------------------
				if (uart_recv_cnt>=uart_recv_maxcnt)
				{
					uart_sof = false;
					uart_eof = true;
				}
			}
			else if (uart_recv_buffer[0]=='$'){							//Start!
				uart_sof = true;
				uart_recv_cnt = 0;
				uart_recv_cs = 0;
				uart_recv_maxcnt = 4;									//ST code+Cmd+Length+End code
			}
		}

    } while (kStatus_Success == error);

    LPUART_RTOS_Deinit(&handle);
    vTaskSuspend(NULL);
}

static void sw_task(void *pvParameters)
{
    do
    {
//    	if(g_ButtonPress){
//    	    GPIO_PortToggle(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_PIN);
//    	    if (RKE_ARM == 0x01)
//    	    	RKE_ARM = 0x02;
//    	    else
//    	    	RKE_ARM = 0x01;
//    	    RFIC_cmd_compose_send(oRKE_command);
//    	    g_ButtonPress = false;
//    	}

    	if(uart_eof==true)
    	{
    		RFIC_cmd_from_RFIC(uart_recvstr);
    		uart_eof = false;
    	}

    	if(fob_sys_rf_poweron_step==0)
    	{
            RFIC_cmd_compose_send(oSW_Reset_RFIC);
            fob_sys_rf_poweron_step = __FIT_RFIC_PWRON_START;			// avoid continuous send reset command
    	}

    } while (1);

    vTaskSuspend(NULL);
}

/*
 * RFIC handler
 */

static void RFIC_cmd_ack_nak(uint8_t ok){

	uint8_t rtn[6];
	APP_DBG_LOG("ACKing...\r\n");
	rtn[0]=0x24;
	rtn[1]=0x00;
	rtn[2]=0x01;
	rtn[3]=ok?0x00:0x01;
	rtn[4]=ok?0x01:0x02;
	rtn[5]=0x0d;

	LPUART_RTOS_Send(&handle, (uint8_t *)rtn, 6);

}

static uint8_t RFIC_cmd_to_RFIC(uint8_t* pkt, uint8_t pktlen){

	LPUART_RTOS_Send(&handle, (uint8_t *)pkt, pktlen);
    return 0;
}

static void wait_for_reply(uint8_t operate){

	while(waitfor!=0){
		APP_DBG_LOG("resend...\r\n");
		vTaskDelay(100);
		RFIC_cmd_to_RFIC(strdone, RFIC_pkt_len[operate]+2);
		resendcnt++;

		if (resendcnt==4){
			waitfor=0;
			resendcnt=0;
			memset(strdone, 0x00, RFIC_pkt_len[operate]+2);
			break;
		}
	}
}

void RFIC_set_power(uint8_t lvl){

	//set TX power for packet composing later
	RFIC_TX_power = lvl;
}

void RFIC_set_int_time(uint16_t int433, uint16_t int125){

	//set 433MHz/125K Interval time
	RFIC_433_Int_time=int433;
	RFIC_125_Int_time=int125;
}

static void RFIC_cmd_from_RFIC(uint8_t *fromstr){

	uint8_t		uart_cmd = 0;
	uint8_t		uart_data[16] = {0};
	uint8_t		uwb_ursk[16] = {0};
	uint8_t		uwb_result = 0;

	int reporttype=0;
	uint8_t check=0;
	uint8_t battFOB=0;
	uint8_t auth=0;

	uart_cmd = *(fromstr+__FIT_UART_CMD_H);
	memcpy((uint8_t*)uart_data, (uint8_t*)(fromstr+__FIT_UART_DATA_START_H), __FIT_UART_DATA_MAX_SIZE_H);
	uart_eof=false;

	switch(uart_cmd)
	{
	case oACK_NAK:
		if(*(fromstr+3)==0x00)
		{
			APP_DBG_LOG("ACK received\r\n");
			if(waitfor != oRead_RFIC_Info)
			{
			    waitfor=0;
			    memset(strdone, 0x00, RFIC_pkt_len[waitfor]+2);
			}

		} else {
			APP_DBG_LOG("NNN NAK received\r\n");

		}

        break;
	case oPowerOn_Info_Rpt:
		APP_DBG_LOG("the 433m int time %x%x\r\n", *(fromstr+4), *(fromstr+5));
		APP_DBG_LOG("the 125k int time %x%x\r\n", *(fromstr+6), *(fromstr+7));
		APP_DBG_LOG("the RFIC ver %x\r\n", *(fromstr+8));
		RFIC_cmd_ack_nak(1);
		fob_sys_rf_poweron_step = __FIT_RFIC_PWRON_COMPLETE;			// complete power on
		break;
	case oSet_RFIC_Interval_Time:
		APP_DBG_LOG("the 433m status %x\r\n", *(fromstr+3));
		APP_DBG_LOG("the 125k status %x\r\n", *(fromstr+4));
		RFIC_cmd_ack_nak(1);
		break;

	case oRFIC_Rpt_from_RFIC:

		if(waitfor==oRead_RFIC_Info){
			waitfor=0;
			memset(strdone, 0x00, RFIC_pkt_len[waitfor]+2);
		}
		reporttype = *(fromstr+3);
		switch(reporttype){
		    case 0x02:
		    APP_DBG_LOG("the 433m and 125k status %x %x\r\n", *(fromstr+4), *(fromstr+5));
		    break;
		    case 0x03:
			APP_DBG_LOG("the role %x 0:fob 1:veh\r\n", *(fromstr+4));
			break;
		    case 0x04:
			APP_DBG_LOG("the version %x \r\n", *(fromstr+4));
			break;
	        case 0x05:
			APP_DBG_LOG("the TX power\r\n", *(fromstr+4));
		    break;
	        case 0x06:
			APP_DBG_LOG("the pairing info %x\r\n", *(fromstr+4));
			break;
		    case 0x07:
			APP_DBG_LOG("the interval time 433m %x%x \r\n", *(fromstr+4), *(fromstr+5));
			APP_DBG_LOG("the interval time 125k %x%x \r\n", *(fromstr+6), *(fromstr+7));
			break;
		    default:
			for (int i=0;i<10 ;i++)
			{
				APP_DBG_LOG("the raw is%x ", *(fromstr+4+i));
			}
			APP_DBG_LOG("\r\n");
		}
		RFIC_cmd_ack_nak(1);
		break;

	case oRKE_command:
		if(*(fromstr+3)==0x01)
			APP_DBG_LOG("direction: RFIC -> MCU\r\n");
		else
			APP_DBG_LOG("direction: weird \r\n");

		if(*(fromstr+4)==0x03)
			APP_DBG_LOG("cmd: find me\r\n");
		else
			APP_DBG_LOG("cmd: weird\r\n");
	    break;

	case oRF_Comm_Auth_Result:
	{
//		auth =
//		auth = *(fromstr+3) & 0x01;
//		battFOB = (*(fromstr+3) >> 1) & 0x01;
//		APP_DBG_LOG("the info of RF auth result %x \r\n", auth);
//		APP_DBG_LOG("the info of FOB batt %x \r\n", battFOB);
		RFIC_cmd_ack_nak(1);

		uwb_result = UWB_Api_Start_Ranging(uwb_ursk, 0x0001);
		__asm("nop");
	} break;

	default:
		RFIC_cmd_ack_nak(0);
	}
}

static uint8_t RFIC_cmd_compose_send(uint8_t RFIC_operate){

	if(waitfor == 0){
	switch(RFIC_operate){

	case oSW_Reset_RFIC:
		APP_DBG_LOG("SW reset RFIC\r\n");

		strdone[0]=0x24;
		strdone[1]=0x01;
		strdone[2]=0x01;
		strdone[3]=0x00;
		strdone[4]=0x02;
		strdone[5]=0x0d;
		RFIC_cmd_to_RFIC(strdone, RFIC_pkt_len[oSW_Reset_RFIC]+2);
        waitfor =oSW_Reset_RFIC;

        break;
	case oSet_TX_Power:
		APP_DBG_LOG("Set TX power\r\n");


		strdone[0]=0x24;
		strdone[1]=0x03;
		strdone[2]=0x01;
		strdone[3]=RFIC_TX_power;
		strdone[4]=0x04+RFIC_TX_power;
		strdone[5]=0x0d;
		RFIC_cmd_to_RFIC(strdone, RFIC_pkt_len[oSet_TX_Power]+2);
		waitfor =oSet_TX_Power;

		break;
//	case oStartStop_RFIC:
//		APP_DBG_LOG("Start RF\r\n");
//
//		strdone[0]=0x24;
//		strdone[1]=0x04;
//		strdone[2]=0x05;
//		strdone[3]=0x00;
//		strdone[4]=(RFIC_433_Int_time>>8) & 0xff;
//		strdone[5]=RFIC_125_Int_time & 0xff;
//		strdone[6]=(RFIC_125_Int_time>>8) & 0xff;
//		strdone[7]=RFIC_125_Int_time & 0xff;
//		strdone[8]=0x09+strdone[4]+strdone[5]+strdone[6]+strdone[7];
//		strdone[9]=0x0d;
//		RFIC_cmd_to_RFIC(strdone, RFIC_pkt_len[oStart_RFIC]+2);
//		waitfor =oStart_RFIC;
//
//        break;

//	case oSet_RFIC_Interval_Time:
//	{
//		APP_DBG_LOG("Set RFIC interval time\r\n");
//
//		strdone[0]=0x24;
//		strdone[1]=0x05;
//		strdone[2]=0x04;
//		strdone[3]=(RFIC_433_Int_time>>8) & 0xff;
//		strdone[4]=RFIC_125_Int_time & 0xff;
//		strdone[5]=(RFIC_125_Int_time>>8) & 0xff;
//		strdone[6]=RFIC_125_Int_time & 0xff;
//		strdone[7]=0x09+strdone[3]+strdone[4]+strdone[5]+strdone[6];
//		strdone[8]=0x0d;
//		RFIC_cmd_to_RFIC(strdone, RFIC_pkt_len[oSet_RFIC_Int_Time]+2);
//	    waitfor =oSet_RFIC_Int_Time;
//
//	} break;
	case oRead_RFIC_Info:
	{
		APP_DBG_LOG("Read RFIC info\r\n");

		strdone[0]=0x24;
		strdone[1]=0x07;
	    strdone[2]=0x01;
		strdone[3]=0x01;
		strdone[4]=0x09;
		strdone[5]=0x0d;
		RFIC_cmd_to_RFIC(strdone, RFIC_pkt_len[oRead_RFIC_Info]+2);
	    waitfor =oRead_RFIC_Info;

	} break;
	case oInto_Pairing:
	{
		APP_DBG_LOG("Into Pairing mode\r\n");

		strdone[0]=0x24;
		strdone[1]=0x30;
		strdone[2]=0x01;
		strdone[3]=0x01;
		strdone[4]=0x32;
		strdone[5]=0x0d;
		RFIC_cmd_to_RFIC(strdone, RFIC_pkt_len[oInto_Pairing]+2);
		waitfor =oInto_Pairing;
	} break;
	case oRKE_command:
	{
		APP_DBG_LOG("RKE cmd \r\n");
		strdone[0]=0x24;
		strdone[1]=0x20;
		strdone[2]=0x02;
		strdone[3]=0x00;
		strdone[4]=RKE_ARM;
		strdone[5]=0x22+RKE_ARM;
		strdone[6]=0x0d;
		RFIC_cmd_to_RFIC(strdone, RFIC_pkt_len[oRKE_command]+2);
		waitfor =oRKE_command;
	} break;
	default:
		RFIC_cmd_ack_nak(0);
		break;

	}
	}
	if(waitfor!=0)
		wait_for_reply(waitfor);

	return 0;
}
