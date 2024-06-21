/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

/************************************************************************************
*
*       INCLUDES
*
************************************************************************************/

#include <stdint.h>
#include "phscaR4CadsTypesIntf.h"


/************************************************************************************
*
*       TYPE DEFINITIONS
*
************************************************************************************/
#define QUEUE_SIZE 10

typedef struct {
	phscaR4CadsTypesIntf_UciFrame_t queue[QUEUE_SIZE];
    int front;
    int rear;
} MessageQueue;

//***********************************************************************************
//
//***********************************************************************************
void MessageQueue_Init(MessageQueue *q);
int MessageQueue_IsFull(MessageQueue *q);
int MessageQueue_IsEmpty(MessageQueue *q);
void MessageQueue_Put(MessageQueue *q, phscaR4CadsTypesIntf_UciFrame_t item);
phscaR4CadsTypesIntf_UciFrame_t* MessageQueue_Get(MessageQueue *q);
void MessageQueue_PutHead(MessageQueue *q, phscaR4CadsTypesIntf_UciFrame_t item);
int MessageQueue_Count(MessageQueue *q);

#endif /* MESSAGE_QUEUE_H */
