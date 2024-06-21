/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */


/************************************************************************************
*
*       INCLUDES
*
************************************************************************************/
#include <stdint.h>
#include "message_queue.h"
#include "phscaR4CadsTypesIntf.h"

//***********************************************************************************
//
//***********************************************************************************
void MessageQueue_Init(MessageQueue *q) {
    q->front = 0;
    q->rear = 0;
}

int MessageQueue_IsFull(MessageQueue *q) {
    return ((q->rear + 1) % QUEUE_SIZE) == q->front;
}

int MessageQueue_IsEmpty(MessageQueue *q) {
    return q->front == q->rear;
}

void MessageQueue_Put(MessageQueue *q, phscaR4CadsTypesIntf_UciFrame_t item)
{
    if (!MessageQueue_IsFull(q)) {
        q->queue[q->rear] = item;
        q->rear = (q->rear + 1) % QUEUE_SIZE;
    } else {
        // 隊列已滿，處理錯誤
    }
}

phscaR4CadsTypesIntf_UciFrame_t* MessageQueue_Get(MessageQueue *q)
{
    if (!MessageQueue_IsEmpty(q)) {
        phscaR4CadsTypesIntf_UciFrame_t* item = &(q->queue[q->front]);
        q->front = (q->front + 1) % QUEUE_SIZE;
        return item;
    } else {
        // 隊列為空，處理錯誤
        return 0;
    }
}

void MessageQueue_PutHead(MessageQueue *q, phscaR4CadsTypesIntf_UciFrame_t item) {
    if (!MessageQueue_IsFull(q)) {
        q->front = (q->front - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        q->queue[q->front] = item;
    } else {
        // 隊列已滿，處理錯誤
    }
}

int MessageQueue_Count(MessageQueue *q) {
    return (q->rear - q->front + QUEUE_SIZE) % QUEUE_SIZE;
}


