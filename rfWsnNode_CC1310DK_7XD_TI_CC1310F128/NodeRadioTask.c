/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
#include "NodeRadioTask.h"

#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

#include <stdlib.h>
#include <driverlib/trng.h>
#include "easylink/EasyLink.h"
#include "RadioProtocol.h"


/***** Defines *****/
#define NODERADIO_TASK_STACK_SIZE 1024
#define NODERADIO_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                 0xFFFFFFFF
#define RADIO_EVENT_SEND_ADC_DATA       (uint32_t)(1 << 0)
#define RADIO_EVENT_DATA_ACK_RECEIVED   (uint32_t)(1 << 1)
#define RADIO_EVENT_ACK_TIMEOUT         (uint32_t)(1 << 2)
#define RADIO_EVENT_SEND_FAIL           (uint32_t)(1 << 3)

#define NODERADIO_MAX_RETRIES 2
#define NORERADIO_ACK_TIMEOUT_TIME_MS (160)


/***** Type declarations *****/
struct RadioOperation {
    EasyLink_TxPacket easyLinkTxPacket;
    uint8_t retriesDone;
    uint8_t maxNumberOfRetries;
    uint32_t ackTimeoutMs;
    enum NodeRadioOperationStatus result;
};


/***** Variable declarations *****/
static Task_Params nodeRadioTaskParams;
Task_Struct nodeRadioTask;        /* not static so you can see in ROV */
static uint8_t nodeRadioTaskStack[NODERADIO_TASK_STACK_SIZE];
Semaphore_Struct radioAccessSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioAccessSemHandle;
Event_Struct radioOperationEvent; /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;
Semaphore_Struct radioResultSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioResultSemHandle;
Clock_Struct ackTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle ackTimeoutClockHandle;
static struct RadioOperation currentRadioOperation;
static uint16_t adcData;
static uint8_t nodeAddress;
static struct AdcSensorPacket adcSensorPacket;


/***** Prototypes *****/
static void nodeRadioTaskFunction(UArg arg0, UArg arg1);
static void returnRadioOperationStatus(enum NodeRadioOperationStatus status);
static void sendAdcPacket(uint16_t adcValue, uint8_t maxRetries, uint32_t ackTimeoutMs);
static void resendPacket();
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
static void ackTimeoutCallback(UArg arg0);


/***** Function definitions *****/
void NodeRadioTask_init(void) {

    /* Create semaphore used for exclusive radio access */
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&radioAccessSem, 1, &semParam);
    radioAccessSemHandle = Semaphore_handle(&radioAccessSem);

    /* Create semaphore used for callers to wait for result */
    Semaphore_construct(&radioResultSem, 0, &semParam);
    radioResultSemHandle = Semaphore_handle(&radioResultSem);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Create clock object which is used for ackknowledgement timeout and retries */
    Clock_Params clkParams;
    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&ackTimeoutClock, ackTimeoutCallback, 1, &clkParams);
    ackTimeoutClockHandle = Clock_handle(&ackTimeoutClock);

    /* Create the radio protocol task */
    Task_Params_init(&nodeRadioTaskParams);
    nodeRadioTaskParams.stackSize = NODERADIO_TASK_STACK_SIZE;
    nodeRadioTaskParams.priority = NODERADIO_TASK_PRIORITY;
    nodeRadioTaskParams.stack = &nodeRadioTaskStack;
    Task_construct(&nodeRadioTask, nodeRadioTaskFunction, &nodeRadioTaskParams, NULL);
}

static void nodeRadioTaskFunction(UArg arg0, UArg arg1)
{
    /* Initialize EasyLink */
    if(EasyLink_init(RADIO_EASYLINK_MODULATION) != EasyLink_Status_Success) {
        System_abort("EasyLink_init failed");
    }

    /* Set frequency */
    if(EasyLink_setFrequency(RADIO_FREQUENCY) != EasyLink_Status_Success) {
        System_abort("EasyLink_setFrequency failed");
    }

    /* Use the True Random Number Generator to generate sensor node address randomly */;
    Power_setDependency(PowerCC26XX_PERIPH_TRNG);
    TRNGEnable();
    /* Do not accept the same address as the concentrator, in that case get a new random value */
    do {
        while(!(TRNGStatusGet() & TRNG_NUMBER_READY));
        nodeAddress = (uint8_t)TRNGNumberGet(TRNG_LOW_WORD);
    } while(nodeAddress == RADIO_CONCENTRATOR_ADDRESS);
    TRNGDisable();
    Power_releaseDependency(PowerCC26XX_PERIPH_TRNG);

    /* Set the filter to the generated random address */
    if(EasyLink_enableRxAddrFilter(&nodeAddress, 1, 1) != EasyLink_Status_Success) {
        System_abort("EasyLink_enableRxAddrFilter failed");
    }

    /* Setup ADC sensor packet */
    adcSensorPacket.header.sourceAddress = nodeAddress;
    adcSensorPacket.header.packetType = RADIO_PACKET_TYPE_ADC_SENSOR_PACKET;

    /* Enter main task loop */
    while (1) {
        /* Wait for an event */
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If we should send ADC data */
        if(events & RADIO_EVENT_SEND_ADC_DATA) {
            sendAdcPacket(adcData, NODERADIO_MAX_RETRIES, NORERADIO_ACK_TIMEOUT_TIME_MS);
        }

        /* If we get an ACK from the concentrator */
        if(events & RADIO_EVENT_DATA_ACK_RECEIVED) {
            returnRadioOperationStatus(NodeRadioStatus_Success);
        }

        /* If we get an ACK timeout */
        if(events & RADIO_EVENT_ACK_TIMEOUT) {

            /* If we haven't resent it the maximum number of times yet, then resend packet */
            if(currentRadioOperation.retriesDone < currentRadioOperation.maxNumberOfRetries) {
                resendPacket();
            }
            else {
                /* Else return send fail */
                Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_FAIL);
            }
        }

        /* If send fail */
        if(events & RADIO_EVENT_SEND_FAIL) {
            returnRadioOperationStatus(NodeRadioStatus_Failed);
        }

    }
}

enum NodeRadioOperationStatus NodeRadioTask_sendAdcData(uint16_t data)
{
    enum NodeRadioOperationStatus status;

    /* Get radio access sempahore */
    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);

    /* Save data to send */
    adcData = data;

    /* Raise RADIO_EVENT_SEND_ADC_DATA event */
    Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_ADC_DATA);

    /* Wait for result */
    Semaphore_pend(radioResultSemHandle, BIOS_WAIT_FOREVER);

    /* Get result */
    status = currentRadioOperation.result;

    /* Return radio access semaphore */
    Semaphore_post(radioAccessSemHandle);

    return status;
}

static void returnRadioOperationStatus(enum NodeRadioOperationStatus result)
{
    /* Abort to exit RX */
	if(EasyLink_abort() != EasyLink_Status_Success) {
        System_abort("EasyLink_abort failed");
    }

    /* Save result */
    currentRadioOperation.result = result;

    /* Post result semaphore */
    Semaphore_post(radioResultSemHandle);
}

static void sendAdcPacket(uint16_t adcValue, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs)
{
    /* Insert ADC value */
    adcSensorPacket.adcValue = adcValue;

    /* Set destination address in EasyLink API */
    currentRadioOperation.easyLinkTxPacket.dstAddr[0] = RADIO_CONCENTRATOR_ADDRESS;

    /* Copy ADC packet to payload
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    memcpy(currentRadioOperation.easyLinkTxPacket.payload, ((uint8_t*)&adcSensorPacket), sizeof(struct AdcSensorPacket));
    currentRadioOperation.easyLinkTxPacket.len = sizeof(struct AdcSensorPacket);

    /* Setup retries */
    currentRadioOperation.maxNumberOfRetries = maxNumberOfRetries;
    currentRadioOperation.ackTimeoutMs = ackTimeoutMs;
    currentRadioOperation.retriesDone = 0;
    Clock_setTimeout(ackTimeoutClockHandle, currentRadioOperation.ackTimeoutMs * 1000 / Clock_tickPeriod);

    /* Send packet  */
    if(EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success) {
        System_abort("EasyLink_transmit failed");
    }

    /* Enter RX */
    if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
        System_abort("EasyLink_receiveAsync failed");
    }

    /* Start the ACK timeout timer */
    Clock_start(ackTimeoutClockHandle);
}

static void resendPacket()
{
    /* Abort to leave RX */
	if(EasyLink_abort() != EasyLink_Status_Success) {
        System_abort("EasyLink_abort failed");
    }

    /* Send packet  */
    if(EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success) {
        System_abort("EasyLink_transmit failed");
    }

    /* Enter RX and wait for ACK with timeout */
    if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
        System_abort("EasyLink_receiveAsync failed");
    }

    /* Reset the ACK timeout timer */
    Clock_start(ackTimeoutClockHandle);

    /* Increase retries by one */
    currentRadioOperation.retriesDone++;
}

static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    struct PacketHeader* packetHeader;

    /* If this callback is called because of a packet received */
    if (status == EasyLink_Status_Success)
    {
        /* Check the payload header */
        packetHeader = (struct PacketHeader*)rxPacket->payload;

        /* Check if this is an ACK packet */
        if(packetHeader->packetType == RADIO_PACKET_TYPE_ACK_PACKET) {

            /* Stop ACK timeout */
            Clock_stop(ackTimeoutClockHandle);

            /* Signal ACK packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_DATA_ACK_RECEIVED);
        }
    }
}

static void ackTimeoutCallback(UArg arg0)
{
    /* Post a RADIO_EVENT_ACK_TIMEOUT event */
    Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
}
