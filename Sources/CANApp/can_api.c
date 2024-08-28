/*************************************************************************//**
 * @file
 * @brief       AFBR-S50 CAN Interface
 * @details     This file defines an CAN interface to communicate with the
 *              AFBR-S50 Time-Of-Flight sensor API.
 *
 * @copyright
 *
 * Copyright (c) 2022, Broadcom Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/*!***************************************************************************
 * @addtogroup  can_intf
 * @{
 *****************************************************************************/

#include "can_api.h"

#include "main.h"
#include "debug.h"

#include "r_can.h"
#include "hal_data.h"
#include "driver/bsp.h"
#include "driver/irq.h"
#include <stdio.h>
#include <canard.h>
#include <linux.h>
#include <dronecan_msgs.h>
#include <stdlib.h>
#include <string.h>

//#include "r_flash_hp.h"

static CanardInstance canard;
static uint8_t memory_pool[1024];

/*! CAN transmission busy status. */
static volatile bool is_can_tx_busy = false;

/*! CAN remote frame if received but not handled. */
static volatile uint32_t can_rx_remote_id = 0;


/*! The used mail box number */
#define CAN_MAILBOX_ID_TRANSMIT CAN_MAILBOX_ID_0

/*! Data length for TX frames. */
#define CAN_FRAME_TRANSMIT_DATA_BYTES   (8U)

static struct
{
    float can_node;
    float sensor_index;
    float telem_rate;
} settings;

/*
  data for dynamic node allocation process
 */
static struct {
    uint32_t send_next_node_id_allocation_request_at_ms;
    uint32_t node_id_allocation_unique_id_offset;
} DNA;


#define UAVCAN_MESSAGE_MAX_SIZE 8 // Replace with the actual size of your message
#define UAVCAN_MESSAGE_SIGNATURE 0xabcdefabcdefabcdULL // Replace with the actual message signature
#define UAVCAN_MESSAGE_ID 1234 // Replace with the actual message ID


/*
  in this example we will use dynamic node allocation if MY_NODE_ID is zero
 */
#define MY_NODE_ID 0

/*
  our preferred node ID if nobody else has it
 */
#define PREFERRED_NODE_ID 73

#define MANUFACTURER_NAME "Example Lidar Co."



static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer);
static void handle_param_GetSet(CanardInstance* ins, CanardRxTransfer* transfer);
static void handle_param_ExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer);
static void handle_RestartNode(CanardInstance* ins, CanardRxTransfer* transfer);
static void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer);
static void CAN_StartTaskManager(void);
static void process1HzTasks(uint64_t timestamp_usec);
static void request_DNA(void);



/*! CAN Data Frame ID definition. */
typedef enum argus_can_frame_id_t
{
    /*! Remote Frame ID for starting measurements */
    CAN_FRAME_ID_START = 8,

    /*! Remote Frame ID for stopping measurements */
    CAN_FRAME_ID_STOP = 9,

    /*! 1D Data Frame ID */
    CAN_FRAME_ID_1D = 28,

} argus_can_frame_id_t;

static struct parameter {
    char *name;
    enum uavcan_protocol_param_Value_type_t type;
    float *value;
    float min_value;
    float max_value;
} parameters[] = {
    // add any parameters you want users to be able to set
    { "CAN_NODE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.can_node, 0, 127 }, // CAN node ID
    { "SENSOR_INDEX", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.sensor_index, 0, 32 }, // index in RawCommand
    { "TELEM_RATE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.telem_rate, 0, 32 }, // index in RawCommand
};

// some convenience macros
#define MIN(a,b) ((a)<(b)?(a):(b))
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

/*
  hold our node status as a static variable. It will be updated on any errors
 */
static struct uavcan_protocol_NodeStatus node_status;
static uint64_t next_1hz_service_at = 0;
static uint64_t last_telem_us = 0;

static uint64_t micros64(void)
{
    return (uint64_t) Time_GetNowUSec();
}
/*
  get monotonic time in milliseconds since startup
 */
static uint32_t millis32(void)
{
    return Time_GetNowMSec();
}


static void getUniqueID(uint8_t id[16])
{
    memset(id, 0, 16);
    const bsp_unique_id_t *unique_id = R_BSP_UniqueIdGet();
    if (unique_id) {
        memcpy(id, unique_id->unique_id_bytes, 16);
    }
}


static void save_settings(void)
{
    fsp_err_t err;
    uint32_t flash_address = 0x00100000;  // Flash address where settings will be stored (example address)
    uint32_t num_bytes = sizeof(settings); // Number of bytes to write

    // Open the flash device
    err = R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
    if (FSP_SUCCESS != err) {
        print("Failed to open flash: %d\n", err);
        return;
    }

    // Erase the flash block before writing
    err = R_FLASH_HP_Erase(&g_flash0_ctrl, flash_address, 1);
    if (FSP_SUCCESS != err) {
        print("Failed to erase flash: %d\n", err);
        R_FLASH_HP_Close(&g_flash0_ctrl);
        return;
    }

    // Write the settings to flash
    err = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t)&settings, flash_address, num_bytes);
    if (FSP_SUCCESS != err) {
        print("Failed to write flash: %d\n", err);
        R_FLASH_HP_Close(&g_flash0_ctrl);
        return;
    }

    // Close the flash device
    err = R_FLASH_HP_Close(&g_flash0_ctrl);
    if (FSP_SUCCESS != err) {
        print("Failed to close flash: %d\n", err);
    }
}

static void load_settings(void)
{
    uint32_t flash_address = 0x40100000;  // Example base address of the data flash

    // Open the flash device (not strictly necessary for reading in this case)
    fsp_err_t err = R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);
    if (FSP_SUCCESS != err) {
        print("Failed to open flash: %d\n", err);
        return;
    }

    // Read the settings from flash
    memcpy(&settings, (void *)flash_address, sizeof(settings));

    // Close the flash device (optional, depending on your implementation)
    err = R_FLASH_HP_Close(&g_flash0_ctrl);
    if (FSP_SUCCESS != err) {
        print("Failed to close flash: %d\n", err);
    }
}


static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
   // switch on data type ID to pass to the right handler function
   if (transfer->transfer_type == CanardTransferTypeRequest) {
       // check if we want to handle a specific service request
       switch (transfer->data_type_id) {
       case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
           handle_GetNodeInfo(ins, transfer);
           break;
       }
       case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
           handle_param_GetSet(ins, transfer);
           break;
       }
       case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
           handle_param_ExecuteOpcode(ins, transfer);
           break;
       }
       case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
           handle_RestartNode(ins, transfer);
           break;
       }
       }
   }
   if (transfer->transfer_type == CanardTransferTypeBroadcast) {
       // check if we want to handle a specific broadcast message
       switch (transfer->data_type_id) {
       case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
           handle_DNA_Allocation(ins, transfer);
           break;
       }
       }
   }
}


/*
This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
by the local node.
If the callback returns true, the library will receive the transfer.
If the callback returns false, the library will ignore the transfer.
All transfers that are addressed to other nodes are always ignored.

This function must fill in the out_data_type_signature to be the signature of the message.
*/
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                uint64_t *out_data_type_signature,
                                uint16_t data_type_id,
                                CanardTransferType transfer_type,
                                uint8_t source_node_id)
{
   if (transfer_type == CanardTransferTypeRequest) {
       // check if we want to handle a specific service request
       switch (data_type_id) {
       case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
           *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
           return true;
       }
       case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
           *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
           return true;
       }
       case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
           *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
           return true;
       }
       case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
           *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
           return true;
       }
       }
   }
   if (transfer_type == CanardTransferTypeBroadcast) {
       // see if we want to handle a specific broadcast packet
       switch (data_type_id) {
       case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
           *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
           return true;
       }
       }
   }
   // we don't want any other messages
   return false;
}

static void canardInitInstance(void)
{
    // Initialize the memory pool for Libcanard
//    static uint8_t memory_pool[1024];  // Size depends on your requirements

    // Initialize the CanardInstance structure
    canardInit(&canard, memory_pool, sizeof(memory_pool), onTransferReceived, shouldAcceptTransfer, NULL);

    // Set the node ID (use 0 for dynamic node allocation)
    canardSetLocalNodeID(&canard, MY_NODE_ID);

    // Optionally set other instance parameters if needed
    // (e.g., transfer interval, max transfer size, etc.)
}

void CAN_Init(void)
{
    fsp_err_t err = R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
    if (FSP_SUCCESS != err)
    {
        print("CAN Open API failed with error code: %d\n", err);
        handle_error(ERROR_FAIL, "CAN Open failed.");
    }

    // Initialize CanardInstance
    canardInitInstance();

    // Initialize timing variables
    next_1hz_service_at = micros64();
    last_telem_us = micros64();

    // Start task manager
    CAN_StartTaskManager();
}



void CAN_StartTaskManager(void)
{
    while (1)
    {
        // Process 1Hz tasks and send NodeStatus
        uint64_t ts = micros64();
        if (ts >= next_1hz_service_at)
        {
            next_1hz_service_at += 1000000ULL;
            process1HzTasks(ts);
        }

        // Request DNA if needed
        if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID && millis32() > DNA.send_next_node_id_allocation_request_at_ms)
        {
            request_DNA();
        }

        // Additional tasks can be added here as needed
        // For example: CAN_HandleCommand();
    }
}




void CAN_Deinit(void)
{
    fsp_err_t err = R_CAN_Close(&g_can0_ctrl);
    if (FSP_SUCCESS != err)
    {
        print("CAN Close API failed with error code: %d\n", err);
        handle_error(ERROR_FAIL, "CAN Close failed.");
    }
}

static void CAN_AwaitIdle(void)
{
    const uint32_t timeout_ms = 500;
    ltc_t start;
    Time_GetNow(&start);

    /* Wait until no transfer is ongoing and claim the control. */
    for (;;)
    {
        while (is_can_tx_busy)
        {
            if (Time_CheckTimeoutMSec(&start, timeout_ms))
            {
                handle_error(ERROR_TIMEOUT, "Can Write has yielded a timeout!");
            }
        }

        /* make sure that no IRQ has happened meanwhile... */
        IRQ_LOCK();
        if (!is_can_tx_busy)
        {
            is_can_tx_busy = true;
            IRQ_UNLOCK();
            break;
        }
        IRQ_UNLOCK();
    }
}



/*
  handle parameter GetSet request
 */
static void handle_param_GetSet(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    struct parameter *p = NULL;
    if (req.name.len != 0) {
        for (uint16_t i=0; i<ARRAY_SIZE(parameters); i++) {
            if (req.name.len == strlen(parameters[i].name) &&
                strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0) {
                p = &parameters[i];
                break;
            }
        }
    } else if (req.index < ARRAY_SIZE(parameters)) {
        p = &parameters[req.index];
    }
    if (p != NULL && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        /*
          this is a parameter set command. The implementation can
          either choose to store the value in a persistent manner
          immediately or can instead store it in memory and save to permanent storage on a
         */
        switch (p->type) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            *p->value = (float) req.value.integer_value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE:
            *p->value = (float) req.value.boolean_value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            *p->value = req.value.real_value;
            break;
        default:
            return;
        }
        save_settings();
    }

    /*
      for both set and get we reply with the current value
     */
    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    if (p != NULL) {
        pkt.value.union_tag = p->type;
        switch (p->type) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            pkt.value.integer_value = (int64_t) *p->value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE:
            pkt.value.integer_value = (int64_t) *p->value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            pkt.value.real_value = *p->value;
            break;
        default:
            return;
        }
        pkt.name.len = (uint8_t) strlen(p->name);
        strcpy((char *)pkt.name.data, p->name);
    }

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = (uint16_t) uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);


    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}


/*
  handle parameter executeopcode request
 */
static void handle_param_ExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        // here is where you would reset all parameters to defaults
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        // here is where you would save all the changed parameters to permanent storage
    }

    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}


void CAN_Write(can_frame_t * const tx_frame)
{
    assert(tx_frame != NULL);

    // Set the transmission busy flag
    is_can_tx_busy = true;

    // Send frame via CAN
    fsp_err_t err = R_CAN_Write(&g_can0_ctrl, CAN_MAILBOX_ID_TRANSMIT, tx_frame);

    if (FSP_SUCCESS != err)
    {
        // Handle the error, maybe retry or log it
        print("CAN Write API failed with error code: %d\n", err);
        // Clear the busy flag as the transmission failed
        is_can_tx_busy = false;
        // Optionally, handle error (like resetting the CAN interface)
    }

    // Clear the transmission busy flag in the CAN callback (once transmission completes)
}

/*
  handle RestartNode request
 */
static void handle_RestartNode(CanardInstance* ins, CanardRxTransfer* transfer)
{

    printf("Rebooting!!!\n");
    exit(0);
}

void CAN_Transmit1D(argus_results_t const * res)
{
    assert(res != NULL);
    CAN_AwaitIdle();
    uint8_t uavcan_payload[UAVCAN_MESSAGE_MAX_SIZE] = {0};

    int32_t range_mm = res->Bin.Range / (Q9_22_ONE / 1000);
    if (range_mm < 0) range_mm = 0;
    else if (range_mm > 0xFFFFFF) range_mm = 0xFFFFFF;

    uint16_t amplitude = res->Bin.Amplitude / (UQ12_4_ONE);
    uint8_t signal_quality = res->Bin.SignalQuality;
    status_t sensor_status = res->Status;

    canardEncodeScalar(uavcan_payload, 0, 24, &range_mm);
    canardEncodeScalar(uavcan_payload, 24, 16, &amplitude);
    canardEncodeScalar(uavcan_payload, 40, 8, &signal_quality);
    canardEncodeScalar(uavcan_payload, 48, 16, &sensor_status);

    static uint8_t transfer_id = 0;

    CanardCANFrame tx_frame;
    tx_frame.id = CANARD_CAN_FRAME_EFF | UAVCAN_MESSAGE_ID;
    tx_frame.data_len = sizeof(uavcan_payload);
    memcpy(tx_frame.data, uavcan_payload, tx_frame.data_len);

    int16_t result = canardBroadcast(&canard,
                                     UAVCAN_MESSAGE_SIGNATURE,
                                     UAVCAN_MESSAGE_ID,
                                     &transfer_id,
                                     CANARD_TRANSFER_PRIORITY_LOW,
                                     uavcan_payload,
                                     sizeof(uavcan_payload));

    if (result >= 0) {
        const CanardCANFrame* txf = NULL;

        while ((txf = canardPeekTxQueue(&canard)) != NULL) {

            // Call CAN_Write without expecting a return value
            CAN_Write((can_frame_t*)txf);

            // Pop the frame from the queue since CAN_Write doesn't return a status
            canardPopTxQueue(&canard);
        }
    } else {
        printf("Failed to broadcast UAVCAN message: %d\n", result);
    }
}







/*
  handle a GetNodeInfo request
*/
static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
    printf("GetNodeInfo request from %d\n", transfer->source_node_id);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = micros64() / 1000000ULL;
    pkt.status = node_status;

    // fill in your major and minor firmware version
    pkt.software_version.major = 1;
    pkt.software_version.minor = 2;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0; // should put git hash in here

    // should fill in hardware version
    pkt.hardware_version.major = 2;
    pkt.hardware_version.minor = 3;

    getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char*)pkt.name.data, "ExampleRangeFinderNode", sizeof(pkt.name.data));
    pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}




/*
  handle a DNA allocation packet
 */
static void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        // already allocated
        return;
    }

    // Rule C - updating the randomized time interval
    DNA.send_next_node_id_allocation_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (rand() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        printf("Allocation request from another allocatee\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation msg;

    uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    getUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        printf("Mismatching allocation response\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        // No match, return
        return;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to
        // the next stage and updating the timeout.
        DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
        DNA.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d\n", msg.unique_id.len);
    } else {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(ins, msg.node_id);
        printf("Node ID allocated: %d\n", msg.node_id);
    }
}


/*
  ask for a dynamic node allocation
 */
static void request_DNA()
{
    const uint32_t now = millis32();
    static uint8_t node_id_allocation_transfer_id = 0;

    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(PREFERRED_NODE_ID << 1U);

    if (DNA.node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;     // First part of unique ID
    }

    uint8_t my_unique_id[16];
    getUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);

    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    const int16_t bcast_res = canardBroadcast(&canard,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                              &node_id_allocation_transfer_id,
                                              CANARD_TRANSFER_PRIORITY_LOW,
                                              &allocation_request[0],
                                              (uint16_t) (uid_size + 1));
    if (bcast_res < 0) {
        printf("Could not broadcast ID allocation req; error %d\n", bcast_res);
    }

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    DNA.node_id_allocation_unique_id_offset = 0;
}


/*


/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
static void send_NodeStatus(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = Time_GetNowSec();
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    node_status.vendor_specific_status_code = 1234;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}


/*
  This function is called at 1 Hz rate from the main loop.
*/
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
      Transmit the node status message
    */
    send_NodeStatus();
}


void CAN_HandleCommand(void)
{
    IRQ_LOCK();
    uint32_t can_rx = can_rx_remote_id;
    /* Rx command handled: clear flags.. */
    can_rx_remote_id = 0;
    IRQ_UNLOCK();

    switch (can_rx)
    {
        case CAN_FRAME_ID_START:
            start_measurements();

            break;

        case CAN_FRAME_ID_STOP:
            stop_measurements();

            break;

        default:
            /* Nothing to do */
            break;
    }
}

/*!***************************************************************************
 * @brief   CAN callback as defined in the "hal_data" module generated by the
 *          Renesas FSP Configuration.
 * @param   p_args The callback arguments provided by HAL.
 *****************************************************************************/
void can_callback(can_callback_args_t * p_args)
{
    assert(p_args != NULL);

    // Handle CAN transmission complete event
    if (CAN_EVENT_TX_COMPLETE & p_args->event)
    {
        // Transmission completed, reset the busy flag
        LED_CAN_TX_OFF();
        is_can_tx_busy = false;
    }

    // Handle CAN reception complete event
    if (CAN_EVENT_RX_COMPLETE & p_args->event)
    {
        // Process the received CAN frame
        CanardCANFrame received_frame;
        received_frame.id = p_args->frame.id;
        received_frame.data_len = p_args->frame.data_length_code;
        memcpy(received_frame.data, p_args->frame.data, received_frame.data_len);

        // Pass the received frame to Libcanard for UAVCAN/DroneCAN processing
        int16_t result = canardHandleRxFrame(&canard, &received_frame, Time_GetNowUSec());
        if (result < 0)
        {
            // Handle errors or unrecognized frames
            printf("Error processing CAN frame: %d\n", result);
        }

        // If Libcanard did not handle the frame, process it further if needed
        if (result == -CANARD_ERROR_RX_WRONG_ADDRESS || result == -CANARD_ERROR_RX_MISSED_START || result == -CANARD_ERROR_RX_WRONG_TOGGLE)
        {
            // Handle non-UAVCAN frames or specific logic here if needed
            // For example: process standard CAN frames or commands
            CAN_HandleCommand(); // Example function to handle specific CAN commands
        }
    }

    // Handle other CAN events
    if (CAN_EVENT_ERR_WARNING & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR WARNING)", p_args->event);
    }

    if (CAN_EVENT_ERR_PASSIVE & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR PASSIVE)", p_args->event);
    }

    if (CAN_EVENT_ERR_BUS_OFF & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR BUS OFF)", p_args->event);
    }

    if (CAN_EVENT_BUS_RECOVERY & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT BUS RECOVERY)", p_args->event);
    }

    if (CAN_EVENT_MAILBOX_MESSAGE_LOST & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT MAILBOX MESSAGE LOST)", p_args->event);
    }

    if (CAN_EVENT_ERR_BUS_LOCK & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR BUS LOCK)", p_args->event);
    }

    if (CAN_EVENT_ERR_CHANNEL & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR CHANNEL)", p_args->event);
    }

    if (CAN_EVENT_ERR_GLOBAL & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT ERROR GLOBAL)", p_args->event);
    }

    if (CAN_EVENT_TX_ABORTED & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT TX ABORTED)", p_args->event);
    }

    if (CAN_EVENT_TX_FIFO_EMPTY & p_args->event)
    {
        error_log("Received CAN event: %d (CAN EVENT TX FIFO EMPTY)", p_args->event);
    }
}



/*! @} */
