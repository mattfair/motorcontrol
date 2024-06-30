/*
 * structs.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"
#include "calibration.h"
#include "can.h"
#include "o1heap.h"
#include "canard.h"

#define CAN_REDUNDANCY_FACTOR 1

typedef struct{
    } GPIOStruct;

typedef struct{
    }COMStruct;

typedef struct State
{
    CanardMicrosecond started_at;

    O1HeapInstance* heap;
    CanardInstance canard;
    CanardTxQueue canard_tx_queues[CAN_REDUNDANCY_FACTOR];
    uint8_t can_payload_buffer[CANARD_MTU_CAN_CLASSIC];
    CAN_RxHeaderTypeDef rx_header;
    CAN_HandleTypeDef canbus[CAN_REDUNDANCY_FACTOR];

    /// The state of the business logic.
    struct
    {
        /// Whether the servo is supposed to actuate the load or to stay idle (safe
        /// low-power mode).
        struct
        {
            bool armed;
            CanardMicrosecond last_update_at;
        } arming;

        /// Setpoint & motion profile (unsupported constraints are to be ignored).
        /// https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/actuator/servo/_.0.1.dsdl
        /// As described in the linked documentation, there are two kinds of servos
        /// supported: linear and rotary. Units per-kind are:   LINEAR ROTARY
        float position;      ///< [meter]                [radian]
        float velocity;      ///< [meter/second]         [radian/second]
        float acceleration;  ///< [(meter/second)^2]     [(radian/second)^2]
        float force;         ///< [newton]               [netwon*meter]
    } servo;

    /// These values are read from the registers at startup. You can also
    /// implement hot reloading if desired. The subjects of the servo network
    /// service are defined in the UDRAL data type definitions here:
    /// https://github.com/OpenCyphal/public_regulated_data_types/blob/master/reg/udral/service/actuator/servo/_.0.1.dsdl
    struct
    {
        struct
        {
            CanardPortID servo_feedback;  //< reg.udral.service.actuator.common.Feedback
            CanardPortID servo_status;    //< reg.udral.service.actuator.common.Status
            CanardPortID servo_power;     //< reg.udral.physics.electricity.PowerTs
            CanardPortID servo_dynamics;  //< (timestamped dynamics)
        } pub;
        struct
        {
            CanardPortID servo_setpoint;   //< (non-timestamped dynamics)
            CanardPortID servo_readiness;  //< reg.udral.service.common.Readiness
        } sub;
    } port_id;

    /// A transfer-ID is an integer that is incremented whenever a new message is
    /// published on a given subject. It is used by the protocol for
    /// deduplication, message loss detection, and other critical things. For CAN,
    /// each value can be of type uint8_t, but we use larger types for genericity
    /// and for statistical purposes, as large values naturally contain the number
    /// of times each subject was published to.
    struct
    {
        uint64_t uavcan_node_heartbeat;
        uint64_t uavcan_node_port_list;
        uint64_t uavcan_pnp_allocation;
        // Messages published synchronously can share the same transfer-ID:
        uint64_t servo_fast_loop;
        uint64_t servo_1Hz_loop;
    } next_transfer_id;

    FSMStruct fsm;
} State;


/* Global Structs */
extern ControllerStruct controller;
extern ObserverStruct observer;
extern COMStruct com;
extern FSMStruct state;
extern EncoderStruct comm_encoder;
extern DRVStruct drv;
extern PreferenceWriter prefs;
extern CalStruct comm_encoder_cal;
extern State servo_state;

#endif
