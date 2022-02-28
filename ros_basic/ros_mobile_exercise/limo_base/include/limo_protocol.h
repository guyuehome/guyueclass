/*
 * Copyright (c) 2021, Agilex Robotics
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
 */

#ifndef LIMO_PROTOCOL_PARSER_H
#define LIMO_PROTOCOL_PARSER_H

#include <stdint.h>

namespace AgileX {

#define FRAME_HEADER 0x55
#define FRAME_LENGTH 0x0E

/*--------------------------- Message IDs ------------------------------*/
// control group: 0x1
#define MSG_MOTION_COMMAND_ID 0x111

// state feedback group: 0x2
#define MSG_SYSTEM_STATE_ID 0x211
#define MSG_MOTION_STATE_ID 0x221

#define MSG_ACTUATOR1_HS_STATE_ID 0x251
#define MSG_ACTUATOR2_HS_STATE_ID 0x252
#define MSG_ACTUATOR3_HS_STATE_ID 0x253
#define MSG_ACTUATOR4_HS_STATE_ID 0x254

#define MSG_ACTUATOR1_LS_STATE_ID 0x261
#define MSG_ACTUATOR2_LS_STATE_ID 0x262
#define MSG_ACTUATOR3_LS_STATE_ID 0x263
#define MSG_ACTUATOR4_LS_STATE_ID 0x264

// sensor data group: 0x3
#define MSG_ODOMETRY_ID  0x311
#define MSG_IMU_ACCEL_ID 0x321
#define MSG_IMU_GYRO_ID  0x322
#define MSG_IMU_EULER_ID 0x323

#define MSG_CTRL_MODE_CONFIG_ID 0x421

// limo protocol data format
typedef struct {
    double stamp;
    uint16_t id;
    uint8_t data[8];
    uint8_t count;
} LimoFrame;

typedef struct {
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double roll;
    double pitch;
    double yaw;
} ImuData;

enum {
    LIMO_WAIT_HEADER = 0,
    LIMO_WAIT_LENGTH,
    LIMO_WAIT_ID_HIGH,
    LIMO_WAIT_ID_LOW,
    LIMO_WAIT_DATA,
    LIMO_COUNT,
    LIMO_CHECK,
};

enum {
    MODE_FOUR_DIFF = 0x00,
    MODE_ACKERMANN = 0x01,
    MODE_MCNAMU = 0x02,
    MODE_UNKNOWN = 0xff,
};

}

#endif  // LIMO_PROTOCOL_H
