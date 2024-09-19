/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <ctype.h>

#include "platform.h"
#include "io/serial.h"
#include "drivers/time.h"
#include "common/log.h"

#if defined(USE_RANGEFINDER_A02)
#include "drivers/rangefinder/rangefinder_virtual.h"

typedef struct __attribute__((packed)) {
    uint8_t     header; // Header Byte
    uint8_t     data_h; // Target distance high 8 bit
    uint8_t     data_l; // Target distance low 8 bit
    uint8_t     sum;    // Checksum
} a02Packet_t;

// Note about the A02YYUW rangefinder:
//

// This driver is for the automatic UART version of the A02YYUW rangefinder. There are a few
// different versions of this rangefinder so choose the automatic UART version for this driver to work.
//
// The A02YYUW rangefinder has two modes of operation: automatic UART mode and real-time UART mode.
//
// When RX is unconnected or connected HIGH, this is automatic UART mode which is more accurate
// Response time: 100-300ms
//
// When RX is pulled LOW, the module outputs a real-time value which may not be as accurate.
// Response time: 100ms
// This driver will read the value in either case...

#define A02_HDR             0xFF // Header Byte
#define A02_TIMEOUT_MS      100 // 100ms per reading
#define A02_PACKET_SIZE     sizeof(a02Packet_t)

static bool hasNewData = false;
static bool hasRecievedData = false;
static serialPort_t * serialPort = NULL;
static serialPortConfig_t * portConfig;
static uint8_t buffer[A02_PACKET_SIZE];

static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;
static timeMs_t lastProtocolActivityMs;

static bool a02RangefinderDetect(void)
{
    portConfig = findSerialPortConfig(FUNCTION_RANGEFINDER);
    if (!portConfig) {
        return false;
    }

    return true;
}

static void a02RangefinderInit(void)
{
    LOG_INFO(SYSTEM, "Attempting to detect A02 rangefinder");
    if (!portConfig) {
        return;
    }

    serialPort = openSerialPort(portConfig->identifier, FUNCTION_RANGEFINDER, NULL, NULL, 9600, MODE_RX, SERIAL_NOT_INVERTED);
    if (!serialPort) {
        return;
    }

    lastProtocolActivityMs = 0;
    LOG_INFO(SYSTEM, "A02 rangefinder detected on %d", portConfig->identifier);
}

static void a02RangefinderUpdate(void)
{
    bool validPacket = false;
    uint8_t index = 0;

    a02Packet_t *a02Packet = (a02Packet_t *)buffer;

    while (serialRxBytesWaiting(serialPort) > 0) {

        uint8_t c = serialRead(serialPort);

        // Check for the header byte
        if (c == A02_HDR && index == 0) {
            buffer[index++] = c;
            continue;
        }

        if(index > 0) {
            buffer[index++] = c;
            if(index == A02_PACKET_SIZE) {
                
                // Check for valid checksum
                if(a02Packet->sum == (a02Packet->header + a02Packet->data_h + a02Packet->data_l)) {
                    validPacket = true;
                }
                index = 0;
            }
        }
    }

    // If no valid packet was found, return
    if(!validPacket) {
        return;
    }
    
    hasNewData = true;
    hasRecievedData = true;
    lastProtocolActivityMs = millis();

    sensorData = ((a02Packet->data_h * 0xFF) + a02Packet->data_l);
    LOG_INFO(SYSTEM, "Range: %ldmm", sensorData);
}

static int32_t a02RangefinderGetDistance(void)
{
    int32_t altitude = (sensorData > 0) ? (sensorData) : RANGEFINDER_OUT_OF_RANGE;

    if (hasNewData) {
        hasNewData = false;
        return altitude;
    }
    else {

        if ((millis() - lastProtocolActivityMs) < A02_TIMEOUT_MS) {
            return altitude;
        }

        return hasRecievedData ? RANGEFINDER_OUT_OF_RANGE : RANGEFINDER_NO_NEW_DATA;
    }
}

virtualRangefinderVTable_t rangefinderA02Vtable = {
    .detect = a02RangefinderDetect,
    .init = a02RangefinderInit,
    .update = a02RangefinderUpdate,
    .read = a02RangefinderGetDistance
};

#endif
