// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef CAN_H
#define CAN_H

// default timeout for a response in milliseconds
#define CAN_DEFAULT_TIMEOUT 1000

// MASKs
#define CAN_EXTD_ID_MASK    0x1FFFFFFF     // 29bit
#define CAN_STD_ID_MASK     0x7FF          // 11bit


/*
#ifdef ARDUINO_ARCH_ESP32
#include "ESP32SJA1000.h"
#else
#include "MCP2515.h"
#endif
*/

#include "ESP32TWAI.h"


#endif
