/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Dave Hylands
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __MICROPY_INCLUDED_TEENSY_RTC_H__
#define __MICROPY_INCLUDED_TEENSY_RTC_H__

#include "py/obj.h"

// The teensy sets the RTC using local time and the unix epoch (Jan 1, 1970)
// TIME_OFFSET is the number of seconds from Jan 1, 1970 until Jan 1, 2000,
// which is the MicroPython epoch.
//
// We keep the RTC using the the 1970 epoch so that if somebody switches back
// and forth between Teensyduino and MicroPython then the RTC will remain correct.

#define RTC_TIME_OFFSET 946684800

extern const mp_obj_type_t pyb_rtc_type;

void rtc_init(void);

#endif  // __MICROPY_INCLUDED_TEENSY_RTC_H__
