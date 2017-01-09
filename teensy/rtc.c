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

#include "rtc.h"
#include "irq.h"
#include "py/gc.h"
#include "py/runtime.h"
#include "timeutils.h"
#include "core_pins.h"

#if MICROPY_HW_ENABLE_RTC

#define RTC_IER_TAIE    (1 << 2)

#if 0
rtc = RTC(id=0, datetime=(year, month, day, hour=0, minute=0, second=0, microsecond=0, tzinfo=None)) create an RTC object instance and set the current time.
rtc.init() re-init
rtc.now() returns a datetime tuple with the current time and date.
rtc.alarm(alarm_id, time=time_ms or datetime_tuple, *, repeat=False) sets the RTC alarm. If the alarm has already expired the returned value will be 0. An alarm can be set both via passing an integer with the number of milliseconds or a datetime tuple with a future time. If the datetime tuple contains a past time, the alarm will expire immediately triggering any enabled IRQ. The same applies when passing a 0 or negative time_ms value.
rtc.alarm_left(alarm_id) get the number of milliseconds left before the alarm expires. Returns 0 if the alarm is already expired.
rtc.alarm_cancel(alarm_id) cancel a running alarm.
rtc.calibration([cal_value]) get or set the RTC calibration value. Platform dependent.
rtc.irq(*, trigger, handler, priority, wake) calls the handler function once the alarm expires. See the IRQ section for details.
#endif

typedef struct {
    mp_obj_base_t base;
} pyb_rtc_obj_t;

// singleton RTC object
STATIC const pyb_rtc_obj_t pyb_rtc_obj = {{&pyb_rtc_type}};

void rtc_init(void) {

    // RTC_IER starts out being set to 7 (on reset), so we clear it here.

    RTC_IER = 0;
}

STATIC void pyb_rtc_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    (void)self_in;
    (void)kind;
    mp_printf(print, "RTC()");
}

STATIC mp_obj_t pyb_rtc_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return (mp_obj_t)&pyb_rtc_obj;
}

STATIC mp_obj_t pyb_rtc_alarm(mp_obj_t self_in, mp_obj_t alarm_id, mp_obj_t time_in) {
    (void)self_in; // unused

    // check we want alarm0
    if (mp_obj_get_int(alarm_id) != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "invalid alarm"));
    }

    // The teensy RTC alarm is only in seconds. We accept milliseconds for
    // compatability with other ports, and convert to seconds.

    mp_int_t duration = mp_obj_get_int(time_in);
    if (duration <= 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "time must be positive"));
    }
    duration /= 1000;

    RTC_TAR = rtc_get() + duration;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_rtc_alarm_obj, pyb_rtc_alarm);

STATIC mp_obj_t pyb_rtc_alarm_left(mp_obj_t self_in, mp_obj_t alarm_id) {
    (void)self_in;

    if (mp_obj_get_int(alarm_id) != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "invalid alarm"));
    }

    uint32_t tar = RTC_TAR;
    uint32_t tsr = RTC_TSR;

    // TODO: Deal with wrapping? This will occur if the current time is less
    //       than 0xffffffff (in 2038) and the alarm is set some point after
    // that (so the alarm would be a small positive number).
    if (tsr < tar) {
        return mp_obj_new_int_from_uint((tar - tsr) * 1000);
    }
    return MP_OBJ_NEW_SMALL_INT(0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_rtc_alarm_left_obj, pyb_rtc_alarm_left);

STATIC mp_obj_t pyb_rtc_alarm_cancel(mp_obj_t self_in, mp_obj_t alarm_id) {
    (void)self_in;

    if (mp_obj_get_int(alarm_id) != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "invalid alarm"));
    }

    RTC_TAR = 0;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_rtc_alarm_cancel_obj, pyb_rtc_alarm_cancel);

STATIC mp_obj_t pyb_rtc_datetime(mp_uint_t n_args, const mp_obj_t *args) {
    if (n_args == 1) {
        // get date and time
        timeutils_struct_time_t tm;
        mp_uint_t t = rtc_get() - RTC_TIME_OFFSET;
        timeutils_seconds_since_2000_to_struct_time(t, &tm);
        mp_obj_t tuple[8] = {
            mp_obj_new_int(tm.tm_year),
            mp_obj_new_int(tm.tm_mon),
            mp_obj_new_int(tm.tm_mday),
            mp_obj_new_int(tm.tm_wday),
            mp_obj_new_int(tm.tm_hour),
            mp_obj_new_int(tm.tm_min),
            mp_obj_new_int(tm.tm_sec),
            mp_obj_new_int(0),
        };
        return mp_obj_new_tuple(8, tuple);
    }

    // set date and time
    mp_obj_t *items;
    mp_obj_get_array_fixed_n(args[1], 8, &items);

    mp_uint_t t = timeutils_mktime(mp_obj_get_int(items[0]),
                                   mp_obj_get_int(items[1]),
                                   mp_obj_get_int(items[2]),
                                   mp_obj_get_int(items[4]),
                                   mp_obj_get_int(items[5]),
                                   mp_obj_get_int(items[6]));
    rtc_set(t + RTC_TIME_OFFSET);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_rtc_datetime_obj, 1, 2, pyb_rtc_datetime);

STATIC mp_obj_t pyb_rtc_calibration(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_tcr, ARG_cir, ARG_tcv, ARG_cic };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_tcr, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_cir, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_tcv, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_cic, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t tcr_reg = RTC_TCR;
    uint32_t tcr = (tcr_reg >>  0) & 0xff;
    uint32_t cir = (tcr_reg >>  8) & 0xff;
    uint32_t tcv = (tcr_reg >> 16) & 0xff;
    uint32_t cic = (tcr_reg >> 24) & 0xff;

    if (args[ARG_tcr].u_obj != mp_const_none) {
        tcr = mp_obj_get_int(args[ARG_tcr].u_obj);
    }
    if (args[ARG_cir].u_obj != mp_const_none) {
        cir = mp_obj_get_int(args[ARG_cir].u_obj);
    }
    if (args[ARG_tcv].u_obj != mp_const_none) {
        tcv = mp_obj_get_int(args[ARG_tcv].u_obj);
    }
    if (args[ARG_cic].u_obj != mp_const_none) {
        cic = mp_obj_get_int(args[ARG_cic].u_obj);
    }
    tcr_reg = RTC_TCR_CIC(cic) | RTC_TCR_TCV(tcv) | RTC_TCR_CIR(cir) | RTC_TCR_TCR(tcr);
    RTC_TCR = tcr_reg;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_rtc_calibration_obj, 2, pyb_rtc_calibration);

STATIC mp_obj_t pyb_rtc_irq(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    // parse args
    enum { ARG_trigger, ARG_priority, ARG_handler, ARG_wake };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_trigger, MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_priority, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_handler, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_wake, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (args[ARG_trigger].u_int != 0) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "invalid alarm"));
    }

    // ignore priority for now.
    // ignore wake for now

    MP_STATE_PORT(rtc_irq_handler) = args[ARG_handler].u_obj;
    if (MP_STATE_PORT(rtc_irq_handler) == mp_const_none) {
        NVIC_DISABLE_IRQ(IRQ_RTC_ALARM);
        RTC_IER &= ~RTC_IER_TAIE;
    } else {
        NVIC_SET_PRIORITY(IRQ_RTC_ALARM, IRQ_PRI_RTC_WKUP * 16);
        NVIC_ENABLE_IRQ(IRQ_RTC_ALARM);
        RTC_IER |= RTC_IER_TAIE;
    }

    // with no arguments returns the current irq object
    // TODO: Implement proper irq object. For now, we just return None

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_rtc_irq_obj, 1, pyb_rtc_irq);

void rtc_alarm_isr(void) {
    if ((RTC_SR & RTC_SR_TAF) == 0) {
        // We were called for some unknown reason. Report it and disable
        // all interrupts.
        NVIC_DISABLE_IRQ(IRQ_RTC_ALARM);
        mp_printf(&mp_plat_print, "RTC unknown irq SR = %x\n", RTC_SR);
        return;
    }

    // Setting RTC_TAR to zero clears the TAF bit in the RTC_SR register.
    // Do this before calling the callback, since the callback might want
    // to re-arm the alarm.
    RTC_TAR = 0;

    if (MP_STATE_PORT(rtc_irq_handler) != mp_const_none) {
        // When executing code within a handler we must lock the GC to prevent
        // any memory allocations.  We must also catch any exceptions.
        gc_lock();
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_call_function_1(MP_STATE_PORT(rtc_irq_handler), (mp_obj_t)&pyb_rtc_obj);
            nlr_pop();
        } else {
            // Uncaught exception; disable the callback so it doesn't run again.
            MP_STATE_PORT(rtc_irq_handler) = mp_const_none;
            mp_printf(&mp_plat_print, "Uncaught exception in RTC interrupt handler\n");
            mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
        }
        gc_unlock();
    }
}

STATIC const mp_map_elem_t pyb_rtc_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_alarm), (mp_obj_t)&pyb_rtc_alarm_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_alarm_left), (mp_obj_t)&pyb_rtc_alarm_left_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_alarm_cancel), (mp_obj_t)&pyb_rtc_alarm_cancel_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_datetime), (mp_obj_t)&pyb_rtc_datetime_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_calibration), (mp_obj_t)&pyb_rtc_calibration_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_irq), (mp_obj_t)&pyb_rtc_irq_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ALARM0), MP_OBJ_NEW_SMALL_INT(0) },
};
STATIC MP_DEFINE_CONST_DICT(pyb_rtc_locals_dict, pyb_rtc_locals_dict_table);

const mp_obj_type_t pyb_rtc_type = {
    { &mp_type_type },
    .name = MP_QSTR_RTC,
    .print = pyb_rtc_print,
    .make_new = pyb_rtc_make_new,
    .locals_dict = (mp_obj_t)&pyb_rtc_locals_dict,
};

#endif // MICROPY_HW_ENABLE_RTC

