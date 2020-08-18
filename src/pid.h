/* SPDX-License-Identifier: ISC */
/**
 * Copyright (c) 2020 Abderraouf Adjal
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * Project name: EPID (Embedded Proportional-Integral-Derivative (PID) controller).
 * Semantic versioning: 1.0.2
 * Version date (ISO-8601): 2020-08-16
 * C standard: C99 (ISO/IEC 9899:1999) or later.
 * 
 * Description:
 * Portable implementation of "Type-C PID controller" for both hosted
 * and freestanding C environments with a flexible API that allow the usage of
 * third-party external and/or internal filter(s) for a better control
 * backed with errors and exceptions handling.
 * 
 * Equations brief [*]:
 * `P[k] = Kp * (x[k-1] - x[k])`
 * `I[k] = Ki * e[k] = Ki * (SP - x[k])`
 * `D[k] = Kp * (2*x[k-1] - x[k-2] - x[k])`
 * `y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k] + D[k]`
 * `x`: Measured process variable (PV).
 * `y`: Control variable (CV/CO).
 * 
 * [*]: D. M. Auslander, Y. Takahashi and M. Tomizuka, "Direct digital process
 * control: Practice and algorithms for microprocessor application,"
 * in Proceedings of the IEEE, vol. 66, no. 2, pp. 199-208, Feb. 1978,
 * doi: 10.1109/PROC.1978.10870.
 */


#ifndef EPID_H
#define EPID_H 1


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h> /* For `NULL`. */

/* A switch to define `EPID_FEATURE_VALID_FLT`. */
#if 1
# define EPID_FEATURE_VALID_FLT 1 /* To check against floating-point errors. */
# include <math.h> /* For `isfinite()` and `isnan()` (implementation defined). */
#endif

/* API and behavior semantic versioning. */
#define EPID_LIB_VERSION "1.0.1"

/* To help with data-type modification. */
#define EPID_NUM_ZERO 0.0f

/* For errors management; Type: epid_info_t */
#define EPID_ERR_INIT (0U) /* Bad Initialization. */
#define EPID_ERR_FLT (1U) /* Floating-point error. */
#define EPID_ERR_NONE (2U) /* No error detected. */


typedef uint_fast8_t epid_info_t; /* An unsigned type for errors flag. */

typedef struct {
    /* Controller settings. */
    float kp; /* Gain constant `Kp` for P-term. */
    float ki; /* Gain constant `Ki` for I-term. */
    float kd; /* Gain constant `Kd` for D-term. */

    /* Controller states. */
    float xk_1; /* Physical measurement `PV[k-1]`. */
    float xk_2; /* Physical measurement `PV[k-2]`. */

    /* Controller outputs. */
    float p_term; /* The P-term calculated value `P[k]`. */
    float i_term; /* The I-term calculated value `I[k]`. */
    float d_term; /* The D-term calculated value `D[k]`. */

    float y_out; /* The controller output (CV). `y[k] = y[k-1] + delta[k]` */
} epid_t;


/**
 * Initialize a `epid_t` context by direct gains assignment,
 * and set {`x[k-1]`, `x[k-2]`, `y[k-1]`}.
 * 
 * ctx: Pointer to the `epid_t` context.
 * xk_1: A process variable (PV) point `x[k-1]`.
 * xk_2: A process variable (PV) point `x[k-2]` for D-term.
 * y_out_previous: A control variable (CV) point `y[k-1]`.
 * kp: Gain constant `Kp` for P-term.
 * ki: Gain constant `Ki` for I-term.
 * kd: Gain constant `Kd` for D-term.
 * 
 * Return:
 *   - `EPID_ERR_NONE` on success.
 *   - `EPID_ERR_INIT` if initialization error occurred.
 *   - `EPID_ERR_FLT` if floating-point arithmetic error occurred.
 */
epid_info_t epid_init(epid_t *ctx,
                      float xk_1, float xk_2, float y_previous,
                      float kp, float ki, float kd);


/**
 * Initialize a `epid_t` context by `Kp` gain and time constants `Ti` and `Td`,
 * and set {`x[k-1]`, `x[k-2]`, `y[k-1]`}.
 * `Ki = Kp / (Ti / Ts) = (Kp * Ts) / Ti`
 * `Kd = Kp * (Td / Ts)`
 * 
 * ctx: Pointer to the `epid_t` context.
 * xk_1: A process variable (PV) point `x[k-1]`.
 * xk_2: A process variable (PV) point `x[k-2]` for D-term.
 * y_out_previous: A control variable (CV) point `y[k-1]`.
 * kp: Gain constant `Kp` for P-term.
 * ti: Rate time constant for I-term [1 / time-unit]; `Ti = Kp / Ki`.
 * td: Reset time constant for D-term [time-unit]; `Td = Kd / Kp`.
 * sample_period: Sample time period in [time-unit] for `Ti` and `Td`.
 * 
 * Return:
 *   - `EPID_ERR_NONE` on success.
 *   - `EPID_ERR_INIT` if initialization error occurred.
 *   - `EPID_ERR_FLT` if floating-point arithmetic error occurred.
 */
epid_info_t epid_init_T(epid_t *ctx,
                        float xk_1, float xk_2, float y_previous,
                        float kp, float ti, float td,
                        float sample_period);


/**
 * Do processing as a Type-C PI controller to calculated and update
 * terms {`P[k]`, `I[k]`} in in `epid_t` context.
 * 
 * ctx: Pointer to the `epid_t` context.
 * setpoint: The desired setpoint (SP).
 * measure: Measured process variable (PV).
 */
void epid_pi_calc(epid_t *ctx, float setpoint, float measure);


/**
 * Do processing as a Type-C PID controller to calculated and update
 * terms {`P[k]`, `I[k]`, `D[k]`} in in `epid_t` context.
 * Note: There is NO noise filtering on the derivative-term (`D[k]`).
 * 
 * ctx: Pointer to the `epid_t` context.
 * setpoint: The desired setpoint (SP).
 * measure: Measured process variable (PV).
 */
void epid_pid_calc(epid_t *ctx, float setpoint, float measure);


/**
 * Do processing as a Type-C PID controller and update the calculated
 * control variable (`y[k]`) in `epid_t` context.
 * `y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k]`
 * 
 * ctx: Pointer to the `epid_t` context.
 * out_min: Min output from controller.
 * out_max: Max output from controller.
 */
void epid_pi_sum(epid_t *ctx, float out_min, float out_max);


/**
 * Do processing as a Type-C PID controller and update the calculated
 * control variable (`y[k]`) in `epid_t` context.
 * Note: There is NO noise filtering on the derivative-term (`D[k]`).
 * `y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k] + D[k]`
 * 
 * ctx: Pointer to the `epid_t` context.
 * out_min: Min output from controller.
 * out_max: Max output from controller.
 */
void epid_pid_sum(epid_t *ctx, float out_min, float out_max);


/**
 * Limit I-term `I[k]` value to boundaries as an integrator anti-windup.
 * 
 * ctx: Pointer to the `epid_t` context.
 * i_min: Min `I[k]` value.
 * i_max: Max `I[k]` value.
 */
void epid_util_ilim(epid_t *ctx, float i_min, float i_max);


#ifdef __cplusplus
}
#endif

#endif /* EPID_H */
