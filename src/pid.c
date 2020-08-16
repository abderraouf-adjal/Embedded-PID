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


#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"


epid_info_t epid_init(epid_t *ctx,
                      float xk_1, float xk_2, float y_previous,
                      float kp, float ki, float kd)
{
    if ((ctx == NULL)
     || (kp < EPID_NUM_ZERO)
     || (ki < EPID_NUM_ZERO)
     || (kd < EPID_NUM_ZERO)
    ) {
        return EPID_ERR_INIT;
    }

    /* Set previous states for equations. */
    ctx->xk_1 = xk_1; /* Set `x[k-1]` */
    ctx->xk_2 = xk_2; /* Set `x[k-1]` for D-term */
    ctx->y_out = y_previous; /* Set `y[k-1]` */

    /* P-term gain constant. */
    ctx->kp = kp; /* Direct `Kp` assignment. */
    /* I-term gain constant. */
    ctx->ki = ki; /* Direct `Ki` assignment. */
    /* D-term gain constant. */
    ctx->kd = kd; /* Direct `Kd` assignment. */

#ifdef EPID_FEATURE_VALID_FLT
    if ((isfinite(ctx->xk_1) == 0)
     || (isfinite(ctx->xk_2) == 0)
     || (isfinite(ctx->y_out) == 0)
     || (isfinite(ctx->kp) == 0)
     || (isfinite(ctx->ki) == 0)
     || (isfinite(ctx->kd) == 0)
    ) {
        return EPID_ERR_FLT;
    }
#endif

    return EPID_ERR_NONE;
}


epid_info_t epid_init_T(epid_t *ctx,
                        float xk_1, float xk_2, float y_previous,
                        float kp, float ti, float td,
                        float sample_period)
{
    if ((ti <= EPID_NUM_ZERO)
     || (td <  EPID_NUM_ZERO)
     || (sample_period <= EPID_NUM_ZERO)
    ) {
        return EPID_ERR_INIT;
    }
    
    /* I-term gain constant. */
    /* `Ki = Kp / (Ti / Ts) = (Kp * Ts) / Ti` */
    const float ki = (kp * sample_period) / ti;
    /* D-term gain constant. */
    /* `Kd = Kp * (Td / Ts)` */
    const float kd = kp * (td / sample_period);

    return epid_init(ctx,
                     xk_1, xk_2, y_previous,
                     kp, ki, kd);
}


void epid_pi_calc(epid_t *ctx, float setpoint, float measure)
{
    /* P-term value: `P[k] = Kp * (x[k-1] - x[k])`
     * I-term value: `I[k] = Ki * e[k] = Ki * (SP - x[k])`
     */
    ctx->p_term = ctx->kp * (ctx->xk_1 - measure);
    ctx->i_term = ctx->ki * (setpoint - measure);

    /* `x[k-1] = x[k]` */
    ctx->xk_1 = measure;
}


void epid_pid_calc(epid_t *ctx, float setpoint, float measure)
{
    /* P-term value: `P[k] = Kp * (x[k-1] - x[k])`
     * I-term value: `I[k] = Ki * e[k] = Ki * (SP - x[k])`
     * D-term value: `D[k] = Kp * (2*x[k-1] - x[k-2] - x[k])`
     */
    ctx->p_term = ctx->kp * (ctx->xk_1 - measure);
    ctx->i_term = ctx->ki * (setpoint - measure);
    ctx->d_term = ctx->kd
                * (ctx->xk_1 + ctx->xk_1 - ctx->xk_2 - measure);

    /* `x[k-2] = x[k-1]` */
    ctx->xk_2 = ctx->xk_1;
    /* `x[k-1] = x[k]` */
    ctx->xk_1 = measure;
}


void epid_pi_sum(epid_t *ctx, float out_min, float out_max)
{
#ifdef EPID_FEATURE_VALID_FLT
    /* Note: Checking `epid_init*()` errors is recommended. */
    const float y_prev = ctx->y_out;
#endif

    /* `y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k]` */
    ctx->y_out += ctx->p_term + ctx->i_term;

#ifdef EPID_FEATURE_VALID_FLT
    if (isnan(ctx->y_out) != 0) {
        ctx->y_out = y_prev;
    }
#endif

    /* Limit the new output `y[k]` (CV) to boundaries. */
    if (ctx->y_out > out_max) {
        ctx->y_out = out_max;
    }
    else if (ctx->y_out < out_min) {
        ctx->y_out = out_min;
    }
}


void epid_pid_sum(epid_t *ctx, float out_min, float out_max)
{
#ifdef EPID_FEATURE_VALID_FLT
    /* Note: Checking `epid_init*()` errors is recommended. */
    const float y_prev = ctx->y_out;
#endif

    /* `y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k] + D[k]` */
    ctx->y_out += ctx->p_term + ctx->i_term + ctx->d_term;

#ifdef EPID_FEATURE_VALID_FLT
    if (isnan(ctx->y_out) != 0) {
        ctx->y_out = y_prev;
    }
#endif

    /* Limit the new output `y[k]` (CV) to boundaries. */
    if (ctx->y_out > out_max) {
        ctx->y_out = out_max;
    }
    else if (ctx->y_out < out_min) {
        ctx->y_out = out_min;
    }
}


void epid_util_ilim(epid_t *ctx, float i_min, float i_max)
{
    /* Limit I-term `I[k]` value to boundaries as an integrator anti-windup. */
    if (ctx->i_term > i_max) {
        ctx->i_term = i_max;
    }
    else if (ctx->i_term < i_min) {
        ctx->i_term = i_min;
    }
}


#ifdef __cplusplus
}
#endif
