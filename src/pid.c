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
#ifdef EPID_FEATURE_VALID_FLT
    if ((isfinite(xk_1) == 0)
     || (isfinite(xk_2) == 0)
     || (isfinite(y_previous) == 0)
     || (isfinite(kp) == 0)
     || (isfinite(ki) == 0)
     || (isfinite(kd) == 0) /* Okay to be zero for PI controller. */
    ) {
        return EPID_ERR_FLT;
    }
#endif

    if ((ctx == NULL)
     || (kp <= EPID_FP_ZERO)
     || (ki <= EPID_FP_ZERO)
     || (kd < EPID_FP_ZERO)
    ) {
        return EPID_ERR_INIT;
    }

    /* Set previous states for equations. */
    ctx->xk_1 = xk_1; /* Set `x[k-1]` */
    ctx->xk_2 = xk_2; /* Set `x[k-1]` for D-term */
    ctx->y_out = y_previous; /* Set `y[k-1]` */

    /* Direct gains assignments. */
    ctx->kp = kp; /* P-term gain constant. */
    ctx->ki = ki; /* I-term gain constant. */
    ctx->kd = kd; /* D-term gain constant. */

    return EPID_ERR_NONE;
}


epid_info_t epid_init_T(epid_t *ctx,
                        float xk_1, float xk_2, float y_previous,
                        float kp, float ti, float td,
                        float sample_period)
{
#ifdef EPID_FEATURE_VALID_FLT
    if ((isfinite(ti) == 0)
     || (isfinite(sample_period) == 0)
    ) {
        return EPID_ERR_FLT;
    }
#endif

    if ((ti <= EPID_FP_ZERO)
     || (td <  EPID_FP_ZERO) /* Okay to be zero for PI controller. */
     || (sample_period <= EPID_FP_ZERO)
    ) {
        return EPID_ERR_INIT;
    }
    
    /* I-term gain constant; `Ki = Kp / (Ti / Ts) = (Kp * Ts) / Ti` */
    const float ki = (kp * sample_period) / ti;
    /* D-term gain constant; `Kd = Kp * (Td / Ts)` */
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

    ctx->xk_1 = measure; /* `x[k-1] = x[k]` */
}


void epid_pid_calc(epid_t *ctx, float setpoint, float measure)
{
    /* P-term value: `P[k] = Kp * (x[k-1] - x[k])`
     * I-term value: `I[k] = Ki * e[k] = Ki * (SP - x[k])`
     * D-term value: `D[k] = Kd * (2*x[k-1] - x[k-2] - x[k])`
     */
    ctx->p_term = ctx->xk_1 - measure;
    ctx->d_term = ctx->kd * (ctx->xk_1 + ctx->p_term - ctx->xk_2);
    ctx->p_term = ctx->kp * ctx->p_term;
    ctx->i_term = ctx->ki * (setpoint - measure);

    ctx->xk_2 = ctx->xk_1; /* `x[k-2] = x[k-1]` */
    ctx->xk_1 = measure;   /* `x[k-1] = x[k]` */
}


void epid_pi_sum(epid_t *ctx, float out_min, float out_max)
{
#ifdef EPID_FEATURE_VALID_FLT
    const float y_prev = ctx->y_out;
#endif

    /* `y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k]` */
    ctx->y_out += ctx->p_term + ctx->i_term;

#ifdef EPID_FEATURE_VALID_FLT
    if ((isnan(ctx->y_out) != 0)
     || (isnan(ctx->p_term) != 0)
     || (isnan(ctx->i_term) != 0)
    ) {
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
    const float y_prev = ctx->y_out;
#endif

    /* `y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k] + D[k]` */
    ctx->y_out += ctx->p_term + ctx->i_term + ctx->d_term;

#ifdef EPID_FEATURE_VALID_FLT
    if ((isnan(ctx->y_out) != 0)
     || (isnan(ctx->p_term) != 0)
     || (isnan(ctx->i_term) != 0)
     || (isnan(ctx->d_term) != 0)
    ) {
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


epid_info_t epid_util_lpf_init(epid_lpf_t *ctx, float smoothing_factor, float x_0)
{
#ifdef EPID_FEATURE_VALID_FLT
    if ((isfinite(smoothing_factor) == 0)
     || (isfinite(x_0) == 0)
    ) {
        return EPID_ERR_FLT;
    }
#endif

    if ((ctx == NULL)
     || (smoothing_factor <= EPID_FP_ZERO)
     || (smoothing_factor >= EPID_FP_ONE)
    ) {
        return EPID_ERR_INIT;
    }

    /* Filter's smoothing factor. `0 < a < 1` */
    ctx->smoothing_factor = smoothing_factor;
    /* `y[0] = smoothing_factor * x[0]` */
    ctx->y = smoothing_factor * x_0;

    return EPID_ERR_NONE;
}


void epid_util_lpf_calc(epid_lpf_t *ctx, float input)
{
    /* Infinite-impulse-response (IIR) single-pole low-pass filter,
     * an exponentially weighted moving average (EMA).
     * `y[k] = FILTER(x[k]) = y[k-1] + smoothing_factor * (x[k] - y[k-1])`
     */
    const float y_prev = ctx->y;
    ctx->y = y_prev + ctx->smoothing_factor * (input - y_prev);
}


#ifdef __cplusplus
}
#endif
