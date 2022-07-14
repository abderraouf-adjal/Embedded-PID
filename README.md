# EPID: Type-C PID controller library

---

## Description

Portable implementation of **Type-C PID controller** for both hosted
and freestanding C environments with a flexible API that allow the usage of
third-party external and/or internal filter(s) for a better control
backed with errors and exceptions handling.

Equations brief *[\*]*:

```math
P[k] = Kp * (x[k-1] - x[k])
I[k] = Ki * e[k] = Ki * (SP - x[k])
D[k] = Kd * (2*x[k-1] - x[k-2] - x[k])

y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k] + D[k]

x: Measured process variable (PV).
y: Control variable (CV/CO).
```

---

## Features

- Portable code: C standard ISO/IEC 9899:1999 (C99) or later.
- Usage of IEEE-754 standard for floating-point arithmetic.
- Handling of floating-point arithmetic errors.
- Flexible API by dividing processing functions to allow the usage of
third-party external and/or internal filter(s).
- Permissive free software license (ISC).

---

## API and usage synopsis

## Macro constants and library settings

```c
#include <pid.h> /* Header file */

EPID_LIB_VERSION "x.y.z" /* API and behavior semantic versioning. */
EPID_FEATURE_VALID_FLT /* To check against floating-point errors. */

/* For errors management; Type: epid_info_t */
EPID_ERR_NONE (2U) /* No error detected. */
EPID_ERR_INIT (0U) /* Bad Initialization. */
EPID_ERR_FLT (1U) /* Floating-point error. */
epid_info_t; /* Type for errors flag. */
```

## Context structures

### `epid_t`

PID context structure.

```c
typedef struct
{
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
```

### `epid_lpf_t`

Low-pass filter context structure.

```c
typedef struct
{
    float smoothing_factor; /* Filter's smoothing factor. `0 <= a <= 1` */
    float y; /* `y[k] = FILTER(x[k])` */
} epid_lpf_t;
```

## User functions

### Initialization functions

```c
/*
Initialize or reset a `epid_t` context by direct gains assignment,
and set {`x[k-1]`, `x[k-2]`, `y[k-1]`}.

ctx: Pointer to the `epid_t` context.
xk_1: A process variable (PV) point `x[k-1]`.
xk_2: A process variable (PV) point `x[k-2]` for D-term.
y_previous: A control variable (CV) point `y[k-1]`.
kp: Gain constant `Kp` for P-term.
ki: Gain constant `Ki` for I-term.
kd: Gain constant `Kd` for D-term.

- {kp, ki, kd} must not be negative.
- {kp, ki} must not be zero.
- {xk_1, xk_2, y_previous, kp, ki, kd} must not be NAN, or INF.

Return:
  - `EPID_ERR_NONE` on success.
  - `EPID_ERR_INIT` if initialization error occurred.
  - `EPID_ERR_FLT` if floating-point arithmetic error occurred.
*/
epid_info_t
epid_init(epid_t *ctx,
          float xk_1, float xk_2, float y_previous,
          float kp, float ki, float kd);
```

```c
/*
Initialize or reset a `epid_t` context by `Kp` gain and time constants `Ti` and `Td`,
and set {`x[k-1]`, `x[k-2]`, `y[k-1]`}.
`Ki = Kp / (Ti / Ts) = (Kp * Ts) / Ti`
`Kd = Kp * (Td / Ts)`

ctx: Pointer to the `epid_t` context.
xk_1: A process variable (PV) point `x[k-1]`.
xk_2: A process variable (PV) point `x[k-2]` for D-term.
y_previous: A control variable (CV) point `y[k-1]`.
kp: Gain constant `Kp` for P-term.
ti: Rate time constant for I-term [1 / time-unit]; `Ti = Kp / Ki`.
td: Reset time constant for D-term [time-unit]; `Td = Kd / Kp`.
sample_period: Sample time period in [time-unit] for `Ti` and `Td`.

- {kp, ti, td, sample_period} must not be negative.
- {ti, sample_period} must not be zero.
- {xk_1, xk_2, y_previous, kp, ti, td, sample_period} must not be NAN, or INF.

Return:
  - `EPID_ERR_NONE` on success.
  - `EPID_ERR_INIT` if initialization error occurred.
  - `EPID_ERR_FLT` if floating-point arithmetic error occurred.
*/
epid_info_t
epid_init_T(epid_t *ctx,
            float xk_1, float xk_2, float y_previous,
            float kp, float ti, float td,
            float sample_period);
```

### Processing functions

#### Step **one (1)** processing functions

`epid_pid_calc()`, `epid_pi_calc()`.

```c
/*
Do processing as a Type-C PI controller to calculated and update
terms {`P[k]`, `I[k]`} in in `epid_t` context.

ctx: Pointer to the `epid_t` context.
setpoint: The desired setpoint (SP).
measure: Measured process variable (PV).
*/
void
epid_pi_calc(epid_t *ctx, float setpoint, float measure);
```

```c
/*
Do processing as a Type-C PID controller to calculated and update
terms {`P[k]`, `I[k]`, `D[k]`} in in `epid_t` context.
Note: There is NO noise filtering on the derivative-term (`D[k]`).

ctx: Pointer to the `epid_t` context.
setpoint: The desired setpoint (SP).
measure: Measured process variable (PV).
*/
void
epid_pid_calc(epid_t *ctx, float setpoint, float measure);
```

#### Step **two (2)** processing functions

`epid_pid_sum()`, `epid_pi_sum()`.

```c
/*
Do processing as a Type-C PID controller and update the calculated
control variable (`y[k]`) in `epid_t` context.
`y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k]`
Note: If `y[k-1]` was FP NaN this will never result normal `y[k]` FP value,
so checking `epid_init*()` errors is recommended.

ctx: Pointer to the `epid_t` context.
out_min: Min output from controller.
out_max: Max output from controller.
*/
void
epid_pi_sum(epid_t *ctx, float out_min, float out_max);
```

```c
/*
Do processing as a Type-C PID controller and update the calculated
control variable (`y[k]`) in `epid_t` context.
`y[k] = y[k-1] + delta[k] = y[k-1] + P[k] + I[k] + D[k]`
Note: There is NO noise filtering on the derivative-term (`D[k]`).
Note: If `y[k-1]` was FP NaN this will never result normal `y[k]` FP value,
so checking `epid_init*()` errors is recommended.

ctx: Pointer to the `epid_t` context.
out_min: Min output from controller.
out_max: Max output from controller.
*/
void
epid_pid_sum(epid_t *ctx, float out_min, float out_max);
```

### Utilities and filters

`epid_util_ilim()`: I-term anti-windup.

```c
/*
Limit I-term `I[k]` value to boundaries as an integrator anti-windup.
Use this function after `epid_pi*_calc()`.

ctx: Pointer to the `epid_t` context.
i_min: Min `I[k]` value.
i_max: Max `I[k]` value.
*/
void
epid_util_ilim(epid_t *ctx, float i_min, float i_max);
```

General infinite-impulse-response (IIR) single-pole low-pass filter (LPF),
an exponentially weighted moving average (EMA).
This can be used for D-term filtering for example.

```c
/*
Initialize or reset a `epid_lpf_t` context.
Infinite-impulse-response (IIR) single-pole low-pass filter (LPF),
an exponentially weighted moving average (EMA).

Use this function once before `epid_util_lpf_calc()`.
smoothing_factor can be defined as: `a = (2PI*dT*f_cut)/(2PI*dT*f_cut + 1)`.

ctx: Pointer to the `epid_lpf_t` context.
smoothing_factor: Filter's smoothing factor. `0 < a < 1`.
x_0: Input `x[0]` value.

Return:
  - `EPID_ERR_NONE` on success.
  - `EPID_ERR_INIT` if initialization error occurred.
  - `EPID_ERR_FLT` if floating-point arithmetic error occurred.
*/
epid_info_t
epid_util_lpf_init(epid_lpf_t *ctx, float smoothing_factor, float x_0);
```

```c
/*
Apply an IIR LPF to an input `x[x]`.
Infinite-impulse-response (IIR) single-pole low-pass filter (LPF),
an exponentially weighted moving average (EMA).
`y[k] = FILTER(x[k]) = y[k-1] + smoothing_factor * (x[k] - y[k-1])`

Use this function after `epid_pid_calc()` to apply a D-term low-pass filter.

ctx: Pointer to the `epid_lpf_t` context.
input: Input `x[x]` value to filter.
*/
void
epid_util_lpf_calc(epid_lpf_t *ctx, float input);
```

---

## Code examples

- Arduino IDE example: `examples/Basic_API/Basic_API.ino`.
- Testing code: `extras/testing/`.

---

## Software repository

GitHub repository: <https://github.com/abderraouf-adjal/Embedded-PID>

Bitbucket repository: <https://bitbucket.org/abderraouf-adjal/embedded-pid>

---

## Copyright and licensing

**File:**  `LICENSE.txt`.

*[\*]*:
D. M. Auslander, Y. Takahashi and M. Tomizuka, "Direct digital process
control: Practice and algorithms for microprocessor application,"
in Proceedings of the IEEE, vol. 66, no. 2, pp. 199-208, Feb. 1978,
doi: 10.1109/PROC.1978.10870.
