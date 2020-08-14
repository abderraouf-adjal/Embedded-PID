/* ISO/IEC C standard: C99 (ISO/IEC 9899:1999) or later. */
/* gcc -std=c99 -Wall -Wextra main.c -o main */

#include <stdio.h>
#include <math.h>

#include "../src/pid.h"
#include "../src/pid.c"


/* Controller parameters */
#define EPID_KP  500.0f
#define EPID_KI  10.0f
#define EPID_KD  200.0f

#if 0
# define EPID_TI  0.0001f
# define EPID_TD  0.0001f
#endif

#define PID_LIM_MIN 0.0f
#define PID_LIM_MAX 500.0f /* Heater max power in W */

#define SAMPLE_TIME_S 0.1f
/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX (6.0*60.0)

#define DEADBAND 0.0f

epid_t c;

/* Simulate heating something, run every `Ts` */
float heating_system_temp_c = 20.0f; /* Sytem memory for measurement. */
void heating_system(float energy_watt)
{
    const float room_temp = 20.0f;
    const float specific_heat = 4.186f; /* Water: joule/gram °C */
    const float mass = 100.0f; /* mass in grams */
    const float surface = 6.0f*0.0025f; /* 6 faces of cube in meters^2 */
    /* Water to Air Heat Exchanger made in Mild Steel (5cm*5cm=0.0025m^2) */
    /* q [W/(m^2)] = (11.3 W/(m^2 K)) (temp_c-room_temp) */
    const float q = 11.3*(heating_system_temp_c-room_temp)*surface;
    float joules = - SAMPLE_TIME_S*(q); /* Get cold, energy out. */

    if (energy_watt > 0.0) {
        /* Add energy to heat. */
        joules += SAMPLE_TIME_S*(energy_watt);
    }

    heating_system_temp_c = heating_system_temp_c + (joules/(specific_heat*mass));
}


int main()
{
    epid_info_t epid_err;
    /* Initialize PID controller */
#ifdef EPID_TI
    epid_err = epid_init_T(&c,
        heating_system_temp_c, heating_system_temp_c, 0.0f
        EPID_KP, EPID_TI, EPID_TD,
        SAMPLE_TIME_S);
#else
    epid_err = epid_init(&c,
        heating_system_temp_c, heating_system_temp_c, 0.0f,
        EPID_KP, EPID_KI, EPID_KD);
#endif

    if (epid_err != EPID_ERR_NONE) {
        fprintf(stderr, "epid_init*() error.\n");
        return -1;
    }
    
    /* To simulate response using test system */
    float setpoint = 70.0f;
    float measurement;
    float deadband_delta;

    printf("Time (s)\tSystem Sensor (C)\tController Output (W)\tPID Delta\n");
    
    double t = 0.0;
    for (; t <= 100.0; t += SAMPLE_TIME_S) {
        measurement = heating_system_temp_c; /* Get measurement from system */
        epid_pid_calc(&c, setpoint, measurement); /* Calc PID terms values */
        /* Apply deadband filter to `delta[k]`. */
        deadband_delta = c.p_term + c.i_term + c.d_term;
        if ((isfinite(deadband_delta) == 0) || (fabsf(deadband_delta) >= DEADBAND)) {
            epid_pid_sum(&c, PID_LIM_MIN, PID_LIM_MAX); /* Compute new control signal output */
        }
        heating_system(c.y_out); /* Apply signal to the system */
        printf("%.2f\t%f\t%f\t%f\n", t, measurement, c.y_out, (c.p_term+c.i_term+c.d_term));
    }
    /* Simulate putting cold water in the hot container after X s. */
    heating_system_temp_c = heating_system_temp_c - 7.0f;

    for (; t <= 150.0; t += SAMPLE_TIME_S) {
        measurement = heating_system_temp_c; /* Get measurement from system */
        epid_pid_calc(&c, setpoint, measurement); /* Calc PID terms values */
        /* Apply deadband filter to `delta[k]`. */
        deadband_delta = c.p_term + c.i_term + c.d_term;
        if ((isfinite(deadband_delta) == 0) || (fabsf(deadband_delta) >= DEADBAND)) {
            epid_pid_sum(&c, PID_LIM_MIN, PID_LIM_MAX); /* Compute new control signal output */
        }
        heating_system(c.y_out); /* Apply signal to the system */
        printf("%.2f\t%f\t%f\t%f\n", t, measurement, c.y_out, (c.p_term+c.i_term+c.d_term));
    }
    /* Simulate setpoint change. */
    setpoint = setpoint + 7.0f;

    for (; t <= 220.0; t += SAMPLE_TIME_S) {
        measurement = heating_system_temp_c; /* Get measurement from system */
        epid_pid_calc(&c, setpoint, measurement); /* Calc PID terms values */
        /* Apply deadband filter to `delta[k]`. */
        deadband_delta = c.p_term + c.i_term + c.d_term;
        if ((isfinite(deadband_delta) == 0) || (fabsf(deadband_delta) >= DEADBAND)) {
            epid_pid_sum(&c, PID_LIM_MIN, PID_LIM_MAX); /* Compute new control signal output */
        }
        heating_system(c.y_out); /* Apply signal to the system */

        printf("%.2f\t%f\t%f\t%f\n", t, measurement, c.y_out, (c.p_term+c.i_term+c.d_term));
    }
    /* Simulate setpoint change. */
    setpoint = setpoint - 2.0f;

    for (; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S) {
        measurement = heating_system_temp_c; /* Get measurement from system */
        epid_pid_calc(&c, setpoint, measurement); /* Calc PID terms values */
        /* Apply deadband filter to `delta[k]`. */
        deadband_delta = c.p_term + c.i_term + c.d_term;
        if ((isfinite(deadband_delta) == 0) || (fabsf(deadband_delta) >= DEADBAND)) {
            epid_pid_sum(&c, PID_LIM_MIN, PID_LIM_MAX); /* Compute new control signal output */
        }
        heating_system(c.y_out); /* Apply signal to the system */

        printf("%.2f\t%f\t%f\t%f\n", t, measurement, c.y_out, (c.p_term+c.i_term+c.d_term));
    }

    return 0;
}
