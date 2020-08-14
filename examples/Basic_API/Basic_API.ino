/**
 * Try Embedded Type-C PID library API.
 * 
 * The circuit:  No external hardware needed.
 * by Abderraouf Adjal.
 * This example code is in the public domain.
 */

/* Library documentation: <pid.h> */

#include <pid.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3 /* Uno PWM pin */

/* Controller parameters */
#define SETPOINT 512.0f /* Setpoint from ADC. */

#define EPID_KP  10.0f
#define EPID_KI  10.0f
#define EPID_KD  10.0f

#define PID_LIM_MIN 0.0f /* Limit for PWM. */
#define PID_LIM_MAX 255.0f /* Limit for PWM. */

#define DEADBAND 0.0f /* Off==0 */

epid_t pid_ctx;

//Define Variables we'll be connecting to
float Input;
int Output = 0;
float deadband_delta;

void setup()
{
    /* Initialize serial and wait for port to open */
    Serial.begin(9600);
    while (!Serial) { ; }

    pinMode(PIN_OUTPUT, OUTPUT);
    analogWrite(PIN_OUTPUT, Output);
    Input = analogRead(PIN_INPUT);
  
    /* Initialize PID controller */
    epid_info_t epid_err = epid_init(&pid_ctx,
        Input, Input, PID_LIM_MIN,
        EPID_KP, EPID_KI, EPID_KD);

    if (epid_err != EPID_ERR_NONE) {
        Serial.print("\n\n** ERROR: epid_err != EPID_ERR_NONE **\n\n");
        while (1) { ; }
    }
}

void loop()
{
    Input = analogRead(PIN_INPUT);

    epid_pid_calc(&pid_ctx, SETPOINT, Input); /* Calc PID terms values */

    /* Apply deadband filter to `delta[k]`. */
    deadband_delta = pid_ctx.p_term + pid_ctx.i_term + pid_ctx.d_term;
    if ((deadband_delta != deadband_delta) || (fabsf(deadband_delta) >= DEADBAND)) {
        /* Compute new control signal output */
        epid_pid_sum(&pid_ctx, PID_LIM_MIN, PID_LIM_MAX);
        Output = (int)lroundf(pid_ctx.y_out); /* float to int */
    }

    analogWrite(PIN_OUTPUT, Output);

    delay(500);
}
