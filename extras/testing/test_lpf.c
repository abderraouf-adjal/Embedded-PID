/* ISO/IEC C standard: C99 (ISO/IEC 9899:1999) or later. */
/* gcc -std=c99 -Wall -Wextra test_lpf.c -lm -o test_lpf.bin */

#include <stdio.h>
#include <math.h>

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

#include "../src/pid.h"
#include "../src/pid.c"

#define SAMPLE_TIME_S 0.001 /* 2 times max noise freq. at least: 4x */
#define SAMPLES_N 250U /* About SAMPLE_TIME_S*SAMPLES_N = 0.25s */

#define SIG_FREQ 10.0
#define NOISE_FREQ 250.0
#define FREQ_CUTOFF 20.0

epid_lpf_t lpf;

float sig_ideal[SAMPLES_N];
float sig_in[SAMPLES_N];
float sig_out[SAMPLES_N];


void make_signal(void)
{
    for (size_t i=0; i < SAMPLES_N; i++) {
        /* Add signal */
        sig_ideal[i]  = 1.0f * sinf(2.0f*M_PI*SIG_FREQ * (SAMPLE_TIME_S*(float)(i)));
        sig_in[i]  = sig_ideal[i];
        /* Add noise */
        sig_in[i] += 0.2f * sinf(2.0f*M_PI*NOISE_FREQ * (SAMPLE_TIME_S*(float)(i)));
        sig_in[i] += 0.2f * sinf(2.0f*M_PI*(NOISE_FREQ/2.0) * (SAMPLE_TIME_S*(float)(i)));
    }
}

void filtre_signal(void)
{
    /* smoothing_factor can be defined as: `a = (2PI*dT*f_cut)/(2PI*dT*f_cut + 1)`. */
    const float smoothing_factor = (2.0f*M_PI*SAMPLE_TIME_S*FREQ_CUTOFF)/(2.0f*M_PI*SAMPLE_TIME_S*FREQ_CUTOFF + 1.0f);

    epid_info_t err = epid_util_lpf_init(&lpf, smoothing_factor, sig_in[0]);
    if (err != EPID_ERR_NONE) {
        fprintf(stderr, "epid_init*() error.\n");
        return ;
    }

    for (size_t i=0; i < SAMPLES_N; i++) {
        epid_util_lpf_calc(&lpf, sig_in[i]);
        sig_out[i] = lpf.y;
    }
}


int main()
{
    make_signal();
    filtre_signal();

    printf("Time (s)\tInput(t)\tOutput(t)\tIdeal(t)\n");
    for (size_t i=0; i < SAMPLES_N; i++) {
        printf("%f\t%f\t%f\t%f\n", SAMPLE_TIME_S*i, sig_in[i], sig_out[i], sig_ideal[i]);
    }

    return 0;
}
