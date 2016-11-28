
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Kernel */
#include "os.h"

/* Freescale */
#include "board.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

/* CMSIS-DSP */
#include <arm_math.h>

/* GCC */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

//extern int waveform[];
   
extern int *getWaveform();

#define USE_BUILTIN_MATH
//#define ARM_CMSIS_MAX
//#define SIMPLE_TEST

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifdef SIMPLE_TEST
#define N 4
#else
#define N 512
#endif

#define E_NUMBER 2.71828f
#define MEXH_HEIGHT_SCALE_FACTOR 0.8672f
#define MEXH_WIDTH_SCALE_FACTOR 0.1538f

//#define TIME_MEAS_PIN (1u << 19u)
//#define TIME_MEAS_PIN_SHIFT 19u
   
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

#define LOG_MEXH_CSV  (0)
#define LOG_XCORR_CSV (1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void mexh(float32_t *mh, float32_t s, uint16_t sz);
void xcorrf(float32_t *c, int *a, float32_t *b, uint32_t size, uint32_t lag);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
void cwt_task(void *p_arg)
{
	OS_ERR err;

	uint16_t maxIndexStack[3] = {0};
    uint32_t i,j;

    //float32_t res[N]; /*xcorr result*/
    float32_t CC[5][N+1]; /* xcorr result, N+1 because lag = N/2, and N+1 = 2*lag+1 */

#ifdef SIMPLE_TEST
    uint32_t lag = N;
    float32_t b[] = {0.1,0.2,0.3,0.4};
    int *a;
#else
    uint32_t lag = N/2;
    float32_t temp = 0;
    float32_t max = 0;;
    float32_t scale;
    float32_t scaleInit = 0.04;
    float32_t scaleStep = 0.04; /* 0.04, 0.08, 0.12, 0.16, 0.2 */
    float32_t scaleMax = 0.2;
    float32_t mh[N];  /*mexh*/
    int *wave;
#endif

    
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };
    
#ifdef SIMPLE_TEST
    a = getWaveform();
#else
    wave = getWaveform();
#endif
    
    /* Init output GPIO. */
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);
    GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN); /* Used for benchmarking, toggle pin to start */
    
    i = 0; // here 'i' is row index of 2D CC array
    
#if 0// LOG_MEXH_CSV /* Print out mexh */
    mexh(mh, 0.10f, N);
#endif
    
#if LOG_MEXH_CSV /* Print out mexh */
    //for ( i = 0; i < N; i++ ) { PRINTF("mh[%d]=%.4f,\r\n", i, mh[i]); }
    for ( i = 0; i < N; i++ ) { PRINTF("%d, %.8f,\r\n",i,mh[i]); }
#endif

#ifdef SIMPLE_TEST
    xcorrf(res, a, b, N, lag);
#else

	for(scale = scaleInit; scale <= scaleMax; scale+=scaleStep)
	{
		mexh(mh, scale, N);
		//xcorrf(res, wave, mh, N, lag);
		xcorrf(CC[i],wave,mh, N,lag);
		i++;
	}
#endif
    
    GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN); /* Used for benchmarking, toggle pin when done */

#if LOG_XCORR_CSV

    /* Print entire row at once*/
    for ( j = 0; j < N+1; j++ )
    {
    	for ( i = 0; i < 5; i++ )
    	{
    		temp = powf(CC[i][j], 2.0f);
    		PRINTF("%f,", temp);

    		if(temp > max)
    		{
    			max = temp;

    			/* Push new index to a stack */
    			maxIndexStack[2] = maxIndexStack[1];
    			maxIndexStack[1] = maxIndexStack[0];
    			maxIndexStack[0] = j; //i*N + j;
    		}
    	}
    	PRINTF("\r\n");
    }
    PRINTF("INDICES\r\n");
    for ( i = 0; i < 3; i++ ) { PRINTF("%d,\r\n", maxIndexStack[i]); }

    //for ( i = 0; i < N+1; i++ ) { PRINTF("%.4f,\r\n", CC[0][i]); }
    //for ( i = 0; i < N+1; i++ ) { PRINTF("%f,\r\n", powf(res[i], 2.0f)); }
    //for ( i = 0; i < N+1; i++ ) { PRINTF("%d,%f,\r\n", i, res[i]); }
    //for ( i = 0; i < N+1; i++ ) { PRINTF("%d,%.8f,\r\n", i, powf(res[i], 2.0f)); }
#endif

    OSTaskSuspend(NULL, &err);
    if (err != OS_ERR_NONE)
    {
    	PRINTF("Error.");
    }
}

/*******************************************************************************
* function: mexh() - Populate array with mexican hat wavelet.
 * @param mh - preallocated float array populated with mexican hat wavelet.
 * @param s - wavelet scaling factor.
 * @param sx - Number of data points.
 ******************************************************************************/
void mexh(float32_t *mh, float32_t s, uint16_t sz)
{
    uint16_t i;
#ifdef ARM_CMSIS_MAX
    uint32_t indexMax;
#endif
    float32_t m;
    float32_t w = MEXH_WIDTH_SCALE_FACTOR;
    float32_t f_res_cmsis_dsp;
    float32_t mhMax = 0;

    for(i=0;i<sz;i++)
    {
        /* Scale mexh from -1/2 to +1/2 */
        m = ((float)i / sz) - 0.5f;

        /* Use CMSIS-DSP library */
        arm_sqrt_f32(s * w, &f_res_cmsis_dsp); // fast ARM core independent sqrt, uses FPU if avail

        mh[i] = 2.0f*PI * (1/f_res_cmsis_dsp) * (1 - 2*PI * powf((m/(s*w)),2.0f)) * powf( E_NUMBER , -PI * powf((m/(s*w)),2.0f) );

#ifndef ARM_CMSIS_MAX /* Calculate max manually */
        if(mh[i] > mhMax) { mhMax = mh[i]; }
#endif
    }

#ifdef ARM_CMSIS_MAX
    arm_max_f32(mh, sz, &mhMax, &indexMax);
#endif

    for(i=0;i<sz;i++)
    {
        mh[i] = mh[i] / mhMax;
        mh[i] = mh[i] * MEXH_HEIGHT_SCALE_FACTOR;
    }
}

/*******************************************************************************
* function: xcorrf() - performs cross-correlation with floating point input
 * @param a
 * @param b
 * @param size - length of a and b
 * @param lag
 ******************************************************************************/
void xcorrf(float32_t *c, int *a, float32_t *b, uint32_t size, uint32_t lag)
{
    uint32_t i;
    uint32_t nLags = 2*lag+1; // total number of shifts or lags
    uint32_t multsPerLag;     // multiplications for a given offset/lag position

    float32_t a_temp;
    float32_t b_temp;

    if(c != NULL)
    {
        for(i=0;i<nLags;i++)
        {
            c[i] = 0; // zero array
        }
        for(i=0;i<nLags;i++)
        {
            multsPerLag = size - abs(lag-i);

            for(int j=0;j<multsPerLag;j++)
            {
                if(i <= lag)
                {
                	a_temp = (float32_t) a[j];
                	b_temp = b[j + lag - i];
                	c[i] = c[i] + (a_temp * b_temp);
                }
                else
                {
                	a_temp = (float32_t) a[j - lag + i];
                	b_temp = b[j];
                	c[i] = c[i] + (a_temp * b_temp);
                }
            }
        }
    }
}

#if 0
/*******************************************************************************
* function: xcorr() - performs cross-correlation.
 * @param a
 * @param b
 * @param size - length of a and b
 * @param lag
 ******************************************************************************/
#ifdef RETURN_ARR
float * xcorr(int *a, int *b, uint32_t size, uint32_t lag)
#else
void xcorr(float *c, int *a, int *b, uint32_t size, uint32_t lag)
#endif
{
    uint32_t i;
    uint32_t m = size;
    uint32_t nLags = 2*lag+1; // total number of shifts or lags
    uint32_t multsPerLag;     // multiplications for a given offset/lag position

#ifdef RETURN_ARR
    static float *c;
    c = malloc(nLags*sizeof(float)); // Create array to hold result of xcorr
#endif

    if(c != NULL)
    {

#ifdef INIT_ARRAY
        for(i=0;i<nLags;i++)
        {
            c[i] = 0; // zero array
        }
#endif

        for(i=0;i<nLags;i++)
        {
            multsPerLag = m - abs(lag-i);

            for(int j=0;j<multsPerLag;j++)
            {
                if(i <= lag)
                {
                    c[i] = c[i] + ( (float) a[j] * b[j + lag - i] );
                }
                else
                {
                    c[i] = c[i] + ( (float) a[j - lag + i] * b[j] );
                }
            }
        }
    }
#ifdef RETURN_ARR
    return c;
#endif
}
#endif
