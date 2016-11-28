
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
#define ARM_CMSIS_SQRT
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

//#define MEXH_MAX_HEIGHT_S_0_20 80.1072388f
#define MEXH_HEIGHT_SCALE_FACTOR 0.8672f

/* MEXH_HEIGHT_SCALE_FACTOR divided by MEXH_MAX_HEIGHT */
#define MEXH_HEIGHT_FACTOR_S_0_04 (0.010825489f)
#define MEXH_HEIGHT_FACTOR_S_0_08 (0.015309553f)
#define MEXH_HEIGHT_FACTOR_S_0_12 (0.018750298f)
#define MEXH_HEIGHT_FACTOR_S_0_16 (0.021650977f)
#define MEXH_HEIGHT_FACTOR_S_0_20 (0.024206531f)

#define S_0_04 (0.04f)
#define S_0_08 (0.08f)
#define S_0_12 (0.12f)
#define S_0_16 (0.16f)
#define S_0_20 (0.20f)

#define MEXH_WIDTH_SCALE_FACTOR 0.1538f
#define MEXH_WIDTH MEXH_WIDTH_SCALE_FACTOR

#define MAX_SCALE 0.2f

//#define TIME_MEAS_PIN (1u << 19u)
//#define TIME_MEAS_PIN_SHIFT 19u
   
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

#define LOG_MEXH_CSV  (0)
#define LOG_XCORR_CSV (1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

float32_t mexh_generate_0_20(uint16_t i, uint16_t n);
float32_t mexh_generate_0_16(uint16_t i, uint16_t n);
float32_t mexh_generate_0_12(uint16_t i, uint16_t n);
float32_t mexh_generate_0_08(uint16_t i, uint16_t n);
float32_t mexh_generate_0_04(uint16_t i, uint16_t n);

void xcorrf_less_ram(float32_t *c, int * a, uint16_t n, uint32_t lag, float32_t (*b)(uint16_t, uint16_t));

float32_t (* mexh_lut[])(uint16_t, uint16_t) = {
		mexh_generate_0_20, \
		mexh_generate_0_16, \
		mexh_generate_0_12, \
		mexh_generate_0_08, \
		mexh_generate_0_04 };

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

    float32_t res[N+1]; /* xcorr result, N+1 because lag = N/2, and N+1 = 2*lag+1 */
    uint32_t lag = N/2;
    float32_t temp = 0;
    float32_t max = 0;

    int *wave;
    
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };
    
    wave = getWaveform();
    
    /* Init output GPIO. */
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);
    GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN); /* Used for benchmarking, toggle pin to start */

#if LOG_MEXH_CSV
    for ( i = 0; i < 5; i++)
    {
		for ( j = 0; j < N; j++ )
		{
			PRINTF("%d, %d, %.8f,\r\n", i, j, mexh_lut[i](j, N));
		}
    }
#endif

    for ( i = 0; i < 5; i++)
    {
    	xcorrf_less_ram(res, wave, N, lag, mexh_lut[i]);

    	for( j = 0; j < N+1; j++)
    	{
        	PRINTF("%d, %d, %.4f\r\n", i, j, res[j]);
    	}
    	PRINTF("\r\n");
    }

#if 0

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
#endif

    OSTaskSuspend(NULL, &err);
    if (err != OS_ERR_NONE)
    {
    	PRINTF("Error.");
    }
}

/*******************************************************************************
* function: xcorrf_less_ram() - performs cross-correlation with floating point input
 * @param a - waveform data
 * @param b - function pointer, returns a single mexh element
 * @param size - length of a and b
 * @param lag - how much to lag in the cross-correlation
 ******************************************************************************/
void xcorrf_less_ram(float32_t *c, int * a, uint16_t n, uint32_t lag, float32_t (*b)(uint16_t, uint16_t))
{
    uint16_t i;
    uint32_t nLags = 2*lag+1; // total number of shifts or lags
    uint32_t multsPerLag;     // multiplications for a given offset/lag position

    if(c != NULL)
    {
        for(i=0;i<nLags;i++)
        {
            c[i] = 0; // zero array
        }
        for(i=0;i<nLags;i++)
        {
            multsPerLag = n - abs(lag-i);

            for(int j=0;j<multsPerLag;j++)
            {
                if(i <= lag)
                {
                    c[i] = c[i] + ( (float) a[j] * b(j + lag - i, n));
                }
                else
                {
                    c[i] = c[i] + ( (float) a[j - lag + i] * b(j,n) );
                }
            }
        }
    }
}

/*******************************************************************************
* function: mexh_generate_0_20() - Dynamically compute non-zero elements of mexh for scale 0.20
 * @param i - mexh data index to generate
 * @param n - Number of data points.
 ******************************************************************************/
float32_t mexh_generate_0_20(uint16_t i, uint16_t n)
{
    float32_t m;
    float32_t sqrt_res;
    float32_t temp;

    //uint16_t highLim = n/2 + (n/10) * s/MAX_SCALE;
    //uint16_t lowLim = n/2 - (n/10) * s/MAX_SCALE;

    assert(i < n);

    //if(i < lowLim || i > highLim)

    /* This seems to be a decent equation for omitting the zero elements
     * in the mexh for different scales and N points */
    if( (i < n/2 - (n/10) * S_0_20/MAX_SCALE) || (i > n/2 + (n/10) * S_0_20/MAX_SCALE))
    {
    	return 0;
    }
    else
    {
        /* Scale mexh from -1/2 to +1/2 */
        m = ((float)i / n) - 0.5f;

#ifdef ARM_CMSIS_SQRT
        /* Use CMSIS-DSP library */
        arm_sqrt_f32(S_0_20 * MEXH_WIDTH, &sqrt_res); // fast ARM core independent sqrt, uses FPU if avail
#else
        /* Use <math.h> */
        sqrt_res = sqrt(S_0_20 * MEXH_WIDTH);
#endif

        temp = 2.0f * PI * (1/sqrt_res) 						\
        		* (1 - 2*PI * powf((m/(S_0_20 * MEXH_WIDTH)),2.0f)) 	\
				* powf(E_NUMBER , -PI * powf((m/(S_0_20 * MEXH_WIDTH)),2.0f));

        return temp * MEXH_HEIGHT_FACTOR_S_0_20;
    }
}


float32_t mexh_generate_0_16(uint16_t i, uint16_t n)
{
    float32_t m;
    float32_t sqrt_res;
    float32_t temp;

    if( (i < n/2 - (n/10) * S_0_16/MAX_SCALE) || (i > n/2 + (n/10) * S_0_16/MAX_SCALE))
    {
    	return 0;
    }
    else
    {
        m = ( (float) i / n) - 0.5f;

#ifdef ARM_CMSIS_SQRT
        arm_sqrt_f32(S_0_16 * MEXH_WIDTH, &sqrt_res);
#else
        sqrt_res = sqrt(S_0_16 * MEXH_WIDTH);
#endif

        temp = 2.0f * PI * (1/sqrt_res) 						\
        		* (1 - 2*PI * powf((m/(S_0_16 * MEXH_WIDTH)),2.0f)) 	\
				* powf(E_NUMBER , -PI * powf((m/(S_0_16 * MEXH_WIDTH)),2.0f));

        return temp * MEXH_HEIGHT_FACTOR_S_0_16;
    }
}


float32_t mexh_generate_0_12(uint16_t i, uint16_t n)
{
    float32_t m;
    float32_t sqrt_res;
    float32_t temp;

    if( (i < n/2 - (n/10) * S_0_12/MAX_SCALE) || (i > n/2 + (n/10) * S_0_12/MAX_SCALE))
    {
    	return 0;
    }
    else
    {
        m = ( (float) i / n) - 0.5f;

#ifdef ARM_CMSIS_SQRT
        arm_sqrt_f32(S_0_12 * MEXH_WIDTH, &sqrt_res);
#else
        sqrt_res = sqrt(S_0_12 * MEXH_WIDTH);
#endif

        temp = 2.0f * PI * (1/sqrt_res) 						\
        		* (1 - 2*PI * powf((m/(S_0_12 * MEXH_WIDTH)),2.0f)) 	\
				* powf(E_NUMBER , -PI * powf((m/(S_0_12 * MEXH_WIDTH)),2.0f));

        return temp * MEXH_HEIGHT_FACTOR_S_0_12;
    }
}


float32_t mexh_generate_0_08(uint16_t i, uint16_t n)
{
    float32_t m;
    float32_t sqrt_res;
    float32_t temp;

    if( (i < n/2 - (n/10) * S_0_08/MAX_SCALE) || (i > n/2 + (n/10) * S_0_08/MAX_SCALE))
    {
    	return 0;
    }
    else
    {
        m = ( (float) i / n) - 0.5f;

#ifdef ARM_CMSIS_SQRT
        arm_sqrt_f32(S_0_08 * MEXH_WIDTH, &sqrt_res);
#else
        sqrt_res = sqrt(S_0_08 * MEXH_WIDTH);
#endif

        temp = 2.0f * PI * (1/sqrt_res) 						\
        		* (1 - 2*PI * powf((m/(S_0_08 * MEXH_WIDTH)),2.0f)) 	\
				* powf(E_NUMBER , -PI * powf((m/(S_0_08 * MEXH_WIDTH)),2.0f));

        return temp * MEXH_HEIGHT_FACTOR_S_0_08;
    }
}

float32_t mexh_generate_0_04(uint16_t i, uint16_t n)
{
    float32_t m;
    float32_t sqrt_res;
    float32_t temp;

    if( (i < n/2 - (n/10) * S_0_04/MAX_SCALE) || (i > n/2 + (n/10) * S_0_04/MAX_SCALE))
    {
    	return 0;
    }
    else
    {
        m = ( (float) i / n) - 0.5f;

#ifdef ARM_CMSIS_SQRT
        arm_sqrt_f32(S_0_04 * MEXH_WIDTH, &sqrt_res);
#else
        sqrt_res = sqrt(S_0_04 * MEXH_WIDTH);
#endif

        temp = 2.0f * PI * (1/sqrt_res) 						\
        		* (1 - 2*PI * powf((m/(S_0_04 * MEXH_WIDTH)),2.0f)) 	\
				* powf(E_NUMBER , -PI * powf((m/(S_0_04 * MEXH_WIDTH)),2.0f));

        return temp * MEXH_HEIGHT_FACTOR_S_0_04;
    }
}
/* EOF */
