/** \file maxim_algorithm.c ***************************************************
*
* Project: MAXREFDES117# (adaptacja)
* Filename: maxim_algorithm.c
* Description: This module calculates the heart rate/SpO2 level
*              (adapted for Zephyr / nRF52 environment)
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#include "maxim_algorithm.h"
#include <stdint.h>

#define FreqS 25
#define BUFFER_SIZE (FreqS * 4)   /* 100 próbek */
#define MA4_SIZE 4
#define min(x,y) ((x) < (y) ? (x) : (y))

/* Ustawienia pod nadgarstek */
#define HR_MAX_BPM       180      /* do min_dist w 1. przebiegu (bezpieczne) */
#define MIN_DIST_FRAC_N  6        /* 2. przebieg: min_dist ~= 0.7 * okresu */
#define MIN_DIST_FRAC_D  10

static const uint8_t uch_spo2_table[184] = {
    95,95,95,96,96,96,97,97,97,97,97,98,98,98,98,98,99,99,99,99,
    99,99,99,99,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,99,99,99,99,99,99,99,99,98,98,98,98,98,98,97,97,
    97,97,96,96,96,96,95,95,95,94,94,94,93,93,93,92,92,92,91,91,
    90,90,89,89,89,88,88,87,87,86,86,85,85,84,84,83,82,82,81,81,
    80,80,79,78,78,77,76,76,75,74,74,73,72,72,71,70,69,69,68,67,
    66,66,65,64,63,62,62,61,60,59,58,57,56,56,55,54,53,52,51,50,
    49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,31,30,29,
    28,27,26,25,23,22,21,20,19,17,16,15,14,12,11,10,9,7,6,5,
    3,2,1
};

static int32_t an_x[BUFFER_SIZE]; /* IR */
static int32_t an_y[BUFFER_SIZE]; /* RED */

/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
static void maxim_sort_ascend(int32_t *x, int32_t n)
{
    for (int i = 1; i < n; i++) {
        int32_t t = x[i], j = i;
        while (j > 0 && t < x[j - 1]) {
            x[j] = x[j - 1];
            j--;
        }
        x[j] = t;
    }
}

/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/
static void maxim_sort_indices_descend(int32_t *x, int32_t *idx, int32_t n)
{
    for (int i = 1; i < n; i++) {
        int32_t t = idx[i], j = i;
        while (j > 0 && x[t] > x[idx[j - 1]]) {
            idx[j] = idx[j - 1];
            j--;
        }
        idx[j] = t;
    }
}

/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
static void maxim_peaks_above_min_height(int32_t *locs, int32_t *npks,
                                         int32_t *x, int32_t n, int32_t min_h)
{
    *npks = 0;
    int32_t i = 1;

    while (i < n - 1) {
        if (x[i] > min_h && x[i] > x[i - 1]) {
            int32_t w = 1;
            while ((i + w) < n && x[i] == x[i + w]) {
                w++;
            }
            /* guard: i+w może == n */
            if ((i + w) < n && x[i] > x[i + w] && *npks < 15) {
                locs[(*npks)++] = i;
                i += (w + 1);
            } else {
                i += w;
            }
        } else {
            i++;
        }
    }
}

/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
static void maxim_remove_close_peaks(int32_t *locs, int32_t *npks,
                                     int32_t *x, int32_t min_dist)
{
    int old_npks;
    maxim_sort_indices_descend(x, locs, *npks);

    for (int i = -1; i < *npks; i++) {
        old_npks = *npks;
        *npks = i + 1;

        for (int j = i + 1; j < old_npks; j++) {
            int d = locs[j] - (i == -1 ? -1 : locs[i]);
            if (d > min_dist || d < -min_dist) {
                locs[(*npks)++] = locs[j];
            }
        }
    }
    maxim_sort_ascend(locs, *npks);
}

/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
static void maxim_find_peaks(int32_t *locs, int32_t *npks, int32_t *x, int32_t n,
                             int32_t min_h, int32_t min_dist, int32_t max_num)
{
    maxim_peaks_above_min_height(locs, npks, x, n, min_h);
    maxim_remove_close_peaks(locs, npks, x, min_dist);
    *npks = min(*npks, max_num);
}

/**
* \brief        Compute heart rate from median peak intervals
* \par          Details
*               Uses median of successive peak-to-peak distances (in samples),
*               then converts to BPM using sampling rate FreqS.
*
* \retval       Heart rate in BPM, or -999 on failure
*/
static int32_t hr_from_median_intervals(const int32_t *locs, int32_t npks)
{
    if (npks < 2) {
        return -999;
    }

    int32_t d[14];
    int32_t m = 0;

    for (int32_t k = 1; k < npks; k++) {
        d[m++] = (locs[k] - locs[k - 1]);
        if (m >= (int32_t)(sizeof(d) / sizeof(d[0]))) {
            break;
        }
    }

    if (m < 1) {
        return -999;
    }

    maxim_sort_ascend(d, m);
    int32_t med = (m % 2) ? d[m / 2] : (d[m / 2 - 1] + d[m / 2]) / 2;
    if (med <= 0) {
        return -999;
    }

    return (int32_t)((FreqS * 60) / med);
}

/**
* \brief        Calculate the heart rate and SpO2 level
* \par          Details
*               By detecting peaks (valleys after inversion) of PPG cycle and corresponding AC/DC
*               of red/infra-red signal, the ratio for the SPO2 is computed and mapped through a LUT.
*
* \param[in]    *ir_buf               - IR sensor data buffer
* \param[in]    n                    - IR/RED buffer length (expected BUFFER_SIZE)
* \param[in]    *red_buf              - Red sensor data buffer
* \param[out]   *spo2                 - Calculated SpO2 value
* \param[out]   *spo2_valid           - 1 if the calculated SpO2 value is valid
* \param[out]   *hr                   - Calculated heart rate value
* \param[out]   *hr_valid             - 1 if the calculated heart rate value is valid
*
* \retval       None
*/
void maxim_heart_rate_and_oxygen_saturation(
    uint32_t *ir_buf, int32_t n, uint32_t *red_buf,
    int32_t *spo2, int8_t *spo2_valid, int32_t *hr, int8_t *hr_valid)
{
    /* Zakładamy, że i tak wołasz z n==BUFFER_SIZE */
    (void)n;

    /* --- IR mean --- */
    uint32_t ir_mean = 0;
    for (int k = 0; k < BUFFER_SIZE; k++) {
        ir_mean += ir_buf[k];
    }
    ir_mean /= BUFFER_SIZE;

    /* remove DC and invert signal so that we can use peak detector as valley detector */
    for (int k = 0; k < BUFFER_SIZE; k++) {
        an_x[k] = -(int32_t)(ir_buf[k] - ir_mean);
    }

    /* 4 pt Moving Average */
    for (int k = 0; k < BUFFER_SIZE - MA4_SIZE; k++) {
        an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / 4;
    }

    /* calculate threshold */
    int32_t th = 0;
    for (int k = 0; k < BUFFER_SIZE; k++) {
        th += an_x[k];
    }
    th /= BUFFER_SIZE;

    if (th < 30) th = 30;
    if (th > 60) th = 60;

    int32_t valley_locs[15] = {0};
    int32_t npks = 0;

    /* --- Pass 1: loose min_dist (based on HR_MAX_BPM) --- */
    int32_t min_dist1 = (FreqS * 60) / HR_MAX_BPM;
    if (min_dist1 < 4) min_dist1 = 4;

    maxim_find_peaks(valley_locs, &npks, an_x, BUFFER_SIZE, th, min_dist1, 15);

    int32_t hr1 = -999;
    if (npks >= 2) {
        hr1 = hr_from_median_intervals(valley_locs, npks);
    }

    /* --- Pass 2: tighten min_dist based on estimated period --- */
    if (hr1 > 0) {
        int32_t period = (FreqS * 60) / hr1;
        int32_t min_dist2 = (period * MIN_DIST_FRAC_N) / MIN_DIST_FRAC_D;
        if (min_dist2 < min_dist1) min_dist2 = min_dist1;

        maxim_find_peaks(valley_locs, &npks, an_x, BUFFER_SIZE, th, min_dist2, 15);
    }

    /* HR final (median intervals) */
    if (npks >= 2) {
        *hr = hr_from_median_intervals(valley_locs, npks);
        *hr_valid = (*hr > 0) ? 1 : 0;
        if (!(*hr_valid)) {
            *hr = -999;
        }
    } else {
        *hr = -999;
        *hr_valid = 0;
    }

    /* --- SpO2 calculation (ratio-of-ratios + LUT) --- */
    for (int k = 0; k < BUFFER_SIZE; k++) {
        an_x[k] = (int32_t)ir_buf[k];
        an_y[k] = (int32_t)red_buf[k];
    }

    int32_t ratio[5] = {0};
    int32_t ratio_cnt = 0;

    for (int k = 0; k < npks - 1 && ratio_cnt < 5; k++) {
        if (valley_locs[k + 1] - valley_locs[k] <= 3) {
            continue;
        }

        int32_t x_dc_max = -16777216, y_dc_max = -16777216;
        int32_t x_dc_max_i = 0,       y_dc_max_i = 0;

        for (int i = valley_locs[k]; i < valley_locs[k + 1]; i++) {
            if (an_x[i] > x_dc_max) { x_dc_max = an_x[i]; x_dc_max_i = i; }
            if (an_y[i] > y_dc_max) { y_dc_max = an_y[i]; y_dc_max_i = i; }
        }

        int32_t y_ac = (an_y[valley_locs[k + 1]] - an_y[valley_locs[k]]) * (y_dc_max_i - valley_locs[k]);
        y_ac = an_y[valley_locs[k]] + y_ac / (valley_locs[k + 1] - valley_locs[k]);
        y_ac = an_y[y_dc_max_i] - y_ac;

        int32_t x_ac = (an_x[valley_locs[k + 1]] - an_x[valley_locs[k]]) * (x_dc_max_i - valley_locs[k]);
        x_ac = an_x[valley_locs[k]] + x_ac / (valley_locs[k + 1] - valley_locs[k]);
        /* BUGFIX: correct index for IR AC */
        x_ac = an_x[x_dc_max_i] - x_ac;

        int32_t nume  = (y_ac * x_dc_max) >> 7;
        int32_t denom = (x_ac * y_dc_max) >> 7;
        if (denom > 0) {
            ratio[ratio_cnt++] = (nume * 100) / denom;
        }
    }

    if (ratio_cnt > 0) {
        maxim_sort_ascend(ratio, ratio_cnt);
        int mid = ratio_cnt / 2;
        int avg = (ratio_cnt % 2 == 0) ? (ratio[mid - 1] + ratio[mid]) / 2 : ratio[mid];

        if (avg > 2 && avg < 184) {
            *spo2 = (int32_t)uch_spo2_table[avg];
            *spo2_valid = 1;
        } else {
            *spo2 = -999;
            *spo2_valid = 0;
        }
    } else {
        *spo2 = -999;
        *spo2_valid = 0;
    }
}

