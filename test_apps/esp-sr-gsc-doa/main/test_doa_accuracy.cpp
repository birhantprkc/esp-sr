/*
 * DOA Capon Embedded Accuracy Test for ESP32-P4
 *
 * Dataset: data_4mic_r5cm_quite (simulated 4-mic uniform circular array,
 * radius 5cm, clean speech, sources on a 2m circle at angles 0..330 deg
 * in 30 deg steps, angle measured counter-clockwise from the +x axis).
 *
 * For each of the 12 angles the first EVAL_FRAMES frames are fed to
 * esp_doa_capon_embedded (VAD forced to speech). The first WARMUP_FRAMES
 * frames are skipped (covariance recursion convergence), the rest are
 * compared against the true angle: exact match and within one grid step
 * (+-10 deg).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"

#include "esp_doa_capon_embedded.h"
#include "doa_test_data.h"

#define MIC_NUM         4
#define MIC_RADIUS      0.05f   /* 5cm circular array radius (matches dataset) */
#define FRAME_LEN       128
#define EVAL_FRAMES     64      /* frames available per angle in doa_test_data.h */
#define WARMUP_FRAMES   10      /* skipped for Rxx convergence */

/* Mic i sits at i*90 degrees, matching the dataset channel order */
static PlaneCoord g_mic_coords[MIC_NUM] = {
    { MIC_RADIUS, 0.0f, 0.0f},
    {0.0f,  MIC_RADIUS, 0.0f},
    {-MIC_RADIUS, 0.0f, 0.0f},
    {0.0f, -MIC_RADIUS, 0.0f},
};

typedef struct {
    int angle_deg;
    const int16_t *data;    /* EVAL_FRAMES frames, interleaved 4ch (converted to planar at runtime) */
} doa_case_t;

static const doa_case_t g_cases[] = {
    {  0, doa_data_angle_000},
    { 30, doa_data_angle_030},
    { 60, doa_data_angle_060},
    { 90, doa_data_angle_090},
    {120, doa_data_angle_120},
    {150, doa_data_angle_150},
    {180, doa_data_angle_180},
    {210, doa_data_angle_210},
    {240, doa_data_angle_240},
    {270, doa_data_angle_270},
    {300, doa_data_angle_300},
    {330, doa_data_angle_330},
};
#define NUM_CASES (sizeof(g_cases) / sizeof(g_cases[0]))

/* Circular absolute difference of two angles in degrees, range 0..180 */
static int angle_diff(int a, int b)
{
    int d = abs(a - b) % 360;
    return (d > 180) ? 360 - d : d;
}

/* Convert one interleaved frame to the planar layout the DOA API expects */
static void deinterleave_frame(const int16_t *inter, int16_t *planar)
{
    for (int n = 0; n < FRAME_LEN; n++) {
        for (int ch = 0; ch < MIC_NUM; ch++) {
            planar[ch * FRAME_LEN + n] = inter[n * MIC_NUM + ch];
        }
    }
}

TEST_CASE("doa capon embedded accuracy", "[gsc_doa]")
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("\n******************************************************\n");
    printf("  ESP32-P4 DOA Capon Embedded Accuracy Test\n");
    printf("  Dataset: data_4mic_r5cm_quite, %d angles x %d frames\n",
           (int)NUM_CASES, EVAL_FRAMES);
    printf("******************************************************\n");

    const int eval_per_case = EVAL_FRAMES - WARMUP_FRAMES;
    int total_exact = 0, total_within10 = 0;
    int invalid_angles = 0;
    static int16_t planar_frame[FRAME_LEN * MIC_NUM];

    for (int c = 0; c < (int)NUM_CASES; c++) {
        esp_doa_capon_embedded_handle_t *doa =
            esp_doa_capon_embedded_create(g_mic_coords, MIC_NUM);
        TEST_ASSERT_NOT_NULL(doa);

        int exact = 0, within10 = 0;
        for (int f = 0; f < EVAL_FRAMES; f++) {
            const int16_t *frame = g_cases[c].data + f * FRAME_LEN * MIC_NUM;
            deinterleave_frame(frame, planar_frame);
            float raw = esp_doa_capon_embedded_process(doa, planar_frame, 1);
            if (!(raw >= 0.0f && raw < 360.0f)) {
                invalid_angles++;
            }
            int est = (int)lroundf(raw);
            if (f < WARMUP_FRAMES) {
                continue;
            }
            int d = angle_diff(est, g_cases[c].angle_deg);
            if (d == 0) exact++;
            if (d <= 10) within10++;
        }
        esp_doa_capon_embedded_destroy(doa);

        total_exact += exact;
        total_within10 += within10;
        printf("  angle %3d deg: exact %2d/%d (%5.1f%%), within +/-10 deg %2d/%d (%5.1f%%)\n",
               g_cases[c].angle_deg,
               exact, eval_per_case, 100.0f * exact / eval_per_case,
               within10, eval_per_case, 100.0f * within10 / eval_per_case);
    }

    int total = eval_per_case * (int)NUM_CASES;
    printf("\n======================================================\n");
    printf("  DOA ACCURACY SUMMARY (warmup %d frames skipped)\n", WARMUP_FRAMES);
    printf("  exact match:   %d/%d (%.1f%%)\n",
           total_exact, total, 100.0f * total_exact / total);
    printf("  within 10 deg: %d/%d (%.1f%%)\n",
           total_within10, total, 100.0f * total_within10 / total);
    printf("======================================================\n");

    /* The estimated angle must always be a valid direction in [0, 360) */
    TEST_ASSERT_EQUAL(0, invalid_angles);
}
