/*
 * GSC/DOA sanity & memory leak tests for ESP32-P4
 *
 * Verifies that the esp_gsc and esp_doa_capon_embedded libraries run
 * correctly on chip:
 *   - repeated create/destroy leaves no memory leak
 *   - processing a synthetic multi-channel signal (a single sine source at a
 *     known direction) returns a valid DOA estimate close to the true angle
 *     and a non-silent GSC beamformed output
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "unity.h"

#include "esp_gsc.h"
#include "esp_doa_capon_embedded.h"

#define MIC_NUM           4
#define MIC_RADIUS        0.05f   /* 5cm circular array radius */
#define FRAME_LEN         128
#define SAMPLE_RATE       16000
#define SIGNAL_FREQ       2000.0f /* inside the DOA processing band (1500-4500 Hz) */
#define SOUND_SPEED       343.0f
#define SOURCE_ANGLE_DEG  0.0f
#define PROCESS_FRAMES    200

/* Mic i sits at i*90 degrees */
static PlaneCoord g_mic_coords[MIC_NUM] = {
    { MIC_RADIUS, 0.0f, 0.0f},
    {0.0f,  MIC_RADIUS, 0.0f},
    {-MIC_RADIUS, 0.0f, 0.0f},
    {0.0f, -MIC_RADIUS, 0.0f},
};

/*
 * Generate one frame of a far-field sine source at SOURCE_ANGLE_DEG.
 * For a plane wave arriving from direction phi, the microphone at
 * coordinate p_i receives the signal advanced by tau_i =
 * (x_i*cos(phi) + y_i*sin(phi)) / c relative to the array origin.
 * Fills both the interleaved layout (for DOA) and the planar layout (for GSC).
 */
static void gen_frame(int frame_idx, int16_t *planar, int16_t *inter)
{
    float phi = SOURCE_ANGLE_DEG * (float)M_PI / 180.0f;
    for (int ch = 0; ch < MIC_NUM; ch++) {
        float tau = (g_mic_coords[ch].x * cosf(phi) + g_mic_coords[ch].y * sinf(phi)) / SOUND_SPEED;
        for (int n = 0; n < FRAME_LEN; n++) {
            float t = (float)(frame_idx * FRAME_LEN + n) / SAMPLE_RATE;
            int16_t v = (int16_t)lrintf(10000.0f * sinf(2.0f * (float)M_PI * SIGNAL_FREQ * (t + tau)));
            planar[ch * FRAME_LEN + n] = v;
            inter[n * MIC_NUM + ch] = v;
        }
    }
}

/* Circular absolute difference of two angles in degrees, range 0..180 */
static int angle_diff(int a, int b)
{
    int d = abs(a - b) % 360;
    return (d > 180) ? 360 - d : d;
}

TEST_CASE("gsc create/destroy API & memory leak", "[gsc_doa]")
{
    vTaskDelay(500 / portTICK_PERIOD_MS);
    int start_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int start_internal_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    gsc_handle_t *gsc = esp_gsc_create(g_mic_coords, MIC_NUM);
    TEST_ASSERT_NOT_NULL(gsc);

    // test memory consumption
    int create_size = start_size - heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int create_internal_size = start_internal_size - heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    printf("Internal RAM: %d, PSRAM: %d\n", create_internal_size, create_size - create_internal_size);
    esp_gsc_destroy(gsc);

    // test memory leak
    int first_end_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int last_end_size = first_end_size;
    int mem_leak = start_size - last_end_size;
    printf("create&destroy times:%d, memory leak:%d\n", 1, mem_leak);

    for (int i = 0; i < 6; i++) {
        printf("create ...\n");
        gsc = esp_gsc_create(g_mic_coords, MIC_NUM);
        TEST_ASSERT_NOT_NULL(gsc);

        printf("destroy ...\n");
        esp_gsc_destroy(gsc);

        last_end_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        mem_leak = start_size - last_end_size;
        printf("create&destroy times:%d, memory leak:%d\n", i + 2, mem_leak);
    }

    TEST_ASSERT_EQUAL(true, (mem_leak) < 1000 && last_end_size == first_end_size);
}

TEST_CASE("doa create/destroy API & memory leak", "[gsc_doa]")
{
    vTaskDelay(500 / portTICK_PERIOD_MS);
    int start_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int start_internal_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    size_t mem_size = esp_doa_capon_embedded_get_mem_size(MIC_NUM);
    void *mem_pool = heap_caps_malloc(mem_size, MALLOC_CAP_SPIRAM);
    TEST_ASSERT_NOT_NULL(mem_pool);
    esp_doa_capon_embedded_handle_t *doa =
        esp_doa_capon_embedded_create(mem_pool, mem_size, g_mic_coords, MIC_NUM);
    TEST_ASSERT_NOT_NULL(doa);

    // test memory consumption
    int create_size = start_size - heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int create_internal_size = start_internal_size - heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    printf("Internal RAM: %d, PSRAM: %d\n", create_internal_size, create_size - create_internal_size);
    esp_doa_capon_embedded_destroy(doa);
    heap_caps_free(mem_pool);

    // test memory leak
    int first_end_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int last_end_size = first_end_size;
    int mem_leak = start_size - last_end_size;
    printf("create&destroy times:%d, memory leak:%d\n", 1, mem_leak);

    for (int i = 0; i < 6; i++) {
        printf("create ...\n");
        mem_pool = heap_caps_malloc(mem_size, MALLOC_CAP_SPIRAM);
        TEST_ASSERT_NOT_NULL(mem_pool);
        doa = esp_doa_capon_embedded_create(mem_pool, mem_size, g_mic_coords, MIC_NUM);
        TEST_ASSERT_NOT_NULL(doa);

        printf("destroy ...\n");
        esp_doa_capon_embedded_destroy(doa);
        heap_caps_free(mem_pool);

        last_end_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        mem_leak = start_size - last_end_size;
        printf("create&destroy times:%d, memory leak:%d\n", i + 2, mem_leak);
    }

    TEST_ASSERT_EQUAL(true, (mem_leak) < 1000 && last_end_size == first_end_size);
}

TEST_CASE("gsc and doa process API & cpu loading", "[gsc_doa]")
{
    vTaskDelay(500 / portTICK_PERIOD_MS);

    size_t mem_size = esp_doa_capon_embedded_get_mem_size(MIC_NUM);
    void *mem_pool = heap_caps_malloc(mem_size, MALLOC_CAP_SPIRAM);
    TEST_ASSERT_NOT_NULL(mem_pool);
    esp_doa_capon_embedded_handle_t *doa =
        esp_doa_capon_embedded_create(mem_pool, mem_size, g_mic_coords, MIC_NUM);
    TEST_ASSERT_NOT_NULL(doa);
    gsc_handle_t *gsc = esp_gsc_create(g_mic_coords, MIC_NUM);
    TEST_ASSERT_NOT_NULL(gsc);

    int16_t *planar = (int16_t *)heap_caps_malloc(FRAME_LEN * MIC_NUM * sizeof(int16_t), MALLOC_CAP_8BIT);
    int16_t *inter = (int16_t *)heap_caps_malloc(FRAME_LEN * MIC_NUM * sizeof(int16_t), MALLOC_CAP_8BIT);
    int16_t *out = (int16_t *)heap_caps_malloc(FRAME_LEN * sizeof(int16_t), MALLOC_CAP_8BIT);
    TEST_ASSERT_NOT_NULL(planar);
    TEST_ASSERT_NOT_NULL(inter);
    TEST_ASSERT_NOT_NULL(out);

    float angle = -1.0f;
    int64_t out_energy = 0;
    int64_t doa_us = 0, gsc_us = 0;
    for (int f = 0; f < PROCESS_FRAMES; f++) {
        gen_frame(f, planar, inter);
        int64_t t0 = esp_timer_get_time();
        angle = esp_doa_capon_embedded_process(doa, inter, 1);
        int64_t t1 = esp_timer_get_time();
        esp_gsc_process(gsc, planar, angle, out);
        doa_us += t1 - t0;
        gsc_us += esp_timer_get_time() - t1;
        if (f >= PROCESS_FRAMES / 2) {  // skip adaptive filter warmup
            for (int n = 0; n < FRAME_LEN; n++) {
                out_energy += (int32_t)out[n] * out[n];
            }
        }
    }

    int run_ms = PROCESS_FRAMES * FRAME_LEN * 1000 / SAMPLE_RATE;
    int est = (int)lroundf(angle);
    int err = angle_diff(est, (int)SOURCE_ANGLE_DEG);
    printf("Done! %d frames (%d ms of audio): DOA avg %.2f ms/frame, GSC avg %.2f ms/frame, total CPU loading(single core):%.1f%%\n",
           PROCESS_FRAMES, run_ms, doa_us / 1000.0f / PROCESS_FRAMES, gsc_us / 1000.0f / PROCESS_FRAMES,
           (doa_us + gsc_us) / 1000.0f * 100.0f / run_ms);
    printf("estimated angle: %d deg (true: %d deg, error: %d deg), output energy: %lld\n",
           est, (int)SOURCE_ANGLE_DEG, err, (long long)out_energy);

    esp_gsc_destroy(gsc);
    esp_doa_capon_embedded_destroy(doa);
    heap_caps_free(mem_pool);
    heap_caps_free(planar);
    heap_caps_free(inter);
    heap_caps_free(out);

    /* Valid angle, within one grid step (10 deg) of the true direction,
       and non-silent beamformed output */
    TEST_ASSERT_EQUAL(true, angle >= 0.0f && angle < 360.0f);
    TEST_ASSERT_EQUAL(true, err <= 10);
    TEST_ASSERT_EQUAL(true, out_energy > 0);
}
