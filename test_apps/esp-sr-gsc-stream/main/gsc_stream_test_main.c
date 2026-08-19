/*
 * GSC Streaming Test for ESP32-P4
 *
 * Receives 4-mic frames from the host over the USB-serial-JTAG console,
 * beamforms them with esp_gsc (steered to the dataset target direction,
 * 0 deg) and sends the mono output back, frame by frame.
 *
 * Binary lockstep protocol (little-endian):
 *   host -> "GSC1" | u32 n_frames
 *   chip -> "RDY0"
 *   then n_frames times:
 *     host -> 1024 bytes (128 samples x 4 ch, int16, planar layout)
 *     chip -> 256 bytes  (128 samples, int16)
 *
 * Host counterpart: host_gsc_stream.py
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_gsc.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

#define MIC_NUM           4
#define MIC_RADIUS        0.05f   /* 5cm circular array radius (matches dataset) */
#define FRAME_LEN         128
#define TARGET_ANGLE_DEG  0.0f    /* data_4mic_r5cm_noise_gsc: target at 0 deg */

#define IN_FRAME_BYTES    (FRAME_LEN * MIC_NUM * sizeof(int16_t))
#define OUT_FRAME_BYTES   (FRAME_LEN * sizeof(int16_t))

/* Mic i sits at i*90 degrees, matching the dataset channel order */
static PlaneCoord g_mic_coords[MIC_NUM] = {
    { MIC_RADIUS, 0.0f, 0.0f},
    {0.0f,  MIC_RADIUS, 0.0f},
    {-MIC_RADIUS, 0.0f, 0.0f},
    {0.0f, -MIC_RADIUS, 0.0f},
};

static int16_t g_in_buf[FRAME_LEN * MIC_NUM];
static int16_t g_out_buf[FRAME_LEN];

static void read_exact(uint8_t *buf, size_t n)
{
    size_t got = 0;
    while (got < n) {
        size_t r = fread(buf + got, 1, n - got, stdin);
        if (r > 0) {
            got += r;
        } else {
            vTaskDelay(1);
        }
    }
}

static void wait_magic(void)
{
    static const char magic[4] = {'G', 'S', 'C', '1'};
    int pos = 0;
    while (pos < 4) {
        int c = getchar();
        if (c < 0) {
            vTaskDelay(1);
            continue;
        }
        pos = (c == magic[pos]) ? pos + 1 : 0;
    }
}

void app_main(void)
{
    /* The console vfs defaults to the no-driver USJ implementation (all reads
     * non-blocking) and CRLF line-ending translation on RX/TX - both corrupt
     * or stall a binary stream. Install the real driver (blocking reads) and
     * switch to LF (byte-transparent) mode. The RX ring buffer must hold a
     * whole frame burst (1024 bytes) since the host sends a frame at once. */
    usb_serial_jtag_driver_config_t usj_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usj_cfg.rx_buffer_size = 4 * IN_FRAME_BYTES;
    usj_cfg.tx_buffer_size = 4 * OUT_FRAME_BYTES;
    usb_serial_jtag_driver_install(&usj_cfg);
    usb_serial_jtag_vfs_use_driver();
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);

    setvbuf(stdout, NULL, _IONBF, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("GSC stream test ready (steer=%.0f deg, frame=%d, usj_connected=%d)\n",
           (double)TARGET_ANGLE_DEG, FRAME_LEN, (int)usb_serial_jtag_is_connected());

    while (1) {
        wait_magic();

        uint32_t n_frames = 0;
        read_exact((uint8_t *)&n_frames, sizeof(n_frames));

        gsc_handle_t *gsc = esp_gsc_create(g_mic_coords, MIC_NUM);
        if (gsc == NULL) {
            /* keep protocol in sync: report failure, then drain the file */
            fwrite("ERR0", 1, 4, stdout);
        } else {
            fwrite("RDY0", 1, 4, stdout);
        }

        for (uint32_t f = 0; f < n_frames; f++) {
            read_exact((uint8_t *)g_in_buf, IN_FRAME_BYTES);
            esp_gsc_process(gsc, g_in_buf, TARGET_ANGLE_DEG, g_out_buf);
            fwrite(g_out_buf, 1, OUT_FRAME_BYTES, stdout);
        }

        if (gsc != NULL) {
            esp_gsc_destroy(gsc);
        }
    }
}
