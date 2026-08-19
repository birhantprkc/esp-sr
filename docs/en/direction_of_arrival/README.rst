Direction of Arrival (DOA)
==========================

:link_to_translation:`zh_CN:[中文]`

Overview
--------

The ESP-SR DOA (Direction of Arrival) module estimates the direction of a sound source relative to the microphone array. It is based on the Capon/MVDR (Minimum Variance Distortionless Response) algorithm with an embedded-optimized implementation, and is widely used in scenarios such as speaker localization, camera steering, and as the front end of beamforming (see :doc:`GSC Beamforming <../gsc_beamforming/README>`).

.. note::

   The embedded DOA module described in this document is currently only supported on ESP32-P4.

The embedded DOA module has the following features:

- Arbitrary microphone array geometry: any array shape with 2 to 8 microphones, microphone coordinates are configured at runtime
- Frame size: 128 samples per channel at 16 kHz (8 ms per frame)
- FFT size: 256 points
- Processing bandwidth: 1500–4500 Hz (optimized for speech)
- Angle resolution: 10 degrees (36 candidate angles: 0°, 10°, ..., 350°)
- Single precision floating point only
- Zero dynamic memory allocation during processing (all buffers are pre-allocated at creation)

Usage
-----

The header file is :project_file:`include/esp32p4/esp_doa_capon_embedded.h`.

**Basic Flow:**

1. **Define the microphone array geometry**

   Microphone coordinates are given in meters, in a right-hand coordinate system, one entry per microphone. The entries may be in any order, but audio channel ``i`` passed to ``esp_doa_capon_embedded_process()`` must always come from the microphone at ``mic_coord[i]``.

   .. code-block:: c

      #include "esp_doa_capon_embedded.h"

      /* 4-mic uniform circular array, radius 5 cm; mic i sits at i*90 degrees */
      PlaneCoord mic_coords[4] = {
          { 0.05f, 0.0f, 0.0f},
          { 0.0f,  0.05f, 0.0f},
          {-0.05f, 0.0f, 0.0f},
          { 0.0f, -0.05f, 0.0f},
      };

2. **Create a DOA instance**

   All memory (handle and internal memory pool) is allocated by the module itself and released by ``esp_doa_capon_embedded_destroy()``. Buffers are allocated from PSRAM by default; see **Memory Configuration** below.

   .. code-block:: c

      esp_doa_capon_embedded_handle_t *doa =
          esp_doa_capon_embedded_create(mic_coords, 4);

3. **Process audio frames**

   The input is ``mic_num``-channel 16-bit PCM audio in **planar** layout (``[ch0_0..ch0_127, ch1_0..ch1_127, ...]``), 128 samples per channel per frame.

   .. code-block:: c

      int16_t audio_frame[128 * 4];  // 4 channels, planar: [ch0_0..ch0_127, ch1_0..ch1_127, ...]
      int vad = 1;                   // 1 = speech, 0 = noise/silence
      float angle = esp_doa_capon_embedded_process(doa, audio_frame, vad);

   The returned angle is in degrees, range 0–360, defined in the absolute array coordinate system (0° = positive x-axis, counter-clockwise), independent of the microphone ordering in ``mic_coord``. ``-1.0f`` is returned on error.

   .. note::

      - When ``vad_result`` is 0, all adaptive state (covariance recursion, matrix inversion, spectrum) is frozen and the last estimated angle is returned unchanged. Feeding a VAD result from the AFE module is recommended, so that noise-only frames do not corrupt the estimation.
      - The covariance recursion needs several frames to converge. Estimates from the first few frames after creation (or reset) should be discarded.

4. **(Optional) Reset the processor state**

   Resets the covariance matrix and smoothing filters, e.g., after a long pause:

   .. code-block:: c

      esp_doa_capon_embedded_reset(doa);

5. **Release resources**

   ``esp_doa_capon_embedded_destroy()`` releases all resources allocated by ``esp_doa_capon_embedded_create()`` (passing NULL is safely ignored):

   .. code-block:: c

      esp_doa_capon_embedded_destroy(doa);

.. warning::

   When chaining DOA with :doc:`GSC beamforming <../gsc_beamforming/README>`, pass the **same** microphone coordinate array to both modules, otherwise the estimated angle refers to the wrong channels.

.. tip::

   ``esp_doa_capon_embedded_print_info(doa)`` prints the processor configuration (frame size, FFT size, frequency range, etc.) for debugging.

Memory Configuration
--------------------

- On ESP32-P4, the DOA internal buffers (memory pool, about 200 KB for 4 microphones) are allocated in PSRAM by default.
- To place them in internal RAM instead, define ``ESP_DOA_DISABLE_PSRAM`` before including ``esp_doa_capon_embedded.h`` (or as a compile definition).

Accuracy Evaluation
-------------------

The test application ``test_apps/esp-sr-gsc-doa`` evaluates the DOA estimation accuracy on chip. The test dataset is a simulated 4-mic uniform circular array (radius 5 cm) with clean speech; the sound source is placed on a 2 m circle at angles 0° to 330° in 30° steps (counter-clockwise from the +x axis).

Test method:

- For each of the 12 angles, 64 frames are fed to ``esp_doa_capon_embedded_process()`` (VAD forced to speech).
- The first 10 frames are skipped to let the covariance recursion converge.
- The remaining frames are compared against the true angle, reporting both the exact match rate and the rate within one grid step (±10°).

Test results:

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Metric
     - Result
     - Note
   * - Exact match rate
     - 94.0% (609/648)
     - Estimated angle equals the true angle
   * - Accuracy within ±10°
     - 94.0% (609/648)
     - Error within one grid step

Resource Consumption
--------------------

The following table shows typical resource usage and performance data (16 kHz sample rate):

.. only:: esp32p4

    .. list-table::
      :header-rows: 1
      :widths: 20 15 15 20 20

      * - Microphones
        - Internal RAM (KB)
        - PSRAM (KB)
        - Time per Frame (ms)
        - CPU Usage (%)
      * - 4
        - 1.8
        - 204.3
        - 1.76 / 8
        - 22.0

    .. note::

      - Frame length is 8 ms (128 samples per channel at 16 kHz).
      - Test setting: ESP32-P4 @ 400 MHz, HEX PSRAM @ 250 MHz, 4-mic uniform circular array (radius 5 cm).
      - Actual resource consumption may vary slightly depending on the number of microphones, compiler optimization level, and specific configuration.
