声源定向 (DOA)
================

:link_to_translation:`en:[English]`

概述
----

ESP-SR DOA（Direction of Arrival，声源定向）模块用于估计声源相对于麦克风阵列的方位角。该模块基于 Capon/MVDR（最小方差无失真响应）算法，并针对嵌入式平台做了优化，广泛应用于说话人定位、摄像头转向等场景，也可作为波束形成的前级模块（参见 :doc:`GSC 波束形成 <../gsc_beamforming/README>`）。

.. note::

   本文档介绍的嵌入式 DOA 模块目前仅支持 ESP32-P4。

嵌入式 DOA 模块具有以下特点：

- 支持任意阵列几何形状：2 至 8 个麦克风，麦克风坐标在运行时配置
- 帧长：16 kHz 采样率下每通道 128 个采样点（每帧 8 ms）
- FFT 点数：256
- 处理频带：1500–4500 Hz（针对语音优化）
- 角度分辨率：10 度（36 个候选角度：0°、10°、…、350°）
- 仅使用单精度浮点运算
- 处理过程中零动态内存分配（所有缓冲区在创建时预分配）

使用方式
--------

头文件为 :project_file:`include/esp32p4/esp_doa_capon_embedded.h`。

**基本流程：**

1. **定义麦克风阵列几何坐标**

   麦克风坐标以米为单位，采用右手坐标系，每个麦克风一项。各坐标项可以是任意顺序，但传入 ``esp_doa_capon_embedded_process()`` 的第 ``i`` 路音频必须始终来自 ``mic_coord[i]`` 对应的麦克风。

   .. code-block:: c

      #include "esp_doa_capon_embedded.h"

      /* 4 麦均匀圆阵，半径 5 cm；第 i 个麦克风位于 i*90 度方向 */
      PlaneCoord mic_coords[4] = {
          { 0.05f, 0.0f, 0.0f},
          { 0.0f,  0.05f, 0.0f},
          {-0.05f, 0.0f, 0.0f},
          { 0.0f, -0.05f, 0.0f},
      };

2. **查询所需内存大小并分配内存池**

   DOA 模块在处理过程中不做动态内存分配，需要调用方提供内存池。建议从 PSRAM 分配（``MALLOC_CAP_SPIRAM``）。

   .. code-block:: c

      size_t mem_size = esp_doa_capon_embedded_get_mem_size(4);
      void *mem_pool = heap_caps_malloc(mem_size, MALLOC_CAP_SPIRAM);

3. **创建 DOA 实例**

   .. code-block:: c

      esp_doa_capon_embedded_handle_t *doa =
          esp_doa_capon_embedded_create(mem_pool, mem_size, mic_coords, 4);

4. **处理音频帧**

   输入为 ``mic_num`` 通道 16-bit PCM 音频，**交织（interleaved）** 排布（``[ch0_s0, ch1_s0, ..., chN_s0, ch0_s1, ...]``），每帧每通道 128 个采样点。

   .. code-block:: c

      int16_t audio_frame[128 * 4];  // 4 通道，交织排布
      int vad = 1;                   // 1 = 语音，0 = 噪声/静音
      float angle = esp_doa_capon_embedded_process(doa, audio_frame, vad);

   返回角度单位为度，范围 0–360，定义在阵列绝对坐标系中（0° = x 轴正方向，逆时针），与 ``mic_coord`` 中麦克风的排列顺序无关。

   .. note::

      - 当 ``vad_result`` 为 0 时，所有自适应状态（协方差递推、矩阵求逆、空间谱）都会被冻结，并直接返回上一次估计的角度。建议接入 AFE 模块的 VAD 结果，避免纯噪声帧破坏估计。
      - 协方差递推需要若干帧才能收敛，创建（或复位）后最初几帧的估计结果应丢弃。

5. **（可选）复位处理器状态**

   复位协方差矩阵和平滑滤波器，适用于长时间静音后重新开始估计等场景：

   .. code-block:: c

      esp_doa_capon_embedded_reset(doa);

6. **释放资源**

   .. code-block:: c

      esp_doa_capon_embedded_destroy(doa);
      heap_caps_free(mem_pool);

.. warning::

   将 DOA 与 :doc:`GSC 波束形成 <../gsc_beamforming/README>` 级联使用时，两个模块必须传入**相同的**麦克风坐标数组，否则估计出的角度会对应错误的通道。

精度测试
--------

测试程序 ``test_apps/esp-sr-doa-accuracy`` 在芯片端评估 DOA 估计精度。测试数据集为仿真的 4 麦均匀圆阵（半径 5 cm）纯净语音，声源位于半径 2 m 的圆周上，角度从 0° 到 330°、步进 30°（从 +x 轴逆时针计量）。

测试方法：

- 对 12 个角度，每个角度向 ``esp_doa_capon_embedded_process()`` 送入 64 帧（VAD 强制为语音）。
- 跳过前 10 帧，等待协方差递推收敛。
- 将剩余帧的估计结果与真实角度比较，分别统计完全命中率和误差在一个网格步长（±10°）以内的准确率。

测试结果：

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - 指标
     - 结果
     - 说明
   * - 完全命中率
     - 100.0% (648/648)
     - 估计角度与真实角度完全一致
   * - ±10° 内准确率
     - 100.0% (648/648)
     - 误差在一个网格步长以内

资源消耗
--------

下表为典型的资源占用与性能数据（16 kHz 采样率）：

.. only:: esp32p4

    .. list-table::
      :header-rows: 1
      :widths: 20 15 15 20 20

      * - 麦克风数量
        - 内部 RAM (KB)
        - PSRAM (KB)
        - 每帧耗时 (ms)
        - CPU 占用 (%)
      * - 4
        - 1.8
        - 204.3
        - 1.76 / 8
        - 22.0

    .. note::

      - 帧长为 8 ms（16 kHz 采样率下每通道 128 个采样点）。
      - 测试条件：ESP32-P4 @ 400 MHz，HEX PSRAM @ 250 MHz，4 麦均匀圆阵（半径 5 cm）。
      - 实际资源消耗可能因麦克风数量、编译器优化等级和具体配置略有差异。
