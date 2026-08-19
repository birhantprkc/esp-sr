#!/usr/bin/env python3
"""Host counterpart of the GSC_STREAM_TEST firmware.

Streams every mixture WAV of data_4mic_r5cm_noise_gsc to the ESP32-P4 over
the USB-serial-JTAG console (binary lockstep protocol, see
gsc_stream_test_main.c), saves the beamformed mono output to
<dataset>/gsc_out/<name>_gsc.wav and reports SI-SDR against the target
reference (first SKIP_FRAMES frames are excluded from the metrics to let the
adaptive filters converge).

Usage:  python3 host_gsc_stream.py
Requires: pyserial, numpy
"""
import glob
import os
import struct
import time
import wave

import numpy as np
import serial

PORT = "/dev/ttyACM1"
DATASET = "/home/shenxiaozheng/work26/test/array_4mic_r5cm_20260814/data_4mic_r5cm_noise_gsc"
OUTDIR = os.path.join(DATASET, "gsc_out")
FRAME = 128          # samples per channel per frame (firmware GSC frame)
CH = 4
SKIP_FRAMES = 125    # 1s warmup, excluded from the SI-SDR metrics


def read_wav(path):
    """Minimal RIFF reader supporting PCM16 and float32.

    Returns (samples, rate) with samples shaped (n, ch), int16 or float32.
    """
    with open(path, "rb") as f:
        riff = f.read()
    assert riff[:4] == b"RIFF" and riff[8:12] == b"WAVE", path
    pos, fmt, data = 12, None, None
    while pos + 8 <= len(riff):
        cid = riff[pos:pos + 4]
        size = struct.unpack("<I", riff[pos + 4:pos + 8])[0]
        body = riff[pos + 8:pos + 8 + size]
        if cid == b"fmt ":
            fmt = body
        elif cid == b"data":
            data = body
        pos += 8 + size + (size & 1)
    tag, ch, rate = struct.unpack("<HHI", fmt[:8])
    if tag == 1:
        a = np.frombuffer(data, dtype="<i2")
    elif tag == 3:
        a = np.frombuffer(data, dtype="<f4")
    else:
        raise ValueError("%s: unsupported WAV format tag %d" % (path, tag))
    return a.reshape(-1, ch), rate


def reset_chip(port):
    """Hard reset via DTR/RTS (USB-serial-JTAG maps them to IO0/EN)."""
    port.dtr = False
    port.rts = True
    time.sleep(0.1)
    port.dtr = True
    port.rts = False
    time.sleep(0.05)
    port.dtr = False


def read_exact(port, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = port.read(n - len(buf))
        if not chunk:
            raise TimeoutError("serial timeout (%d/%d bytes)" % (len(buf), n))
        buf += chunk
    return bytes(buf)


def best_lag(est, ref, max_lag=1024):
    """Lag of est relative to ref (est[t] ~ ref[t - lag]), FFT correlation."""
    n = min(16384, len(est), len(ref))
    a = est[:n].astype(np.float64)
    b = ref[:n].astype(np.float64)
    a -= a.mean()
    b -= b.mean()
    nfft = 1 << (2 * n - 1).bit_length()
    corr = np.fft.irfft(np.fft.rfft(a, nfft) * np.conj(np.fft.rfft(b, nfft)), nfft)
    corr = np.concatenate([corr[-max_lag:], corr[:max_lag + 1]])
    return int(np.argmax(corr)) - max_lag


def si_sdr(est, ref):
    alpha = np.dot(est, ref) / (np.dot(ref, ref) + 1e-12)
    proj = alpha * ref
    err = est - proj
    return 10.0 * np.log10(np.dot(proj, proj) / (np.dot(err, err) + 1e-12) + 1e-12)


def shifted_si_sdr(est, ref, lag):
    if lag >= 0:
        return si_sdr(est[lag:], ref[:len(ref) - lag])
    return si_sdr(est[:len(est) + lag], ref[-lag:])


def main():
    port = serial.Serial(PORT, 115200, timeout=30)
    reset_chip(port)
    time.sleep(2.0)
    port.reset_input_buffer()

    os.makedirs(OUTDIR, exist_ok=True)
    files = sorted(glob.glob(os.path.join(DATASET, "cmu_*.wav")))
    print("dataset: %s (%d mixture files)" % (DATASET, len(files)))

    rows = []
    for idx, path in enumerate(files):
        name = os.path.basename(path)[:-4]
        mix, rate = read_wav(path)
        assert rate == 16000 and mix.shape[1] == CH, name
        n_frames = mix.shape[0] // FRAME
        mix = mix[:n_frames * FRAME]

        port.write(b"GSC1" + struct.pack("<I", n_frames))
        win = bytearray()
        while not win.endswith(b"RDY0"):
            b = port.read(1)
            if not b:
                raise TimeoutError("no RDY0 for " + name)
            win += b

        out = np.empty(n_frames * FRAME, dtype="<i2")
        for f in range(n_frames):
            planar = np.ascontiguousarray(mix[f * FRAME:(f + 1) * FRAME].T)
            port.write(planar.tobytes())
            out[f * FRAME:(f + 1) * FRAME] = np.frombuffer(read_exact(port, FRAME * 2), dtype="<i2")

        out_path = os.path.join(OUTDIR, name + "_gsc.wav")
        with wave.open(out_path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(16000)
            w.writeframes(out.tobytes())

        ref, _ = read_wav(os.path.join(DATASET, "reference", name + "_target.wav"))
        skip = SKIP_FRAMES * FRAME
        r = ref[skip:n_frames * FRAME, 0].astype(np.float64)
        m = mix[skip:, 0].astype(np.float64)
        o = out[skip:].astype(np.float64)
        in_sdr = shifted_si_sdr(m, r, best_lag(m, r))
        out_sdr = shifted_si_sdr(o, r, best_lag(o, r))
        rows.append((name, in_sdr, out_sdr))
        print("[%2d/%d] %-55s in %6.2f dB  out %6.2f dB  gain %+5.2f dB"
              % (idx + 1, len(files), name, in_sdr, out_sdr, out_sdr - in_sdr))

    print("\n=== summary by SNR ===")
    for snr in ("neg5db", "0db", "pos5db"):
        sel = [x for x in rows if "_snr_" + snr in x[0]]
        if sel:
            mi = np.mean([x[1] for x in sel])
            mo = np.mean([x[2] for x in sel])
            print("  snr %-6s: in %6.2f dB -> out %6.2f dB (mean gain %+5.2f dB, %d files)"
                  % (snr, mi, mo, mo - mi, len(sel)))
    all_in = np.mean([x[1] for x in rows])
    all_out = np.mean([x[2] for x in rows])
    print("  overall     : in %6.2f dB -> out %6.2f dB (mean gain %+5.2f dB, %d files)"
          % (all_in, all_out, all_out - all_in, len(rows)))
    print("output audio saved to: %s" % OUTDIR)


if __name__ == "__main__":
    main()
