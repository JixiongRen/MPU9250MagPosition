# -*- coding: utf-8 -*-
"""
plot_sensor_raw_data.py

Read raw MPU9250 sensor data from UART and plot time-domain waveforms
and Welch power spectral density (PSD).

Frame format:
- Header: 0xAA 0x55 0xAA 0x55 (4 bytes)
- Payload length: uint16 (2 bytes, little-endian)
- Timestamp: uint32 (4 bytes, little-endian, FreeRTOS tick)
- Note: firmware may append diagnostics after the first 144 bytes of sensor data
- Sensor payload: 12 sensors × 3 axes × 4 bytes = 144 bytes (single)
- CRC16: uint16 (2 bytes, little-endian), CRC16-CCITT-FALSE over header..payload
- Total length: 4 + 2 + 4 + payload_len + 2 bytes/frame (variable)
"""

import time
import struct
from datetime import datetime

import numpy as np
import serial
import matplotlib.pyplot as plt
from scipy.signal import welch
from scipy.io import savemat


def crc16_ccitt_false_matlab(data: bytes) -> int:
    """
    CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF)
    MATLAB-equivalent implementation
    """
    crc = 0xFFFF
    poly = 0x1021

    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def read_mpu9250_uart_data(serial_port, baud_rate, max_samples=1000, tick_hz=1000, max_wait_s: float = 30.0):
    """
    Read raw MPU9250 sensor frames from UART

    Inputs:
        serial_port : serial port name, e.g. 'COM7'
        baud_rate   : UART baud rate
        max_samples : maximum number of samples to acquire (default = 1000)
        tick_hz     : FreeRTOS tick frequency in Hz (default = 1000)

    Outputs:
        sensor_data     : [N, 12, 3] array
        timestamps      : [N] time in seconds converted from tick
        tick_timestamps : [N] raw uint32 tick values
        info            : dict with acquisition metadata
    """
    FRAME_HEADER = bytes([0xAA, 0x55, 0xAA, 0x55])
    NUM_SENSORS = 12
    NUM_AXES = 3
    PAYLOAD_LEN_EXPECTED = NUM_SENSORS * NUM_AXES * 4
    MAX_PAYLOAD_LEN = 2048  # accept diagnostic extensions, reject corrupted length

    print(f"Opening serial port {serial_port}...")
    try:
        s = serial.Serial(serial_port, baud_rate, timeout=10)
        s.reset_input_buffer()
        s.reset_output_buffer()
        print("Serial port opened successfully")
    except Exception as e:
        raise RuntimeError(f"Unable to open serial port: {e}") from e

    sensor_data = np.zeros((max_samples, NUM_SENSORS, NUM_AXES), dtype=np.float32)
    timestamps = np.zeros((max_samples,), dtype=np.float64)
    tick_timestamps = np.zeros((max_samples,), dtype=np.uint32)

    sample_count = 0
    crc_fail_count = 0
    payload_len_mismatch_count = 0
    header_resync_count = 0
    open_time = time.perf_counter()
    last_wait_report = open_time
    first_tick = None

    print("\nStarting data acquisition...")
    print("Press Ctrl+C to stop acquisition\n")

    rx_buf = bytearray()

    try:
        while sample_count < max_samples:
            n_avail = s.in_waiting
            if n_avail > 0:
                new_bytes = s.read(n_avail)
                rx_buf.extend(new_bytes)
            else:
                time.sleep(0.005)

            while True:
                header_idx = rx_buf.find(FRAME_HEADER)
                if header_idx == -1:
                    if len(rx_buf) > 4096:
                        rx_buf = rx_buf[-3:]
                    break

                h = header_idx
                if h > 0:
                    header_resync_count += 1
                    rx_buf = rx_buf[h:]

                if len(rx_buf) < 10:
                    break

                payload_len = struct.unpack("<H", rx_buf[4:6])[0]
                if payload_len > MAX_PAYLOAD_LEN:
                    payload_len_mismatch_count += 1
                    rx_buf = rx_buf[1:]
                    continue
                frame_len = 4 + 2 + 4 + int(payload_len) + 2

                if len(rx_buf) < frame_len:
                    break

                frame = bytes(rx_buf[:frame_len])
                rx_buf = rx_buf[frame_len:]

                if payload_len < PAYLOAD_LEN_EXPECTED:
                    payload_len_mismatch_count += 1
                    continue

                crc_rx = struct.unpack("<H", frame[-2:])[0]
                crc_calc = crc16_ccitt_false_matlab(frame[:-2])
                if crc_rx != crc_calc:
                    crc_fail_count += 1
                    continue

                tick = struct.unpack("<I", frame[6:10])[0]
                payload = frame[10:10 + payload_len]
                values = np.frombuffer(payload[:PAYLOAD_LEN_EXPECTED], dtype="<f4")

                if values.size != NUM_SENSORS * NUM_AXES:
                    continue

                if first_tick is None:
                    first_tick = tick
                    print("First valid frame received, acquisition started.")

                # MATLAB:
                # values = reshape(values, [NUM_AXES, NUM_SENSORS])';
                values = values.reshape((NUM_SENSORS, NUM_AXES))

                sensor_data[sample_count, :, :] = values.reshape((1, NUM_SENSORS, NUM_AXES))
                tick_timestamps[sample_count] = tick
                timestamps[sample_count] = float(tick - first_tick) / float(tick_hz)

                sample_count += 1

                if sample_count % 100 == 0:
                    print(
                        f"Collected {sample_count} samples | "
                        f"tick: {tick_timestamps[sample_count - 1]} | "
                        f"t: {timestamps[sample_count - 1]:.3f} s"
                    )

                if sample_count >= max_samples:
                    break

            if sample_count == 0:
                now = time.perf_counter()
                if now - last_wait_report >= 5.0:
                    print(f"Waiting for first valid frame... {now - open_time:.1f}s")
                    last_wait_report = now
                if (now - open_time) >= float(max_wait_s):
                    print(f"Timeout waiting for first valid frame ({max_wait_s:.1f}s).")
                    break

    except KeyboardInterrupt:
        print("\nData acquisition stopped: KeyboardInterrupt")
    except Exception as e:
        print(f"\nData acquisition stopped: {e}")
    finally:
        s.close()

    print("\nSerial port closed")

    sensor_data = sensor_data[:sample_count, :, :]
    timestamps = timestamps[:sample_count]
    tick_timestamps = tick_timestamps[:sample_count]

    info = {
        "sampleCount": sample_count,
        "numSensors": NUM_SENSORS,
        "numAxes": NUM_AXES,
        "crcFailCount": crc_fail_count,
        "payloadLenMismatchCount": payload_len_mismatch_count,
        "headerResyncCount": header_resync_count,
        "tickHz": tick_hz,
        "serialPort": serial_port,
        "baudRate": baud_rate,
    }

    return sensor_data, timestamps, tick_timestamps, info


def main(
    serial_port: str = "COM5",
    baud_rate: int = 1024000,
    max_samples: int = 1000,
    tick_hz: int = 1000,
    max_wait_s: float = 30.0,
    no_plot: bool = False,
    no_save: bool = False,
):
    # =========================
    # Configuration
    # =========================
    # =========================
    # Acquire data from UART
    # =========================
    sensor_data, timestamps, tick_timestamps, info = read_mpu9250_uart_data(
        serial_port, baud_rate, max_samples, tick_hz, max_wait_s=max_wait_s
    )

    print("\n=== Acquisition Summary ===")
    print(f"Total valid samples: {info['sampleCount']}")
    print(f"CRC failures: {info['crcFailCount']}")
    print(f"Payload length mismatches: {info['payloadLenMismatchCount']}")
    print(f"Header resync count: {info['headerResyncCount']}")

    # =========================
    # Compute average sampling rate
    # =========================
    if timestamps.size > 1:
        total_duration = timestamps[-1] - timestamps[0]
        if total_duration > 0:
            avg_sampling_rate = (timestamps.size - 1) / total_duration
        else:
            avg_sampling_rate = np.nan

        print(f"Average sampling rate: {avg_sampling_rate:.2f} Hz")
        print(f"Total duration: {total_duration:.2f} s")
    else:
        avg_sampling_rate = np.nan
        print("Insufficient samples to estimate sampling rate")

    if no_plot:
        print("\nSkipping plots (--no-plot)")

        if (not no_save) and sensor_data.size > 0:
            save_data = input("Do you want to save the acquired data? (Yes/No) [No]: ").strip()

            if save_data.lower() == "yes":
                timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"sensor_data_{timestamp_str}.mat"

                save_dict = {
                    "sensorData": sensor_data,
                    "timestamps": timestamps,
                    "tickTimestamps": tick_timestamps,
                    "info": info,
                }

                savemat(filename, save_dict)
                print(f"Data saved to: {filename}")

        print("\nScript execution completed")
        return

    # =========================
    # Plot time-domain waveforms: Z-axis for all sensors
    # =========================
    plt.figure(
        num="Z-Axis Time-Domain Waveforms for All Sensors",
        figsize=(14, 8)
    )

    for sensor in range(info["numSensors"]):
        plt.subplot(4, 3, sensor + 1)
        z_data = sensor_data[:, sensor, 2]
        plt.plot(timestamps, z_data, "b-", linewidth=1.2)

        plt.title(f"Sensor {sensor + 1} (Z Axis)")
        plt.xlabel("Time (s)")
        plt.ylabel("Magnetic Field")
        plt.grid(True)

        mean_val = np.mean(z_data) if z_data.size > 0 else np.nan
        std_val = np.std(z_data, ddof=1) if z_data.size > 1 else np.nan

        plt.text(
            0.02, 0.98,
            f"Mean: {mean_val:.2f}\nStd: {std_val:.2f}",
            transform=plt.gca().transAxes,
            verticalalignment="top",
            bbox=dict(facecolor="white", edgecolor="black")
        )

    plt.tight_layout()

    # =========================
    # Plot Welch PSD: Z-axis for all sensors
    # =========================
    plt.figure(
        num="Z-Axis Welch PSD for All Sensors",
        figsize=(14, 8)
    )

    for sensor in range(info["numSensors"]):
        plt.subplot(4, 3, sensor + 1)
        z_data = sensor_data[:, sensor, 2]

        if z_data.size >= 8 and np.isfinite(avg_sampling_rate) and avg_sampling_rate > 0:
            nfft = min(256, max(64, int(2 ** np.ceil(np.log2(z_data.size)))))
            window_len = min(256, z_data.size)
            if window_len < 8:
                window_len = z_data.size
            overlap_len = window_len // 2

            f, pxx = welch(
                z_data,
                fs=avg_sampling_rate,
                window="hamming",
                nperseg=window_len,
                noverlap=overlap_len,
                nfft=nfft,
                detrend=False,
                return_onesided=True,
                scaling="density",
            )

            plt.plot(f, 10 * np.log10(pxx + np.finfo(float).eps), "b-", linewidth=1.2)
            plt.xlabel("Frequency (Hz)")
            plt.ylabel("PSD (dB/Hz)")
            plt.title(f"Sensor {sensor + 1} (Z Axis PSD)")
            plt.grid(True)
        else:
            plt.plot([0], [0], "w.")
            plt.title(f"Sensor {sensor + 1} (Z Axis PSD)")
            plt.xlabel("Frequency (Hz)")
            plt.ylabel("PSD (dB/Hz)")
            plt.text(
                0.5, 0.5, "Not enough data",
                transform=plt.gca().transAxes,
                horizontalalignment="center"
            )
            plt.grid(True)

    plt.tight_layout()

    # =========================
    # Plot all three axes for Sensor 1: time domain
    # =========================
    plt.figure(
        num="Sensor 1 Three-Axis Time-Domain Data",
        figsize=(12, 6)
    )

    axis_names = ["X Axis", "Y Axis", "Z Axis"]
    colors = ["r", "g", "b"]

    for axis_idx in range(info["numAxes"]):
        plt.subplot(3, 1, axis_idx + 1)
        data = sensor_data[:, 0, axis_idx]
        plt.plot(timestamps, data, colors[axis_idx], linewidth=1.2)

        plt.title(f"Sensor 1 - {axis_names[axis_idx]}")
        plt.xlabel("Time (s)")
        plt.ylabel("Magnetic Field")
        plt.grid(True)

        mean_val = np.mean(data) if data.size > 0 else np.nan
        std_val = np.std(data, ddof=1) if data.size > 1 else np.nan
        plt.legend([f"Mean: {mean_val:.2f}, Std: {std_val:.2f}"], loc="best")

    plt.tight_layout()

    # =========================
    # Plot all three axes for Sensor 1: Welch PSD
    # =========================
    plt.figure(
        num="Sensor 1 Three-Axis Welch PSD",
        figsize=(12, 6)
    )

    for axis_idx in range(info["numAxes"]):
        plt.subplot(3, 1, axis_idx + 1)
        data = sensor_data[:, 0, axis_idx]

        if data.size >= 8 and np.isfinite(avg_sampling_rate) and avg_sampling_rate > 0:
            nfft = min(256, max(64, int(2 ** np.ceil(np.log2(data.size)))))
            window_len = min(256, data.size)
            if window_len < 8:
                window_len = data.size
            overlap_len = window_len // 2

            f, pxx = welch(
                data,
                fs=avg_sampling_rate,
                window="hamming",
                nperseg=window_len,
                noverlap=overlap_len,
                nfft=nfft,
                detrend=False,
                return_onesided=True,
                scaling="density",
            )

            plt.plot(f, 10 * np.log10(pxx + np.finfo(float).eps), colors[axis_idx], linewidth=1.2)
        else:
            plt.plot([0], [0], "w.")
            plt.text(
                0.5, 0.5, "Not enough data",
                transform=plt.gca().transAxes,
                horizontalalignment="center"
            )

        plt.title(f"Sensor 1 - {axis_names[axis_idx]} PSD")
        plt.xlabel("Frequency (Hz)")
        plt.ylabel("PSD (dB/Hz)")
        plt.grid(True)

    plt.tight_layout()
    plt.show(block=False)

    # =========================
    # Save data
    # =========================
    if (not no_save) and sensor_data.size > 0:
        save_data = input("Do you want to save the acquired data? (Yes/No) [No]: ").strip()

        if save_data.lower() == "yes":
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"sensor_data_{timestamp_str}.mat"

            save_dict = {
                "sensorData": sensor_data,
                "timestamps": timestamps,
                "tickTimestamps": tick_timestamps,
                "info": info,
            }

            savemat(filename, save_dict)
            print(f"Data saved to: {filename}")

    print("\nScript execution completed")
    plt.show()


if __name__ == "__main__":
    import argparse
    from pathlib import Path
    import sys

    parser = argparse.ArgumentParser(description="Plot MPU9250 raw UART data")
    parser.add_argument("--selftest", action="store_true", help="Run protocol/decoder self-test and exit")
    parser.add_argument("--port", default="COM5", help="Serial port (e.g. COM5)")
    parser.add_argument("--baud", type=int, default=1024000, help="Baud rate")
    parser.add_argument("--max-samples", type=int, default=1000, help="Max samples to acquire")
    parser.add_argument("--tick-hz", type=int, default=1000, help="FreeRTOS tick rate (Hz)")
    parser.add_argument("--max-wait", type=float, default=30.0, help="Max seconds to wait for first valid frame")
    parser.add_argument("--no-plot", action="store_true", help="Skip plotting (useful for automated runs)")
    parser.add_argument("--no-save", action="store_true", help="Do not prompt/save .mat output")
    args = parser.parse_args()

    if args.selftest:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from uart_frame import build_frame, make_test_payload

        NUM_SENSORS = 12
        NUM_AXES = 3
        PAYLOAD_LEN_EXPECTED = NUM_SENSORS * NUM_AXES * 4

        payload = make_test_payload(extra_len=264)
        frame = build_frame(payload, tick=99)

        payload_len = struct.unpack("<H", frame[4:6])[0]
        if payload_len < PAYLOAD_LEN_EXPECTED:
            raise SystemExit("SELFTEST FAIL: payload_len too short")

        payload_bytes = frame[10:10 + payload_len]
        values = np.frombuffer(payload_bytes[:PAYLOAD_LEN_EXPECTED], dtype="<f4")
        if values.size != NUM_SENSORS * NUM_AXES:
            raise SystemExit("SELFTEST FAIL: float decode size mismatch")

        print("SELFTEST OK")
        raise SystemExit(0)

    main(
        serial_port=args.port,
        baud_rate=args.baud,
        max_samples=args.max_samples,
        tick_hz=args.tick_hz,
        max_wait_s=args.max_wait,
        no_plot=args.no_plot,
        no_save=args.no_save,
    )
