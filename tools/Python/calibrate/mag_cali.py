# -*- coding: utf-8 -*-
"""
MPU9250 Magnetometer Calibration System

Functions:
1. Acquire magnetic field data from 12 sensors
2. Use a stable calibration algorithm to compute hard iron / soft iron parameters
3. Generate an embedded-ready .h file

Usage:
1. Run the script
2. Hold the board with 12 sensors and perform a figure-eight motion during acquisition
3. The script will automatically compute the calibration coefficients and generate the .h file
"""

import os
import time
import struct
from datetime import datetime

import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def crc16_ccitt_false(data: bytes) -> int:
    """
    CRC16-CCITT-FALSE
    poly = 0x1021
    init = 0xFFFF
    refin = False
    refout = False
    xorout = 0x0000
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


def fit_ellipsoid_stable(mag_data: np.ndarray):
    """
    Stable magnetometer calibration algorithm

    Input:
        mag_data: Nx3 raw magnetic field data
    Output:
        M: 3x3 soft iron calibration matrix
        b: 1x3 hard iron offset
        residual: standard deviation of calibrated vector magnitudes
    """
    # ---------- 1. Initial hard iron ----------
    b = np.mean(mag_data, axis=0)
    X = mag_data - b

    # ---------- 2. Covariance matrix ----------
    C = np.cov(X, rowvar=False)

    # Handle abnormal cases
    if (not np.all(np.isfinite(C))) or (np.linalg.matrix_rank(C) < 3):
        C = C + 1e-6 * np.eye(3)

    # ---------- 3. Eigen decomposition ----------
    eigvals, V = np.linalg.eigh(C)
    D = eigvals.copy()

    # Prevent numerical issues
    D[D < 1e-6] = 1e-6

    # ---------- 4. Whitening matrix ----------
    M0 = V @ np.diag(1.0 / np.sqrt(D)) @ V.T

    # ---------- 5. Restore a reasonable radius scale ----------
    Xw = (M0 @ X.T).T
    r_raw = np.mean(np.linalg.norm(X, axis=1))
    r_new = np.mean(np.linalg.norm(Xw, axis=1))

    if r_new < 1e-12:
        scale = 1.0
    else:
        scale = r_raw / r_new

    M = scale * M0

    # ---------- 6. Residual ----------
    Xtest = (M @ X.T).T
    norms = np.linalg.norm(Xtest, axis=1)
    residual = np.std(norms)

    return M, b, residual


def apply_calibration_batch(mag_data: np.ndarray, hard_iron: np.ndarray, soft_iron: np.ndarray):
    """
    Apply calibration in batch
    """
    if mag_data.size == 0:
        return np.zeros((0, 3), dtype=np.float64)

    mag_centered = mag_data - hard_iron
    mag_calibrated = (soft_iron @ mag_centered.T).T
    return mag_calibrated


def generate_h_code(calibration_params, num_sensors: int) -> str:
    """
    Generate the h file content in the required format

    Format:
    #include "mag_calibration.h"

    const float mag_hard_iron[12][3] = { ... };
    const float mag_soft_iron[12][3][3] = { ... };
    """
    lines = []

    lines.append('#include "mag_calibration.h"\n')

    # ---------- hard iron ----------
    lines.append(f"const float mag_hard_iron[{num_sensors}][3] = {{")
    for i in range(num_sensors):
        hi = calibration_params[i]["hard_iron"]
        if hi is None:
            hi = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        if i < num_sensors - 1:
            lines.append(f"    {{{hi[0]:.6f}, {hi[1]:.6f}, {hi[2]:.6f}}},")
        else:
            lines.append(f"    {{{hi[0]:.6f}, {hi[1]:.6f}, {hi[2]:.6f}}}")
    lines.append("};\n")

    # ---------- soft iron ----------
    lines.append(f"const float mag_soft_iron[{num_sensors}][3][3] = {{")
    for i in range(num_sensors):
        si = calibration_params[i]["soft_iron_matrix"]
        if si is None:
            si = np.eye(3, dtype=np.float64)

        lines.append("    {")
        lines.append(f"        {{{si[0,0]:.6f}, {si[0,1]:.6f}, {si[0,2]:.6f}}},")
        lines.append(f"        {{{si[1,0]:.6f}, {si[1,1]:.6f}, {si[1,2]:.6f}}},")
        lines.append(f"        {{{si[2,0]:.6f}, {si[2,1]:.6f}, {si[2,2]:.6f}}}")
        if i < num_sensors - 1:
            lines.append("    },")
        else:
            lines.append("    }")
    lines.append("};")

    return "\n".join(lines)


def plot_sphere(ax, radius=1.0, alpha=0.1):
    """
    Draw a reference sphere
    """
    u = np.linspace(0, 2 * np.pi, 21)
    v = np.linspace(0, np.pi, 21)
    xs = radius * np.outer(np.cos(u), np.sin(v))
    ys = radius * np.outer(np.sin(u), np.sin(v))
    zs = radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(xs, ys, zs, alpha=alpha, edgecolor="none", color="red")


def safe_condition_number(mat: np.ndarray) -> float:
    try:
        return float(np.linalg.cond(mat))
    except Exception:
        return float("inf")


def main(
    serial_port: str = "COM7",
    baud_rate: int = 1024000,
    calibration_duration: int = 60,
    max_wait_s: float = 30.0,
    no_input: bool = False,
    no_plot: bool = False,
):
    # =========================
    # System parameter configuration
    # =========================
    print("=== MPU9250 Magnetometer Calibration System ===\n")

    # Serial port parameters
    # serial_port / baud_rate are provided via arguments

    # Calibration parameters
    num_sensors = 12
    num_channels = 3  # kept for 1:1 semantic consistency
    # calibration_duration is provided via arguments (seconds)
    fs = 100  # sampling rate: 100 Hz

    _ = num_channels  # silence unused warning style-equivalent

    # =========================
    # Initialize data storage
    # =========================
    print("Initializing data storage...")
    raw_mag_data = [np.zeros((0, 3), dtype=np.float32) for _ in range(num_sensors)]

    # =========================
    # Configure serial port
    # =========================
    print(f"Configuring serial connection ({serial_port}, {baud_rate} baud)...")
    try:
        s = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=0,
        )
        print("Serial connection established successfully\n")
    except Exception as e:
        raise RuntimeError(f"Serial connection failed: {e}") from e

    # =========================
    # Data acquisition
    # =========================
    print("Preparing to acquire magnetic field data...")
    print("Please hold the board with 12 sensors and perform a figure-eight motion")
    print(f"Acquisition duration: {calibration_duration} seconds")
    if not no_input:
        input("Press Enter to start...\n")

    print("\nStarting acquisition...")
    open_time = time.perf_counter()
    start_time = None
    sample_count = 0

    # Frame parsing parameters
    rx_buffer = bytearray()
    FRAME_HDR = bytes([0xAA, 0x55, 0xAA, 0x55])
    FRAME_PAYLOAD_LEN = 36 * 4  # 12 sensors * 3 axes * 4 bytes (minimum required)
    FRAME_MIN_LEN = 4 + 2 + 4 + 2  # header + len + tick + crc (payload may be longer)
    MAX_PAYLOAD_LEN = 2048  # accept diagnostic extensions, reject corrupted length

    bytes_total = 0
    hdr_total = 0
    crc_fail_total = 0
    len_mismatch_total = 0
    last_stat_time = 0.0
    last_wait_report = open_time

    try:
        while start_time is None or (time.perf_counter() - start_time) < calibration_duration:
            # Read serial data
            try:
                bytes_available = s.in_waiting
                if bytes_available > 0:
                    new_bytes = s.read(bytes_available)
                    rx_buffer.extend(new_bytes)
                    bytes_total += len(new_bytes)
            except Exception:
                continue

            current_raw = None
            current_time = time.perf_counter()
            if start_time is None:
                now_t = current_time - open_time
            else:
                now_t = current_time - start_time

            if (now_t - last_stat_time) > 2.0:
                print(
                    f"RX bytes={bytes_total} | buf={len(rx_buffer)} | hdr={hdr_total} | "
                    f"lenMismatch={len_mismatch_total} | crcFail={crc_fail_total} | "
                    f"samples={sample_count}"
                )
                last_stat_time = now_t

            # Frame parsing loop
            while len(rx_buffer) >= FRAME_MIN_LEN:
                hdr_pos = rx_buffer.find(FRAME_HDR)
                if hdr_pos == -1:
                    if len(rx_buffer) > 3:
                        rx_buffer = rx_buffer[-3:]
                    break

                hdr_total += 1
                start_idx = hdr_pos

                if start_idx > 0:
                    rx_buffer = rx_buffer[start_idx:]

                if len(rx_buffer) < FRAME_MIN_LEN:
                    break

                # Check payload length (allow extensions)
                payload_len = struct.unpack("<H", rx_buffer[4:6])[0]
                if payload_len > MAX_PAYLOAD_LEN:
                    len_mismatch_total += 1
                    rx_buffer = rx_buffer[1:]
                    continue

                if payload_len < FRAME_PAYLOAD_LEN:
                    len_mismatch_total += 1
                    rx_buffer = rx_buffer[1:]
                    continue

                # Check complete frame
                frame_len = 4 + 2 + 4 + payload_len + 2
                if len(rx_buffer) < frame_len:
                    break

                # CRC check
                frame = bytes(rx_buffer[:frame_len])
                crc_recv = struct.unpack("<H", frame[-2:])[0]
                crc_calc = crc16_ccitt_false(frame[:-2])

                if crc_calc != crc_recv:
                    crc_fail_total += 1
                    rx_buffer = rx_buffer[1:]
                    continue

                # Parse data
                # MATLAB: payload = frame(11 : 10 + payload_len)
                # Python 0-based: payload = frame[10 : 10 + payload_len]
                payload = frame[10:10 + payload_len]

                # First 36 floats are magnetometer values; ignore any appended diagnostics
                data_floats = np.frombuffer(payload[:FRAME_PAYLOAD_LEN], dtype="<f4")
                current_raw = data_floats.reshape((num_sensors, 3))

                if start_time is None:
                    start_time = time.perf_counter()
                    last_wait_report = start_time
                    last_stat_time = 0.0
                    print("First valid frame received, calibration timer started.")

                # Store data
                for sensor_idx in range(num_sensors):
                    raw_mag_data[sensor_idx] = np.vstack(
                        [raw_mag_data[sensor_idx], current_raw[sensor_idx:sensor_idx + 1, :]]
                    )

                sample_count += 1

                # Display progress
                if sample_count % 10 == 0:
                    elapsed = time.perf_counter() - start_time
                    print(
                        f"Acquired {sample_count} frames | Elapsed {elapsed:.1f}/{calibration_duration:.0f} s | "
                        f"Sensor 1: [{current_raw[0, 0]:.2f}, {current_raw[0, 1]:.2f}, {current_raw[0, 2]:.2f}]"
                    )

                rx_buffer = rx_buffer[frame_len:]
                break

            if sample_count == 0:
                now = time.perf_counter()
                if now - last_wait_report >= 5.0:
                    print(f"Waiting for first valid frame... {now - open_time:.1f}s")
                    last_wait_report = now
                if (now - open_time) >= float(max_wait_s):
                    print(f"Timeout waiting for first valid frame ({max_wait_s:.1f}s).")
                    break

            if not no_plot:
                plt.pause(0.001)
            time.sleep(0.001)

    except Exception as e:
        print(f"Error during acquisition: {e}")

    finally:
        s.close()

    print(f"\nAcquisition completed! Total frames acquired: {sample_count}")

    # =========================
    # Data quality diagnosis
    # =========================
    print("\n=== Data Quality Diagnosis ===")
    print("Sensor ID | Data Count | X Range | Y Range | Z Range | Outlier Count")
    print(f"{'---':<8s} | {'---':<8s} | {'---':<12s} | {'---':<12s} | {'---':<12s} | {'---':<8s}")

    for sensor_idx in range(num_sensors):
        mag_data = raw_mag_data[sensor_idx]
        n_points = mag_data.shape[0]

        if n_points < 10:
            print(f"Sensor {sensor_idx:2d} | {n_points:8d} | Insufficient data")
            continue

        x_range = np.max(mag_data[:, 0]) - np.min(mag_data[:, 0])
        y_range = np.max(mag_data[:, 1]) - np.min(mag_data[:, 1])
        z_range = np.max(mag_data[:, 2]) - np.min(mag_data[:, 2])

        outliers = 0
        for ch in range(3):
            mean_val = np.mean(mag_data[:, ch])
            std_val = np.std(mag_data[:, ch], ddof=1) if n_points > 1 else 0.0
            if std_val > 0:
                outliers += int(np.sum(np.abs(mag_data[:, ch] - mean_val) > 3 * std_val))

        print(
            f"Sensor {sensor_idx:2d} | {n_points:8d} | "
            f"[{x_range:.0f}] | [{y_range:.0f}] | [{z_range:.0f}] | {outliers:8d}"
        )

    # =========================
    # Calibration coefficient computation
    # =========================
    print("\n=== Starting Calibration Coefficient Computation ===")

    calibration_params = []
    for _ in range(num_sensors):
        calibration_params.append({
            "sensor_id": None,
            "hard_iron": None,
            "soft_iron_matrix": None,
            "residual": None,
            "data_points": None,
            "cond_num": None,
        })

    for sensor_idx in range(num_sensors):
        print(f"Processing sensor {sensor_idx + 1}...")

        mag_data = raw_mag_data[sensor_idx]

        if mag_data.shape[0] < 10:
            print("  Warning: insufficient data, using default parameters")
            calibration_params[sensor_idx]["sensor_id"] = sensor_idx + 1
            calibration_params[sensor_idx]["hard_iron"] = np.array([0.0, 0.0, 0.0], dtype=np.float64)
            calibration_params[sensor_idx]["soft_iron_matrix"] = np.eye(3, dtype=np.float64)
            calibration_params[sensor_idx]["residual"] = np.nan
            calibration_params[sensor_idx]["data_points"] = mag_data.shape[0]
            calibration_params[sensor_idx]["cond_num"] = 1.0
            continue

        # Check data quality
        data_range = np.max(mag_data, axis=0) - np.min(mag_data, axis=0)
        if np.all(data_range < 1):
            print(
                f"  Warning: data range too small "
                f"[{data_range[0]:.2f}, {data_range[1]:.2f}, {data_range[2]:.2f}], "
                f"using default parameters"
            )
            calibration_params[sensor_idx]["sensor_id"] = sensor_idx + 1
            calibration_params[sensor_idx]["hard_iron"] = np.array([0.0, 0.0, 0.0], dtype=np.float64)
            calibration_params[sensor_idx]["soft_iron_matrix"] = np.eye(3, dtype=np.float64)
            calibration_params[sensor_idx]["residual"] = np.nan
            calibration_params[sensor_idx]["data_points"] = mag_data.shape[0]
            calibration_params[sensor_idx]["cond_num"] = 1.0
            continue

        # ===== Correct calibration algorithm: stable whitening method =====
        soft_iron_matrix, hard_iron, residual = fit_ellipsoid_stable(mag_data)

        cond_num = safe_condition_number(soft_iron_matrix)
        if cond_num > 100:
            print(
                f"  Warning: calibration matrix condition number too large ({cond_num:.1f}), "
                f"possible over-amplification"
            )

        calibration_params[sensor_idx]["sensor_id"] = sensor_idx + 1
        calibration_params[sensor_idx]["hard_iron"] = hard_iron
        calibration_params[sensor_idx]["soft_iron_matrix"] = soft_iron_matrix
        calibration_params[sensor_idx]["residual"] = float(residual)
        calibration_params[sensor_idx]["data_points"] = mag_data.shape[0]
        calibration_params[sensor_idx]["cond_num"] = cond_num

        print(
            f"  Hard iron offset: "
            f"[{hard_iron[0]:.6f}, {hard_iron[1]:.6f}, {hard_iron[2]:.6f}]"
        )
        print("  Soft iron matrix:")
        print(
            f"    [{soft_iron_matrix[0,0]:.6f}, {soft_iron_matrix[0,1]:.6f}, {soft_iron_matrix[0,2]:.6f}]"
        )
        print(
            f"    [{soft_iron_matrix[1,0]:.6f}, {soft_iron_matrix[1,1]:.6f}, {soft_iron_matrix[1,2]:.6f}]"
        )
        print(
            f"    [{soft_iron_matrix[2,0]:.6f}, {soft_iron_matrix[2,1]:.6f}, {soft_iron_matrix[2,2]:.6f}]"
        )
        print(f"  Fitting residual: {residual:.6f} | Condition number: {cond_num:.2f}\n")

    # =========================
    # Visualization - raw data
    # =========================
    print("Generating visualization plots...")

    fig1 = plt.figure("Raw Magnetic Field Data - 3D Trajectories", figsize=(16, 10))
    fig1.suptitle("Raw Magnetic Field Data - 3D Trajectories")

    for sensor_idx in range(num_sensors):
        mag_data = raw_mag_data[sensor_idx]
        ax = fig1.add_subplot(3, 4, sensor_idx + 1, projection="3d")

        if mag_data.size > 0:
            ax.plot(mag_data[:, 0], mag_data[:, 1], mag_data[:, 2], "b.", markersize=3)
            center = np.mean(mag_data, axis=0)
            ax.plot([center[0]], [center[1]], [center[2]], "r*", markersize=15)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title(f"Sensor {sensor_idx + 1} (Raw)")
        ax.grid(True)
        try:
            ax.set_box_aspect((1, 1, 1))
        except Exception:
            pass

    # =========================
    # Visualization - calibrated data
    # =========================
    fig2 = plt.figure("Calibrated Magnetic Field Data - 3D Trajectories", figsize=(16, 10))
    fig2.suptitle("Calibrated Magnetic Field Data - 3D Trajectories")

    for sensor_idx in range(num_sensors):
        mag_data = raw_mag_data[sensor_idx]

        hard_iron = calibration_params[sensor_idx]["hard_iron"]
        soft_iron = calibration_params[sensor_idx]["soft_iron_matrix"]

        mag_calibrated = apply_calibration_batch(mag_data, hard_iron, soft_iron)

        ax = fig2.add_subplot(3, 4, sensor_idx + 1, projection="3d")

        if mag_calibrated.size > 0:
            ax.plot(
                mag_calibrated[:, 0],
                mag_calibrated[:, 1],
                mag_calibrated[:, 2],
                "g.",
                markersize=3,
            )
            center = np.mean(mag_calibrated, axis=0)
            ax.plot([center[0]], [center[1]], [center[2]], "r*", markersize=15)
            plot_sphere(ax, radius=1.0, alpha=0.1)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title(f"Sensor {sensor_idx + 1} (Calibrated)")
        ax.grid(True)
        try:
            ax.set_box_aspect((1, 1, 1))
        except Exception:
            pass

    # =========================
    # Visualization - time-domain comparison
    # =========================
    fig3 = plt.figure("Time-Domain Comparison (Blue: Raw, Green: Calibrated)", figsize=(16, 10))
    fig3.suptitle("Time-Domain Comparison (Blue: Raw, Green: Calibrated)")

    for sensor_idx in range(num_sensors):
        mag_data = raw_mag_data[sensor_idx]

        hard_iron = calibration_params[sensor_idx]["hard_iron"]
        soft_iron = calibration_params[sensor_idx]["soft_iron_matrix"]
        mag_calibrated = apply_calibration_batch(mag_data, hard_iron, soft_iron)

        time_axis = np.arange(mag_data.shape[0]) / fs if mag_data.shape[0] > 0 else np.array([])

        ax = fig3.add_subplot(4, 3, sensor_idx + 1)
        if mag_data.size > 0:
            ax.plot(time_axis, mag_data[:, 0], "b-", linewidth=1, label="Raw")
            ax.plot(time_axis, mag_calibrated[:, 0], "g-", linewidth=1, label="Calibrated")
            ax.legend(loc="best")

        ax.set_title(f"Sensor {sensor_idx + 1} - X Axis")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Magnitude")
        ax.grid(True)

    plt.tight_layout()
    if not no_plot:
        plt.show(block=False)

    # =========================
    # Generate h file
    # =========================
    print("\n=== Generating h File ===")

    h_code = generate_h_code(calibration_params, num_sensors)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    h_filename = f"mag_calibration_data_{timestamp}.h"
    with open(h_filename, "w", encoding="utf-8") as f:
        f.write(h_code)

    print(f"h file saved to: {h_filename}")
    print(f"\n{h_code}")

    # =========================
    # Save Python data
    # =========================
    npz_filename = f"mag_calibration_{timestamp}.npz"

    save_dict = {"num_sensors": np.array([num_sensors], dtype=np.int32)}
    for i in range(num_sensors):
        save_dict[f"raw_mag_data_{i}"] = raw_mag_data[i].astype(np.float32)
        save_dict[f"hard_iron_{i}"] = calibration_params[i]["hard_iron"].astype(np.float64)
        save_dict[f"soft_iron_{i}"] = calibration_params[i]["soft_iron_matrix"].astype(np.float64)
        save_dict[f"residual_{i}"] = np.array([calibration_params[i]["residual"]], dtype=np.float64)
        save_dict[f"data_points_{i}"] = np.array([calibration_params[i]["data_points"]], dtype=np.int32)
        save_dict[f"cond_num_{i}"] = np.array([calibration_params[i]["cond_num"]], dtype=np.float64)

    np.savez(npz_filename, **save_dict)
    print(f"Python data saved to: {npz_filename}")

    print("\nCalibration completed!")

    # 保持图窗
    if not no_plot:
        plt.show()


if __name__ == "__main__":
    import argparse
    from pathlib import Path
    import sys

    parser = argparse.ArgumentParser(description="MPU9250 magnetometer calibration (UART)")
    parser.add_argument("--selftest", action="store_true", help="Run protocol/decoder self-test and exit")
    parser.add_argument("--port", default="COM7", help="Serial port (e.g. COM5)")
    parser.add_argument("--baud", type=int, default=1024000, help="Baud rate")
    parser.add_argument("--duration", type=int, default=60, help="Calibration acquisition duration (seconds)")
    parser.add_argument("--max-wait", type=float, default=30.0, help="Max seconds to wait for first valid frame")
    parser.add_argument("--no-input", action="store_true", help="Do not wait for Enter before acquisition")
    parser.add_argument("--no-plot", action="store_true", help="Skip plotting/GUI windows")
    args = parser.parse_args()

    if args.selftest:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from uart_frame import build_frame, make_test_payload

        FRAME_PAYLOAD_LEN = 36 * 4
        payload = make_test_payload(extra_len=264)
        frame = build_frame(payload, tick=1)

        payload_len = struct.unpack("<H", frame[4:6])[0]
        if payload_len < FRAME_PAYLOAD_LEN:
            raise SystemExit("SELFTEST FAIL: payload_len too short")

        payload_bytes = frame[10:10 + payload_len]
        data_floats = np.frombuffer(payload_bytes[:FRAME_PAYLOAD_LEN], dtype="<f4")
        if data_floats.size != 36:
            raise SystemExit("SELFTEST FAIL: float decode size mismatch")

        current_raw = data_floats.reshape((12, 3))
        if current_raw.shape != (12, 3):
            raise SystemExit("SELFTEST FAIL: reshape mismatch")

        print("SELFTEST OK")
        raise SystemExit(0)

    main(
        serial_port=args.port,
        baud_rate=args.baud,
        calibration_duration=args.duration,
        max_wait_s=args.max_wait,
        no_input=args.no_input,
        no_plot=args.no_plot,
    )
