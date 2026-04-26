# -*- coding: utf-8 -*-
"""
UART Frame Sequence Rate Check Tool
Used to verify the actual UART output frame rate and sequence increment behavior
for the MPU9250 magnetometer data stream.

Notes:
- This script does NOT assume the sequence number increments by 1 per UART frame.
- It will automatically estimate the dominant sequence increment (e.g. 26).
- Frame loss is judged relative to that dominant increment.
"""

import time
import struct
from collections import Counter

import serial
import numpy as np
import matplotlib.pyplot as plt


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


def mode_of_positive_values(values: np.ndarray) -> int:
    """
    MATLAB mode-like behavior for integer positive values.
    """
    if values.size == 0:
        raise ValueError("No valid values for mode calculation.")

    counter = Counter(values.tolist())
    # 若频次相同，取较小值，尽量接近 MATLAB 常见行为
    dominant = sorted(counter.items(), key=lambda x: (-x[1], x[0]))[0][0]
    return int(dominant)


def main(
    serial_port: str = "COM5",
    baud_rate: int = 1024000,
    test_duration: int = 10,
    expected_frame_rate: float = 100.0,
    frame_rate_tol_hz: float = 1.0,
    max_wait_s: float = 30.0,
    no_plot: bool = False,
):
    # =========================
    # Configuration parameters
    # =========================
    expected_period_ms = 1000.0 / expected_frame_rate

    print("=== UART Sequence Increment Rate Check ===")
    print(f"Target UART frame rate: {expected_frame_rate:.1f} Hz "
          f"(period: {expected_period_ms:.3f} ms)")
    print(f"Test duration: {test_duration:d} seconds\n")

    # =========================
    # Configure serial port
    # =========================
    try:
        s = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=0,  # non-blocking
        )
        print(f"Serial port {serial_port} connected successfully")
    except Exception as e:
        raise RuntimeError(f"Serial connection failed: {e}") from e

    # =========================
    # Data acquisition
    # =========================
    rx_buffer = bytearray()
    FRAME_HDR = bytes([0xAA, 0x55, 0xAA, 0x55])
    FRAME_MIN_MAG_PAYLOAD_LEN = 36 * 4  # minimum required to be considered a valid telemetry frame
    FRAME_MIN_LEN = 4 + 2 + 4 + 2  # header + len + tick + crc (payload may be longer)
    MAX_PAYLOAD_LEN = 2048  # accept diagnostic extensions, reject corrupted length

    # Storage for sequence numbers and timestamps
    sequence_numbers = []
    timestamps = []
    frame_count = 0

    # Statistics
    crc_fail_count = 0
    len_mismatch_count = 0
    header_resync_count = 0

    open_time = time.perf_counter()
    start_time = None
    last_wait_report = open_time
    print("\nStarting data acquisition...")

    try:
        while start_time is None or (time.perf_counter() - start_time) < test_duration:
            # Read serial data
            bytes_available = s.in_waiting
            if bytes_available > 0:
                new_bytes = s.read(bytes_available)
                rx_buffer.extend(new_bytes)

            # Parse data frames
            while len(rx_buffer) >= FRAME_MIN_LEN:
                # Find frame header
                hdr_pos = rx_buffer.find(FRAME_HDR)
                if hdr_pos == -1:
                    if len(rx_buffer) > 3:
                        rx_buffer = rx_buffer[-3:]
                    break

                start_idx = hdr_pos
                if start_idx > 0:
                    header_resync_count += 1
                    rx_buffer = rx_buffer[start_idx:]

                if len(rx_buffer) < 10:
                    break

                # Payload length (allow extensions)
                payload_len = struct.unpack("<H", rx_buffer[4:6])[0]
                if payload_len > MAX_PAYLOAD_LEN:
                    len_mismatch_count += 1
                    rx_buffer = rx_buffer[1:]
                    continue
                if payload_len < FRAME_MIN_MAG_PAYLOAD_LEN:
                    len_mismatch_count += 1
                    rx_buffer = rx_buffer[1:]
                    continue

                # Check frame length
                frame_len = 4 + 2 + 4 + int(payload_len) + 2
                if len(rx_buffer) < frame_len:
                    break

                # Extract complete frame
                frame = bytes(rx_buffer[:frame_len])

                # Verify CRC
                crc_recv = struct.unpack("<H", frame[-2:])[0]
                crc_calc = crc16_ccitt_false(frame[:-2])
                if crc_calc != crc_recv:
                    crc_fail_count += 1
                    rx_buffer = rx_buffer[1:]
                    continue

                # Extract sequence number (bytes 7-10 in MATLAB -> index 6:10 in Python)
                seq_num = struct.unpack("<I", frame[6:10])[0]

                # Record sequence number and timestamp
                if start_time is None:
                    start_time = time.perf_counter()
                    last_wait_report = start_time
                    frame_time = 0.0
                    print("First valid frame received, test timer started.")
                else:
                    frame_time = time.perf_counter() - start_time

                frame_count += 1
                sequence_numbers.append(float(seq_num))
                timestamps.append(frame_time)

                # Remove processed frame
                rx_buffer = rx_buffer[frame_len:]

                # Show real-time progress
                if frame_count % 20 == 0:
                    print(f"Received {frame_count:d} frames | "
                          f"Current seq: {seq_num:d} | "
                          f"Time: {timestamps[-1]:.2f} s")

            if frame_count == 0:
                now = time.perf_counter()
                if now - last_wait_report >= 5.0:
                    print(f"Waiting for first valid frame... {now - open_time:.1f}s")
                    last_wait_report = now
                if (now - open_time) >= float(max_wait_s):
                    print(f"Timeout waiting for first valid frame ({max_wait_s:.1f}s).")
                    break

            time.sleep(0.001)

    except Exception as e:
        print(f"\nAn error occurred: {e}")

    finally:
        s.close()

    # =========================
    # Data analysis
    # =========================
    print("\n=== Data Analysis Results ===")
    print(f"Total received frames: {frame_count:d}")
    print(f"CRC failures: {crc_fail_count:d}")
    print(f"Length mismatches: {len_mismatch_count:d}")
    print(f"Header resync count: {header_resync_count:d}")

    if frame_count < 2:
        print("Insufficient data for analysis")
        return

    sequence_numbers = np.array(sequence_numbers, dtype=np.float64)
    timestamps = np.array(timestamps, dtype=np.float64)

    # 1. Compute actual UART frame rate
    total_time = timestamps[-1] - timestamps[0]
    actual_frame_rate = (frame_count - 1) / total_time
    print(f"\nActual UART frame rate: {actual_frame_rate:.3f} Hz "
          f"(expected: {expected_frame_rate:.1f} Hz)")
    print(f"Frame rate error: "
          f"{abs(actual_frame_rate - expected_frame_rate) / expected_frame_rate * 100:.2f}%")

    # 2. Sequence increment analysis
    seq_diff = np.diff(sequence_numbers)

    # Keep only positive increments for dominant-step estimation
    valid_seq_diff = seq_diff[(seq_diff > 0) & np.isfinite(seq_diff)]

    if valid_seq_diff.size == 0:
        print("\nNo valid positive sequence increments found")
        return

    dominant_increment = mode_of_positive_values(valid_seq_diff.astype(np.int64))

    print(f"\nDominant sequence increment: {dominant_increment:d}")
    print(f"This means the sequence counter advances by about "
          f"{dominant_increment:d} counts per UART frame.")

    # 3. Check for irregular increments / inferred dropped frames
    # A "normal" frame-to-frame increment should be:
    #   dominant_increment * k, where k is a positive integer
    # If k == 1: normal
    # If k > 1: likely dropped UART frames
    # If not close to an integer multiple: irregular event
    ratio_to_dom = seq_diff / dominant_increment
    nearest_mult = np.round(ratio_to_dom)
    mult_error = np.abs(ratio_to_dom - nearest_mult)

    # tolerance for deciding "integer multiple"
    multiple_tol = 1e-6

    is_positive = seq_diff > 0
    is_integer_multiple = is_positive & (mult_error < multiple_tol)
    is_normal = is_integer_multiple & (nearest_mult == 1)
    is_drop = is_integer_multiple & (nearest_mult > 1)
    is_irregular = ~is_integer_multiple

    num_normal = int(np.sum(is_normal))
    num_drop_events = int(np.sum(is_drop))
    num_irregular = int(np.sum(is_irregular))

    estimated_lost_frames = 0
    if num_drop_events > 0:
        estimated_lost_frames = int(np.sum(nearest_mult[is_drop] - 1))

    print("\nSequence increment consistency analysis:")
    print(f"  Normal increments (={dominant_increment:d}): {num_normal:d}")
    print(f"  Drop events (integer multiples > 1): {num_drop_events:d}")
    print(f"  Estimated lost UART frames: {estimated_lost_frames:d}")
    print(f"  Irregular increments: {num_irregular:d}")

    if num_drop_events > 0:
        drop_indices = np.where(is_drop)[0]
        print("\nExamples of inferred dropped-frame events:")
        for i in range(min(5, len(drop_indices))):
            idx = int(drop_indices[i])
            print(f"  Seq {int(sequence_numbers[idx])} -> {int(sequence_numbers[idx + 1])} | "
                  f"increment = {int(seq_diff[idx])} = {int(nearest_mult[idx])} x "
                  f"{dominant_increment:d} | estimated lost frames = "
                  f"{int(nearest_mult[idx]) - 1}")

    if num_irregular > 0:
        irregular_indices = np.where(is_irregular)[0]
        print("\nExamples of irregular increment events:")
        for i in range(min(5, len(irregular_indices))):
            idx = int(irregular_indices[i])
            print(f"  Seq {int(sequence_numbers[idx])} -> {int(sequence_numbers[idx + 1])} | "
                  f"increment = {int(seq_diff[idx])} | "
                  f"ratio to dominant = {ratio_to_dom[idx]:.6f}")

    # 4. Inter-frame timing analysis
    time_intervals = np.diff(timestamps) * 1000.0  # ms
    mean_interval = float(np.mean(time_intervals))
    std_interval = float(np.std(time_intervals))

    print("\nInter-frame interval statistics:")
    print(f"  Mean: {mean_interval:.3f} ms (expected: {expected_period_ms:.3f} ms)")
    print(f"  Std: {std_interval:.3f} ms")
    print(f"  Min: {np.min(time_intervals):.3f} ms")
    print(f"  Max: {np.max(time_intervals):.3f} ms")

    # 5. Sequence growth rate
    seq_rate = (sequence_numbers[-1] - sequence_numbers[0]) / total_time
    print(f"\nSequence growth rate: {seq_rate:.3f} counts/s")

    predicted_seq_rate = actual_frame_rate * dominant_increment
    print("Predicted sequence growth rate from frame_rate * dominant_increment: "
          f"{predicted_seq_rate:.3f} counts/s")
    print(f"Difference: {abs(seq_rate - predicted_seq_rate):.3f} counts/s")

    if no_plot:
        print("\nSkipping plots (--no-plot)")
        print("\nTest completed")
        return

    # =========================
    # Visualization
    # =========================
    plt.figure("UART Sequence Increment Analysis", figsize=(13, 8.5))

    # Subplot 1: sequence number vs time
    plt.subplot(2, 2, 1)
    plt.plot(timestamps, sequence_numbers, "b.-", linewidth=1.2)
    plt.xlabel("Time (s)")
    plt.ylabel("Sequence Number")
    plt.title("Sequence Number vs Time")
    plt.grid(True)

    p = np.polyfit(timestamps, sequence_numbers, 1)
    fit_line = np.polyval(p, timestamps)
    plt.plot(timestamps, fit_line, "r--", linewidth=1.2)
    plt.legend(
        ["Measured sequence", f"Linear fit slope: {p[0]:.2f} counts/s"],
        loc="upper left"
    )

    # Subplot 2: sequence increment
    plt.subplot(2, 2, 2)
    plt.plot(seq_diff, "g.-", linewidth=1.0)
    plt.xlabel("Frame Index")
    plt.ylabel("Sequence Increment")
    plt.title(f"Sequence Increment (dominant = {dominant_increment:d})")
    plt.grid(True)
    plt.axhline(
        dominant_increment,
        color="r",
        linestyle="--",
        linewidth=1.5,
        label=f"Dominant = {dominant_increment:d}"
    )
    plt.legend(loc="upper left")

    # Subplot 3: inter-frame interval histogram
    plt.subplot(2, 2, 3)
    plt.hist(time_intervals, bins=50, color="b", edgecolor="k")
    plt.xlabel("Time Interval (ms)")
    plt.ylabel("Count")
    plt.title("Inter-Frame Interval Distribution")
    plt.axvline(
        expected_period_ms,
        color="r",
        linestyle="--",
        linewidth=2,
        label=f"Expected {expected_period_ms:.2f} ms"
    )
    plt.axvline(
        mean_interval,
        color="g",
        linestyle="--",
        linewidth=1.5,
        label=f"Mean {mean_interval:.2f} ms"
    )
    plt.grid(True)
    plt.legend(loc="upper right")

    # Subplot 4: inter-frame interval vs time
    plt.subplot(2, 2, 4)
    plt.plot(timestamps[1:], time_intervals, "b.-")
    plt.xlabel("Time (s)")
    plt.ylabel("Inter-Frame Interval (ms)")
    plt.title("Inter-Frame Interval vs Time")
    plt.grid(True)
    plt.axhline(
        expected_period_ms,
        color="r",
        linestyle="--",
        linewidth=1.5,
        label=f"Expected {expected_period_ms:.2f} ms"
    )
    plt.ylim([0, max(expected_period_ms * 2, np.max(time_intervals) * 1.05)])
    plt.legend(loc="upper right")

    plt.tight_layout()
    plt.show()

    # =========================
    # Conclusion
    # =========================
    print("\n=== Conclusion ===")

    frame_rate_ok = abs(actual_frame_rate - expected_frame_rate) < frame_rate_tol_hz
    sequence_behavior_ok = (num_irregular == 0)

    if frame_rate_ok:
        print(f"UART frame rate check passed: {actual_frame_rate:.2f} Hz "
              f"≈ {expected_frame_rate:.1f} Hz")
    else:
        print(f"UART frame rate deviation is too large: {actual_frame_rate:.2f} Hz "
              f"(expected {expected_frame_rate:.1f} Hz)")

    print(f"Dominant sequence increment is {dominant_increment:d}, not necessarily 1.")

    if sequence_behavior_ok:
        print(f"Sequence behavior is consistent with a dominant increment of "
              f"{dominant_increment:d}.")
    else:
        print("Irregular sequence increment events were detected.")

    if estimated_lost_frames == 0:
        print("No dropped UART frames were inferred relative to the dominant increment.")
    else:
        print(f"Estimated dropped UART frames: {estimated_lost_frames:d}")

    print("\nTest completed")


if __name__ == "__main__":
    import argparse
    from pathlib import Path
    import sys

    parser = argparse.ArgumentParser(description="UART frame sequence rate check")
    parser.add_argument("--selftest", action="store_true", help="Run protocol/decoder self-test and exit")
    parser.add_argument("--port", default="COM5", help="Serial port (e.g. COM5)")
    parser.add_argument("--baud", type=int, default=1024000, help="Baud rate")
    parser.add_argument("--duration", type=int, default=10, help="Test duration (seconds)")
    parser.add_argument("--expected-frame-rate", type=float, default=100.0, help="Expected UART frame rate (Hz)")
    parser.add_argument("--frame-rate-tol", type=float, default=1.0, help="Allowed frame rate error (Hz)")
    parser.add_argument("--max-wait", type=float, default=30.0, help="Max seconds to wait for first valid frame")
    parser.add_argument("--no-plot", action="store_true", help="Skip visualization/GUI")
    args = parser.parse_args()

    if args.selftest:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from uart_frame import build_frame, make_test_payload, crc16_ccitt_false

        payload = make_test_payload(extra_len=264)
        frame = build_frame(payload, tick=1234)

        crc_recv = struct.unpack("<H", frame[-2:])[0]
        crc_calc = crc16_ccitt_false(frame[:-2])
        if crc_recv != crc_calc:
            raise SystemExit("SELFTEST FAIL: crc mismatch")

        payload_len = struct.unpack("<H", frame[4:6])[0]
        if payload_len < (36 * 4):
            raise SystemExit("SELFTEST FAIL: payload_len too short")

        tick = struct.unpack("<I", frame[6:10])[0]
        if tick != 1234:
            raise SystemExit("SELFTEST FAIL: tick mismatch")

        print("SELFTEST OK")
        raise SystemExit(0)

    main(
        serial_port=args.port,
        baud_rate=args.baud,
        test_duration=args.duration,
        expected_frame_rate=args.expected_frame_rate,
        frame_rate_tol_hz=args.frame_rate_tol,
        max_wait_s=args.max_wait,
        no_plot=args.no_plot,
    )
