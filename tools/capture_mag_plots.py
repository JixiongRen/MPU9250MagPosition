import argparse
import csv
import math
import struct
import sys
import time
from pathlib import Path

import serial

from usart_monitor import (
    AUTO_BAUDRATES,
    FRAME_HEADER,
    FRAME_MIN_TOTAL_LEN,
    FRAME_PAYLOAD_LEN_V1,
    FRAME_PAYLOAD_LEN_V2,
    FRAME_PAYLOAD_LEN_V3,
    FRAME_PAYLOAD_LEN_V4,
    FRAME_PAYLOAD_LEN_V5,
    SENSOR_COUNT,
    detect_protocol,
    list_ports,
    crc16_ccitt_false,
)

AUTO_BAUD_DETECT_TIMEOUT_S = 8.0
FIRST_FRAME_WAIT_TIMEOUT_S = 10.0
VALID_PAYLOAD_LENGTHS = (
    FRAME_PAYLOAD_LEN_V1,
    FRAME_PAYLOAD_LEN_V2,
    FRAME_PAYLOAD_LEN_V3,
    FRAME_PAYLOAD_LEN_V4,
    FRAME_PAYLOAD_LEN_V5,
)
CHANNEL_COLORS = (
    ("X", "#d62728"),
    ("Y", "#2ca02c"),
    ("Z", "#1f77b4"),
)


def safe_print(text: str) -> None:
    sys.stdout.write(text + "\n")
    sys.stdout.flush()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture 120 seconds of magnetometer frames and save 12 SVG plots.")
    parser.add_argument("--port", default="COM4", help="Serial port, default: COM4")
    parser.add_argument(
        "--baudrate",
        default="auto",
        help="Baudrate or auto. Supported auto candidates: 115200, 1024000",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=120.0,
        help="Capture duration in seconds, default: 120",
    )
    parser.add_argument(
        "--output-dir",
        default="tools/mag_plots",
        help="Directory to store generated SVG files",
    )
    parser.add_argument(
        "--csv-path",
        default="",
        help="Optional CSV output path. Default: <output-dir>/mag_capture.csv",
    )
    parser.add_argument(
        "--input-csv",
        default="",
        help="Load an existing CSV file and regenerate HTML outputs without capturing",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=1,
        help="Number of repeated capture runs. When greater than 1, each run is saved under run_XX",
    )
    parser.add_argument(
        "--no-prompt-before-run",
        action="store_true",
        help="Do not wait for Enter before each run when using repeated capture mode",
    )
    return parser.parse_args()


def decode_mag_values(payload: bytes) -> tuple[float, ...]:
    return struct.unpack_from("<36f", payload, 0)


def collect_frames(port: str, baudrate: int, duration: float) -> tuple[list[float], list[list[list[float]]]]:
    buffer = bytearray()
    times: list[float] = []
    sensor_series = [[[ ] for _ in range(3)] for _ in range(SENSOR_COUNT)]
    safe_print(f"Capture start: port={port}, baudrate={baudrate}, duration={duration:.1f}s")

    with serial.Serial(port, baudrate, timeout=0.2) as ser:
        ser.reset_input_buffer()
        open_time = time.time()
        start_time: float | None = None
        last_report: float | None = None
        last_wait_report: float = open_time

        while True:
            now = time.time()
            if start_time is None:
                if now - open_time >= FIRST_FRAME_WAIT_TIMEOUT_S:
                    break
            elif now - start_time >= duration:
                break

            chunk = ser.read(4096)
            if not chunk:
                continue
            buffer.extend(chunk)

            while True:
                header_idx = buffer.find(FRAME_HEADER)
                if header_idx < 0:
                    if len(buffer) > FRAME_MIN_TOTAL_LEN:
                        del buffer[:-FRAME_MIN_TOTAL_LEN]
                    break

                if header_idx > 0:
                    del buffer[:header_idx]

                if len(buffer) < FRAME_MIN_TOTAL_LEN:
                    break

                payload_len = struct.unpack_from("<H", buffer, 4)[0]
                total_len = 4 + 2 + 4 + payload_len + 2
                if payload_len not in VALID_PAYLOAD_LENGTHS:
                    del buffer[:4]
                    continue

                if len(buffer) < total_len:
                    break

                frame = bytes(buffer[:total_len])
                expected_crc = struct.unpack_from("<H", frame, total_len - 2)[0]
                actual_crc = crc16_ccitt_false(frame[:-2])
                if expected_crc != actual_crc:
                    del buffer[:4]
                    continue

                if start_time is None:
                    start_time = time.time()
                    last_report = start_time
                    safe_print("First valid frame received, capture timer started.")

                capture_time = time.time() - start_time
                payload = frame[10:-2]
                mag_values = decode_mag_values(payload)
                times.append(capture_time)
                for sensor_idx in range(SENSOR_COUNT):
                    base = sensor_idx * 3
                    sensor_series[sensor_idx][0].append(mag_values[base])
                    sensor_series[sensor_idx][1].append(mag_values[base + 1])
                    sensor_series[sensor_idx][2].append(mag_values[base + 2])
                del buffer[:total_len]

            now = time.time()
            if start_time is None:
                waited = now - open_time
                if now - last_wait_report >= 5.0:
                    safe_print(f"Waiting for first valid frame... {waited:.1f}s")
                    last_wait_report = now
                continue

            if last_report is not None and now - last_report >= 5.0:
                safe_print(f"Captured {len(times)} frames in {now - start_time:.1f}s")
                last_report = now

    return times, sensor_series


def build_ticks(min_value: float, max_value: float, count: int) -> list[float]:
    if count <= 1:
        return [min_value]
    span = max_value - min_value
    if abs(span) < 1e-12:
        return [min_value for _ in range(count)]
    return [min_value + span * i / (count - 1) for i in range(count)]


def escape_xml(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&apos;")
    )


def format_points(times: list[float], values: list[float], x_min: float, x_max: float, y_min: float, y_max: float,
                  left: float, top: float, width: float, height: float) -> str:
    x_span = max(x_max - x_min, 1e-9)
    y_span = max(y_max - y_min, 1e-9)
    points: list[str] = []
    for timestamp, value in zip(times, values):
        x = left + (timestamp - x_min) / x_span * width
        y = top + height - (value - y_min) / y_span * height
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points)


def write_sensor_svg(sensor_id: int, times: list[float], channels: list[list[float]], output_path: Path) -> None:
    width = 1200
    height = 720
    left = 90.0
    right = 40.0
    top = 70.0
    bottom = 90.0
    plot_width = width - left - right
    plot_height = height - top - bottom

    x_min = times[0]
    x_max = times[-1] if times[-1] > times[0] else times[0] + 1.0
    all_values = channels[0] + channels[1] + channels[2]
    y_min = min(all_values)
    y_max = max(all_values)
    if math.isclose(y_min, y_max, rel_tol=0.0, abs_tol=1e-9):
        pad = 1.0 if math.isclose(y_min, 0.0, abs_tol=1e-9) else abs(y_min) * 0.1
        y_min -= pad
        y_max += pad
    else:
        pad = (y_max - y_min) * 0.08
        y_min -= pad
        y_max += pad

    x_ticks = build_ticks(x_min, x_max, 6)
    y_ticks = build_ticks(y_min, y_max, 6)

    svg: list[str] = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">')
    svg.append('<rect width="100%" height="100%" fill="white"/>')
    svg.append(f'<text x="{width / 2:.1f}" y="36" text-anchor="middle" font-family="Arial" font-size="24">Sensor {sensor_id:02d} Magnetic Channels</text>')
    svg.append(f'<text x="{width / 2:.1f}" y="{height - 24}" text-anchor="middle" font-family="Arial" font-size="18">Time (s)</text>')
    svg.append(f'<text x="26" y="{height / 2:.1f}" text-anchor="middle" font-family="Arial" font-size="18" transform="rotate(-90 26 {height / 2:.1f})">Field</text>')
    svg.append(f'<rect x="{left:.2f}" y="{top:.2f}" width="{plot_width:.2f}" height="{plot_height:.2f}" fill="none" stroke="black" stroke-width="1.5"/>')

    for tick in x_ticks:
        x = left + (tick - x_min) / max(x_max - x_min, 1e-9) * plot_width
        svg.append(f'<line x1="{x:.2f}" y1="{top:.2f}" x2="{x:.2f}" y2="{top + plot_height:.2f}" stroke="#d0d0d0" stroke-width="1"/>')
        svg.append(f'<text x="{x:.2f}" y="{top + plot_height + 28:.2f}" text-anchor="middle" font-family="Arial" font-size="14">{tick:.1f}</text>')

    for tick in y_ticks:
        y = top + plot_height - (tick - y_min) / max(y_max - y_min, 1e-9) * plot_height
        svg.append(f'<line x1="{left:.2f}" y1="{y:.2f}" x2="{left + plot_width:.2f}" y2="{y:.2f}" stroke="#d0d0d0" stroke-width="1"/>')
        svg.append(f'<text x="{left - 12:.2f}" y="{y + 5:.2f}" text-anchor="end" font-family="Arial" font-size="14">{tick:.2f}</text>')

    for idx, (label, color) in enumerate(CHANNEL_COLORS):
        points = format_points(times, channels[idx], x_min, x_max, y_min, y_max, left, top, plot_width, plot_height)
        svg.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.5" points="{points}"/>')
        legend_x = left + 20 + idx * 90
        legend_y = 48
        svg.append(f'<line x1="{legend_x:.2f}" y1="{legend_y:.2f}" x2="{legend_x + 26:.2f}" y2="{legend_y:.2f}" stroke="{color}" stroke-width="3"/>')
        svg.append(f'<text x="{legend_x + 34:.2f}" y="{legend_y + 5:.2f}" font-family="Arial" font-size="16">{escape_xml(label)}</text>')

    svg.append('</svg>')
    output_path.write_text("\n".join(svg), encoding="utf-8")


def save_plots(times: list[float], sensor_series: list[list[list[float]]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for sensor_idx in range(SENSOR_COUNT):
        output_path = output_dir / f"sensor_{sensor_idx + 1:02d}.svg"
        write_sensor_svg(sensor_idx + 1, times, sensor_series[sensor_idx], output_path)
        safe_print(f"Saved {output_path}")


def save_csv(times: list[float], sensor_series: list[list[list[float]]], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    headers = ["time_s"]
    for sensor_idx in range(SENSOR_COUNT):
        sensor_id = sensor_idx + 1
        headers.extend(
            [
                f"sensor_{sensor_id:02d}_x",
                f"sensor_{sensor_id:02d}_y",
                f"sensor_{sensor_id:02d}_z",
            ]
        )

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(headers)
        for sample_idx, timestamp in enumerate(times):
            row = [f"{timestamp:.6f}"]
            for sensor_idx in range(SENSOR_COUNT):
                row.extend(
                    [
                        f"{sensor_series[sensor_idx][0][sample_idx]:.9f}",
                        f"{sensor_series[sensor_idx][1][sample_idx]:.9f}",
                        f"{sensor_series[sensor_idx][2][sample_idx]:.9f}",
                    ]
                )
            writer.writerow(row)

    safe_print(f"Saved {csv_path}")


def load_csv(csv_path: Path) -> tuple[list[float], list[list[list[float]]]]:
    times: list[float] = []
    sensor_series = [[[] for _ in range(3)] for _ in range(SENSOR_COUNT)]
    with csv_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.reader(csv_file)
        header = next(reader, None)
        if header is None:
            raise ValueError("CSV file is empty")
        expected_columns = 1 + SENSOR_COUNT * 3
        if len(header) < expected_columns:
            raise ValueError(f"CSV file has {len(header)} columns, expected at least {expected_columns}")

        for row in reader:
            if not row:
                continue
            times.append(float(row[0]))
            for sensor_idx in range(SENSOR_COUNT):
                base = 1 + sensor_idx * 3
                sensor_series[sensor_idx][0].append(float(row[base]))
                sensor_series[sensor_idx][1].append(float(row[base + 1]))
                sensor_series[sensor_idx][2].append(float(row[base + 2]))

    return times, sensor_series


def compute_capture_duration(times: list[float]) -> float:
    return times[-1] - times[0] if len(times) >= 2 else 0.0


def compute_channel_stats(values: list[float]) -> dict[str, float]:
    sample_count = len(values)
    if sample_count == 0:
        raise ValueError("Cannot compute statistics for an empty channel")
    mean_value = math.fsum(values) / sample_count
    variance = math.fsum((value - mean_value) * (value - mean_value) for value in values) / sample_count
    return {
        "min": min(values),
        "max": max(values),
        "peak_to_peak": max(values) - min(values),
        "peak_abs": max(abs(value) for value in values),
        "mean": mean_value,
        "stddev": math.sqrt(max(variance, 0.0)),
        "samples": float(sample_count),
    }


def save_overview_html(output_dir: Path) -> None:
    output_path = output_dir / "overview.html"
    cards = []
    for sensor_idx in range(SENSOR_COUNT):
        sensor_id = sensor_idx + 1
        file_name = f"sensor_{sensor_id:02d}.svg"
        cards.append(
            "\n".join(
                [
                    '<section class="card">',
                    f'  <h2>Sensor {sensor_id:02d}</h2>',
                    f'  <img src="{file_name}" alt="Sensor {sensor_id:02d} plot" loading="lazy"/>',
                    '</section>',
                ]
            )
        )

    html = "\n".join(
        [
            '<!DOCTYPE html>',
            '<html lang="en">',
            '<head>',
            '  <meta charset="utf-8"/>',
            '  <meta name="viewport" content="width=device-width, initial-scale=1"/>',
            '  <title>Magnetic Sensor Overview</title>',
            '  <style>',
            '    body { font-family: Arial, sans-serif; margin: 0; background: #f5f7fb; color: #1f2937; }',
            '    header { padding: 24px 32px; background: #111827; color: white; }',
            '    header h1 { margin: 0 0 8px 0; font-size: 28px; }',
            '    header p { margin: 0; color: #d1d5db; }',
            '    main { padding: 24px; display: grid; grid-template-columns: repeat(2, minmax(420px, 1fr)); gap: 20px; }',
            '    .card { background: white; border-radius: 14px; padding: 16px; box-shadow: 0 8px 24px rgba(15, 23, 42, 0.08); }',
            '    .card h2 { margin: 0 0 12px 0; font-size: 20px; }',
            '    .card img { width: 100%; height: auto; border: 1px solid #e5e7eb; border-radius: 10px; background: white; }',
            '    @media (max-width: 1100px) { main { grid-template-columns: 1fr; } }',
            '  </style>',
            '</head>',
            '<body>',
            '  <header>',
            '    <h1>Magnetic Sensor Overview</h1>',
            '    <p>Combined overview of 12 sensor SVG plots</p>',
            '  </header>',
            '  <main>',
            *cards,
            '  </main>',
            '</body>',
            '</html>',
        ]
    )
    output_path.write_text(html, encoding="utf-8")
    safe_print(f"Saved {output_path}")


def save_stats_report(times: list[float], sensor_series: list[list[list[float]]], output_dir: Path) -> None:
    output_path = output_dir / "stats_report.html"
    duration = compute_capture_duration(times)
    sections: list[str] = []
    for sensor_idx in range(SENSOR_COUNT):
        sensor_id = sensor_idx + 1
        rows = []
        for axis_idx, (axis_name, _) in enumerate(CHANNEL_COLORS):
            stats = compute_channel_stats(sensor_series[sensor_idx][axis_idx])
            rows.append(
                "\n".join(
                    [
                        '      <tr>',
                        f'        <td>{axis_name}</td>',
                        f'        <td>{int(stats["samples"])}</td>',
                        f'        <td>{stats["peak_to_peak"]:.6f}</td>',
                        f'        <td>{stats["peak_abs"]:.6f}</td>',
                        f'        <td>{stats["mean"]:.6f}</td>',
                        f'        <td>{stats["stddev"]:.6f}</td>',
                        f'        <td>{stats["min"]:.6f}</td>',
                        f'        <td>{stats["max"]:.6f}</td>',
                        '      </tr>',
                    ]
                )
            )

        sections.append(
            "\n".join(
                [
                    '<section class="sensor-block">',
                    f'  <h2>Sensor {sensor_id:02d}</h2>',
                    '  <table>',
                    '    <thead>',
                    '      <tr><th>Axis</th><th>Samples</th><th>PeakToPeak</th><th>PeakAbs</th><th>Mean</th><th>StdDev</th><th>Min</th><th>Max</th></tr>',
                    '    </thead>',
                    '    <tbody>',
                    *rows,
                    '    </tbody>',
                    '  </table>',
                    '</section>',
                ]
            )
        )

    html = "\n".join(
        [
            '<!DOCTYPE html>',
            '<html lang="en">',
            '<head>',
            '  <meta charset="utf-8"/>',
            '  <meta name="viewport" content="width=device-width, initial-scale=1"/>',
            '  <title>Magnetic Sensor Statistics</title>',
            '  <style>',
            '    body { font-family: Arial, sans-serif; margin: 0; background: #f5f7fb; color: #111827; }',
            '    header { padding: 24px 32px; background: #0f172a; color: white; }',
            '    header h1 { margin: 0 0 8px 0; font-size: 28px; }',
            '    header p { margin: 0; color: #cbd5e1; }',
            '    main { padding: 24px; display: grid; gap: 20px; }',
            '    .summary { display: grid; grid-template-columns: repeat(3, minmax(180px, 1fr)); gap: 16px; }',
            '    .stat-card, .sensor-block { background: white; border-radius: 14px; padding: 18px; box-shadow: 0 8px 24px rgba(15, 23, 42, 0.08); }',
            '    .stat-card h2, .sensor-block h2 { margin: 0 0 12px 0; font-size: 20px; }',
            '    .stat-card .value { font-size: 28px; font-weight: 700; }',
            '    table { width: 100%; border-collapse: collapse; }',
            '    th, td { padding: 10px 12px; border-bottom: 1px solid #e5e7eb; text-align: right; }',
            '    th:first-child, td:first-child { text-align: left; }',
            '    thead th { background: #f8fafc; }',
            '    @media (max-width: 900px) { .summary { grid-template-columns: 1fr; } }',
            '  </style>',
            '</head>',
            '<body>',
            '  <header>',
            '    <h1>Magnetic Sensor Statistics</h1>',
            '    <p>Automatic report with peak, mean, standard deviation, minimum, and maximum for every channel</p>',
            '  </header>',
            '  <main>',
            '    <section class="summary">',
            '      <div class="stat-card"><h2>Sensor Count</h2><div class="value">12</div></div>',
            f'      <div class="stat-card"><h2>Sample Count</h2><div class="value">{len(times)}</div></div>',
            f'      <div class="stat-card"><h2>Duration</h2><div class="value">{duration:.2f} s</div></div>',
            '    </section>',
            *sections,
            '  </main>',
            '</body>',
            '</html>',
        ]
    )
    output_path.write_text(html, encoding="utf-8")
    safe_print(f"Saved {output_path}")


def save_single_run_outputs(times: list[float], sensor_series: list[list[list[float]]], output_dir: Path, csv_path: Path) -> None:
    save_csv(times, sensor_series, csv_path)
    save_plots(times, sensor_series, output_dir)
    save_overview_html(output_dir)
    save_stats_report(times, sensor_series, output_dir)


def prompt_before_run(run_index: int, run_count: int) -> None:
    safe_print("")
    safe_print(f"Run {run_index:02d}/{run_count:02d}: reflash firmware, then press Enter to start capture.")
    try:
        input()
    except EOFError as exc:
        raise RuntimeError("stdin is not available for run confirmation") from exc


def resolve_baudrate(port: str, baudrate_arg: str) -> int:
    ports = list_ports()
    safe_print(f"Detected ports: {ports}")
    if port not in ports:
        raise ValueError(f"{port} not found")

    if baudrate_arg == "auto":
        mode, baudrate = detect_protocol(port, AUTO_BAUDRATES, timeout_s=AUTO_BAUD_DETECT_TIMEOUT_S)
        if mode != "frame":
            raise ValueError("failed to detect binary frame protocol on the selected port")
        safe_print(f"Auto-detected mode={mode}, baudrate={baudrate}")
        return baudrate

    return int(baudrate_arg)


def build_run_result(run_name: str, output_dir: Path, csv_path: Path,
                     times: list[float], sensor_series: list[list[list[float]]]) -> dict[str, object]:
    return {
        "run_name": run_name,
        "output_dir": output_dir,
        "csv_path": csv_path,
        "times": times,
        "sensor_series": sensor_series,
    }


def compute_multi_run_stability(run_results: list[dict[str, object]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for sensor_idx in range(SENSOR_COUNT):
        sensor_id = sensor_idx + 1
        for axis_idx, (axis_name, _) in enumerate(CHANNEL_COLORS):
            run_means: list[float] = []
            run_stddevs: list[float] = []
            run_peak_to_peaks: list[float] = []
            run_samples: list[int] = []
            run_means_text: list[str] = []
            run_stddevs_text: list[str] = []
            run_peak_to_peaks_text: list[str] = []

            for run_result in run_results:
                run_name = str(run_result["run_name"])
                sensor_series = run_result["sensor_series"]
                channel_values = sensor_series[sensor_idx][axis_idx]
                stats = compute_channel_stats(channel_values)
                run_means.append(stats["mean"])
                run_stddevs.append(stats["stddev"])
                run_peak_to_peaks.append(stats["peak_to_peak"])
                run_samples.append(int(stats["samples"]))
                run_means_text.append(f"{run_name}:{stats['mean']:.6f}")
                run_stddevs_text.append(f"{run_name}:{stats['stddev']:.6f}")
                run_peak_to_peaks_text.append(f"{run_name}:{stats['peak_to_peak']:.6f}")

            run_count = len(run_means)
            mean_of_means = math.fsum(run_means) / run_count
            mean_variance = math.fsum((value - mean_of_means) * (value - mean_of_means) for value in run_means) / run_count
            rows.append(
                {
                    "sensor_id": sensor_id,
                    "axis": axis_name,
                    "run_count": run_count,
                    "mean_of_means": mean_of_means,
                    "mean_span": max(run_means) - min(run_means),
                    "mean_stddev": math.sqrt(max(mean_variance, 0.0)),
                    "avg_in_run_stddev": math.fsum(run_stddevs) / run_count,
                    "max_in_run_stddev": max(run_stddevs),
                    "avg_peak_to_peak": math.fsum(run_peak_to_peaks) / run_count,
                    "max_peak_to_peak": max(run_peak_to_peaks),
                    "min_samples": min(run_samples),
                    "max_samples": max(run_samples),
                    "run_means": " | ".join(run_means_text),
                    "run_stddevs": " | ".join(run_stddevs_text),
                    "run_peak_to_peaks": " | ".join(run_peak_to_peaks_text),
                }
            )
    return rows


def save_run_manifest(run_results: list[dict[str, object]], output_dir: Path) -> None:
    output_path = output_dir / "run_manifest.csv"
    output_dir.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["run_name", "sample_count", "duration_s", "csv_path", "overview_html", "stats_report_html"])
        for run_result in run_results:
            run_name = str(run_result["run_name"])
            times = run_result["times"]
            csv_path = Path(run_result["csv_path"])
            run_output_dir = Path(run_result["output_dir"])
            writer.writerow(
                [
                    run_name,
                    len(times),
                    f"{compute_capture_duration(times):.6f}",
                    csv_path.as_posix(),
                    (run_output_dir / "overview.html").as_posix(),
                    (run_output_dir / "stats_report.html").as_posix(),
                ]
            )
    safe_print(f"Saved {output_path}")


def save_multi_run_overview_html(run_results: list[dict[str, object]], output_dir: Path) -> None:
    output_path = output_dir / "overview.html"
    cards: list[str] = []
    total_samples = 0
    for run_result in run_results:
        run_name = str(run_result["run_name"])
        run_output_dir = Path(run_result["output_dir"])
        times = run_result["times"]
        sample_count = len(times)
        total_samples += sample_count
        duration = compute_capture_duration(times)
        overview_path = (run_output_dir / "overview.html").relative_to(output_dir).as_posix()
        stats_path = (run_output_dir / "stats_report.html").relative_to(output_dir).as_posix()
        csv_path = Path(run_result["csv_path"]).relative_to(output_dir).as_posix()
        cards.append(
            "\n".join(
                [
                    '<section class="card">',
                    f'  <h2>{run_name}</h2>',
                    '  <dl class="metrics">',
                    f'    <div><dt>Samples</dt><dd>{sample_count}</dd></div>',
                    f'    <div><dt>Duration</dt><dd>{duration:.2f} s</dd></div>',
                    '  </dl>',
                    '  <div class="links">',
                    f'    <a href="{overview_path}">Plots</a>',
                    f'    <a href="{stats_path}">Run Stats</a>',
                    f'    <a href="{csv_path}">CSV</a>',
                    '  </div>',
                    '</section>',
                ]
            )
        )

    html = "\n".join(
        [
            '<!DOCTYPE html>',
            '<html lang="en">',
            '<head>',
            '  <meta charset="utf-8"/>',
            '  <meta name="viewport" content="width=device-width, initial-scale=1"/>',
            '  <title>Magnetic Sensor Multi-Run Overview</title>',
            '  <style>',
            '    body { font-family: Arial, sans-serif; margin: 0; background: #f5f7fb; color: #1f2937; }',
            '    header { padding: 24px 32px; background: #111827; color: white; }',
            '    header h1 { margin: 0 0 8px 0; font-size: 28px; }',
            '    header p { margin: 0; color: #d1d5db; }',
            '    main { padding: 24px; display: grid; gap: 20px; }',
            '    .summary { display: grid; grid-template-columns: repeat(3, minmax(180px, 1fr)); gap: 16px; }',
            '    .card, .summary-card { background: white; border-radius: 14px; padding: 18px; box-shadow: 0 8px 24px rgba(15, 23, 42, 0.08); }',
            '    .summary-card h2, .card h2 { margin: 0 0 12px 0; font-size: 20px; }',
            '    .summary-card .value { font-size: 28px; font-weight: 700; }',
            '    .runs { display: grid; grid-template-columns: repeat(2, minmax(320px, 1fr)); gap: 20px; }',
            '    .metrics { margin: 0 0 16px 0; display: grid; grid-template-columns: repeat(2, minmax(120px, 1fr)); gap: 12px; }',
            '    .metrics div { background: #f8fafc; padding: 10px 12px; border-radius: 10px; }',
            '    .metrics dt { font-size: 12px; text-transform: uppercase; color: #64748b; }',
            '    .metrics dd { margin: 6px 0 0 0; font-size: 20px; font-weight: 700; }',
            '    .links { display: flex; gap: 12px; flex-wrap: wrap; }',
            '    .links a { color: #2563eb; text-decoration: none; font-weight: 600; }',
            '    .links a:hover { text-decoration: underline; }',
            '    @media (max-width: 1100px) { .runs, .summary { grid-template-columns: 1fr; } }',
            '  </style>',
            '</head>',
            '<body>',
            '  <header>',
            '    <h1>Magnetic Sensor Multi-Run Overview</h1>',
            '    <p>Repeated no-field capture overview with links to per-run plots, statistics, and CSV files</p>',
            '  </header>',
            '  <main>',
            '    <section class="summary">',
            f'      <div class="summary-card"><h2>Run Count</h2><div class="value">{len(run_results)}</div></div>',
            f'      <div class="summary-card"><h2>Total Samples</h2><div class="value">{total_samples}</div></div>',
            '      <div class="summary-card"><h2>Reports</h2><div class="value"><a href="stability_report.html">Open</a></div></div>',
            '    </section>',
            '    <section class="runs">',
            *cards,
            '    </section>',
            '  </main>',
            '</body>',
            '</html>',
        ]
    )
    output_path.write_text(html, encoding="utf-8")
    safe_print(f"Saved {output_path}")


def save_stability_summary_csv(rows: list[dict[str, object]], output_dir: Path) -> None:
    output_path = output_dir / "stability_summary.csv"
    output_dir.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "sensor_id",
                "axis",
                "run_count",
                "mean_of_means",
                "mean_span",
                "mean_stddev",
                "avg_in_run_stddev",
                "max_in_run_stddev",
                "avg_peak_to_peak",
                "max_peak_to_peak",
                "min_samples",
                "max_samples",
                "run_means",
                "run_stddevs",
                "run_peak_to_peaks",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row["sensor_id"],
                    row["axis"],
                    row["run_count"],
                    f"{row['mean_of_means']:.9f}",
                    f"{row['mean_span']:.9f}",
                    f"{row['mean_stddev']:.9f}",
                    f"{row['avg_in_run_stddev']:.9f}",
                    f"{row['max_in_run_stddev']:.9f}",
                    f"{row['avg_peak_to_peak']:.9f}",
                    f"{row['max_peak_to_peak']:.9f}",
                    row["min_samples"],
                    row["max_samples"],
                    row["run_means"],
                    row["run_stddevs"],
                    row["run_peak_to_peaks"],
                ]
            )
    safe_print(f"Saved {output_path}")


def save_stability_report_html(run_results: list[dict[str, object]], rows: list[dict[str, object]], output_dir: Path) -> None:
    output_path = output_dir / "stability_report.html"
    worst_mean_span = max(rows, key=lambda row: row["mean_span"])
    worst_in_run_stddev = max(rows, key=lambda row: row["max_in_run_stddev"])
    worst_peak_to_peak = max(rows, key=lambda row: row["max_peak_to_peak"])
    total_samples = sum(len(run_result["times"]) for run_result in run_results)
    table_rows: list[str] = []
    for row in rows:
        table_rows.append(
            "\n".join(
                [
                    '      <tr>',
                    f'        <td>Sensor {row["sensor_id"]:02d}</td>',
                    f'        <td>{row["axis"]}</td>',
                    f'        <td>{row["run_count"]}</td>',
                    f'        <td>{row["mean_of_means"]:.6f}</td>',
                    f'        <td>{row["mean_span"]:.6f}</td>',
                    f'        <td>{row["mean_stddev"]:.6f}</td>',
                    f'        <td>{row["avg_in_run_stddev"]:.6f}</td>',
                    f'        <td>{row["max_in_run_stddev"]:.6f}</td>',
                    f'        <td>{row["avg_peak_to_peak"]:.6f}</td>',
                    f'        <td>{row["max_peak_to_peak"]:.6f}</td>',
                    f'        <td>{row["min_samples"]}</td>',
                    f'        <td>{row["max_samples"]}</td>',
                    f'        <td>{escape_xml(str(row["run_means"]))}</td>',
                    f'        <td>{escape_xml(str(row["run_stddevs"]))}</td>',
                    '      </tr>',
                ]
            )
        )

    html = "\n".join(
        [
            '<!DOCTYPE html>',
            '<html lang="en">',
            '<head>',
            '  <meta charset="utf-8"/>',
            '  <meta name="viewport" content="width=device-width, initial-scale=1"/>',
            '  <title>Magnetic Sensor Stability Report</title>',
            '  <style>',
            '    body { font-family: Arial, sans-serif; margin: 0; background: #f5f7fb; color: #111827; }',
            '    header { padding: 24px 32px; background: #0f172a; color: white; }',
            '    header h1 { margin: 0 0 8px 0; font-size: 28px; }',
            '    header p { margin: 0; color: #cbd5e1; }',
            '    main { padding: 24px; display: grid; gap: 20px; }',
            '    .summary { display: grid; grid-template-columns: repeat(3, minmax(220px, 1fr)); gap: 16px; }',
            '    .stat-card, .table-card { background: white; border-radius: 14px; padding: 18px; box-shadow: 0 8px 24px rgba(15, 23, 42, 0.08); }',
            '    .stat-card h2, .table-card h2 { margin: 0 0 12px 0; font-size: 20px; }',
            '    .stat-card .value { font-size: 28px; font-weight: 700; }',
            '    .label { color: #64748b; font-size: 14px; margin-top: 6px; }',
            '    .table-wrap { overflow-x: auto; }',
            '    table { width: 100%; border-collapse: collapse; min-width: 1500px; }',
            '    th, td { padding: 10px 12px; border-bottom: 1px solid #e5e7eb; text-align: right; vertical-align: top; }',
            '    th:first-child, td:first-child, th:nth-child(2), td:nth-child(2), th:nth-child(13), td:nth-child(13), th:nth-child(14), td:nth-child(14) { text-align: left; }',
            '    thead th { background: #f8fafc; position: sticky; top: 0; }',
            '    @media (max-width: 1100px) { .summary { grid-template-columns: 1fr; } }',
            '  </style>',
            '</head>',
            '<body>',
            '  <header>',
            '    <h1>Magnetic Sensor Stability Report</h1>',
            '    <p>Cross-run stability analysis for repeated no-field captures</p>',
            '  </header>',
            '  <main>',
            '    <section class="summary">',
            f'      <div class="stat-card"><h2>Run Count</h2><div class="value">{len(run_results)}</div><div class="label">Repeated captures included in this analysis</div></div>',
            f'      <div class="stat-card"><h2>Total Samples</h2><div class="value">{total_samples}</div><div class="label">Sum of all accepted frames across runs</div></div>',
            f'      <div class="stat-card"><h2>Worst Mean Span</h2><div class="value">S{worst_mean_span["sensor_id"]:02d} {worst_mean_span["axis"]}</div><div class="label">{worst_mean_span["mean_span"]:.6f}</div></div>',
            f'      <div class="stat-card"><h2>Worst In-Run StdDev</h2><div class="value">S{worst_in_run_stddev["sensor_id"]:02d} {worst_in_run_stddev["axis"]}</div><div class="label">{worst_in_run_stddev["max_in_run_stddev"]:.6f}</div></div>',
            f'      <div class="stat-card"><h2>Worst Peak-To-Peak</h2><div class="value">S{worst_peak_to_peak["sensor_id"]:02d} {worst_peak_to_peak["axis"]}</div><div class="label">{worst_peak_to_peak["max_peak_to_peak"]:.6f}</div></div>',
            '      <div class="stat-card"><h2>Artifacts</h2><div class="value"><a href="overview.html">Open</a></div><div class="label">Run overview, manifest, and per-run reports</div></div>',
            '    </section>',
            '    <section class="table-card">',
            '      <h2>Per-Channel Stability Table</h2>',
            '      <div class="table-wrap">',
            '        <table>',
            '          <thead>',
            '            <tr><th>Sensor</th><th>Axis</th><th>Runs</th><th>MeanOfMeans</th><th>MeanSpan</th><th>MeanStdDev</th><th>AvgInRunStdDev</th><th>MaxInRunStdDev</th><th>AvgPeakToPeak</th><th>MaxPeakToPeak</th><th>MinSamples</th><th>MaxSamples</th><th>RunMeans</th><th>RunStdDevs</th></tr>',
            '          </thead>',
            '          <tbody>',
            *table_rows,
            '          </tbody>',
            '        </table>',
            '      </div>',
            '    </section>',
            '  </main>',
            '</body>',
            '</html>',
        ]
    )
    output_path.write_text(html, encoding="utf-8")
    safe_print(f"Saved {output_path}")


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir)
    csv_path = Path(args.csv_path) if args.csv_path else output_dir / "mag_capture.csv"
    if args.runs < 1:
        safe_print("ERROR: --runs must be at least 1")
        return 1
    if args.input_csv and args.runs != 1:
        safe_print("ERROR: --input-csv cannot be combined with --runs greater than 1")
        return 1
    if args.runs > 1 and args.csv_path:
        safe_print("ERROR: --csv-path is only supported for single-run capture")
        return 1
    if args.input_csv:
        csv_path = Path(args.input_csv)
        try:
            times, sensor_series = load_csv(csv_path)
        except (OSError, ValueError) as exc:
            safe_print(f"ERROR: cannot load CSV {csv_path}: {exc}")
            return 1
        save_plots(times, sensor_series, output_dir)
        save_overview_html(output_dir)
        save_stats_report(times, sensor_series, output_dir)
        safe_print(f"Done. Captured {len(times)} frames.")
        return 0

    try:
        baudrate = resolve_baudrate(args.port, args.baudrate)
    except ValueError as exc:
        safe_print(f"ERROR: {exc}")
        return 1

    if args.runs == 1:
        try:
            times, sensor_series = collect_frames(args.port, baudrate, args.duration)
        except serial.SerialException as exc:
            safe_print(f"ERROR: cannot open {args.port}: {exc}")
            return 1

        if not times:
            safe_print("ERROR: no valid binary frame captured")
            return 2

        save_single_run_outputs(times, sensor_series, output_dir, csv_path)
        safe_print(f"Done. Captured {len(times)} frames.")
        return 0

    run_results: list[dict[str, object]] = []
    for run_index in range(1, args.runs + 1):
        run_name = f"run_{run_index:02d}"
        run_output_dir = output_dir / run_name
        run_csv_path = run_output_dir / "mag_capture.csv"
        if not args.no_prompt_before_run:
            try:
                prompt_before_run(run_index, args.runs)
            except RuntimeError as exc:
                safe_print(f"ERROR: {exc}")
                return 1
            safe_print(f"Starting {run_name}...")
        else:
            safe_print(f"Starting {run_name} without prompt...")
        try:
            times, sensor_series = collect_frames(args.port, baudrate, args.duration)
        except serial.SerialException as exc:
            safe_print(f"ERROR: cannot open {args.port}: {exc}")
            return 1

        if not times:
            safe_print(f"ERROR: no valid binary frame captured for {run_name}")
            return 2

        save_single_run_outputs(times, sensor_series, run_output_dir, run_csv_path)
        run_results.append(build_run_result(run_name, run_output_dir, run_csv_path, times, sensor_series))

    save_run_manifest(run_results, output_dir)
    save_multi_run_overview_html(run_results, output_dir)
    stability_rows = compute_multi_run_stability(run_results)
    save_stability_summary_csv(stability_rows, output_dir)
    save_stability_report_html(run_results, stability_rows, output_dir)
    safe_print(f"Done. Completed {len(run_results)} runs.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
