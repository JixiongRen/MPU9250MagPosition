# -*- coding: utf-8 -*-
"""
test_harness.py

End-to-end runner for scripts under tools/Python using real UART hardware.

Default behavior is a short "smoke run" for each script so it completes quickly:
- Uses COM port provided by user
- Baud rate is read from firmware (Core/Src/usart.c, huart2.Init.BaudRate) if possible
- Sets matplotlib backend to Agg to avoid blocking GUI windows during automated runs
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path


def _read_uart2_baud_from_firmware(repo_root: Path) -> int | None:
    usart_c = repo_root / "Core" / "Src" / "usart.c"
    if not usart_c.exists():
        return None
    text = usart_c.read_text(encoding="utf-8", errors="ignore")
    m = re.search(r"huart2\.Init\.BaudRate\s*=\s*(\d+)\s*;", text)
    if not m:
        return None
    try:
        return int(m.group(1))
    except ValueError:
        return None


def run(cmd: list[str], *, cwd: Path, env: dict[str, str], timeout_s: float) -> None:
    print("\n$ " + " ".join(cmd))
    p = subprocess.run(cmd, cwd=str(cwd), env=env, timeout=timeout_s)
    if p.returncode != 0:
        raise SystemExit(p.returncode)


def main() -> None:
    this_dir = Path(__file__).resolve().parent
    repo_root = this_dir.parents[1]

    parser = argparse.ArgumentParser(description="Run tools/Python scripts against real UART hardware")
    parser.add_argument("--port", default="COM5", help="Serial port (e.g. COM5)")
    parser.add_argument("--baud", type=int, default=0, help="Baud rate; 0 means read from firmware (USART2)")
    parser.add_argument("--tick-hz", type=int, default=1000, help="FreeRTOS tick frequency (Hz)")
    parser.add_argument("--max-wait", type=float, default=12.0, help="Max seconds to wait for the first valid frame")
    parser.add_argument("--max-samples", type=int, default=200, help="Max samples for sample-based scripts")
    parser.add_argument("--duration", type=float, default=6.0, help="Seconds for duration-based scripts")
    parser.add_argument("--timeout", type=float, default=120.0, help="Per-script subprocess timeout (seconds)")
    args = parser.parse_args()

    baud = args.baud or _read_uart2_baud_from_firmware(repo_root) or 1024000
    print(f"Using serial: port={args.port}, baud={baud} (USART2), tick_hz={args.tick_hz}")

    env = os.environ.copy()
    env.setdefault("MPLBACKEND", "Agg")
    env.setdefault("PYTHONUTF8", "1")

    # Ensure local imports like uart_frame.py are found for all scripts
    env["PYTHONPATH"] = str(this_dir) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")

    python = sys.executable

    # 1) Basic syntax/bytecode compile check
    run([python, str(this_dir / "run_selftests.py")], cwd=this_dir, env=env, timeout_s=args.timeout)

    # 2) UART + frame parsing + filtering scripts
    run(
        [
            python,
            str(this_dir / "main.py"),
            "--port",
            args.port,
            "--baud",
            str(baud),
            "--max-samples",
            str(args.max_samples),
            "--tick-hz",
            str(args.tick_hz),
            "--max-wait",
            str(args.max_wait),
            "--no-vis",
            "--no-save",
        ],
        cwd=this_dir,
        env=env,
        timeout_s=args.timeout,
    )

    run(
        [
            python,
            str(this_dir / "signal_reception" / "plot_sensor_raw_data.py"),
            "--port",
            args.port,
            "--baud",
            str(baud),
            "--max-samples",
            str(args.max_samples),
            "--tick-hz",
            str(args.tick_hz),
            "--max-wait",
            str(args.max_wait),
            "--no-plot",
            "--no-save",
        ],
        cwd=this_dir,
        env=env,
        timeout_s=args.timeout,
    )

    run(
        [
            python,
            str(this_dir / "sequences_rate_check" / "check_sequence_rate.py"),
            "--port",
            args.port,
            "--baud",
            str(baud),
            "--duration",
            str(int(args.duration)),
            "--expected-frame-rate",
            "38",
            "--max-wait",
            str(args.max_wait),
            "--no-plot",
        ],
        cwd=this_dir,
        env=env,
        timeout_s=args.timeout,
    )

    run(
        [
            python,
            str(this_dir / "lms" / "lms_realtime.py"),
            "--port",
            args.port,
            "--baud",
            str(baud),
            "--max-samples",
            str(args.max_samples),
            "--tick-hz",
            str(args.tick_hz),
            "--max-wait",
            str(args.max_wait),
            "--no-plot",
            "--no-save",
        ],
        cwd=this_dir,
        env=env,
        timeout_s=args.timeout,
    )

    run(
        [
            python,
            str(this_dir / "lms" / "lms_realtime_adaptive.py"),
            "--port",
            args.port,
            "--baud",
            str(baud),
            "--max-samples",
            str(args.max_samples),
            "--tick-hz",
            str(args.tick_hz),
            "--max-wait",
            str(args.max_wait),
            "--no-plot",
            "--no-save",
        ],
        cwd=this_dir,
        env=env,
        timeout_s=args.timeout,
    )

    run(
        [
            python,
            str(this_dir / "calibrate" / "mag_cali.py"),
            "--port",
            args.port,
            "--baud",
            str(baud),
            "--duration",
            str(int(max(8.0, float(args.duration)))),
            "--max-wait",
            str(args.max_wait),
            "--no-input",
            "--no-plot",
        ],
        cwd=this_dir,
        env=env,
        timeout_s=max(args.timeout, 180.0),
    )

    print("\nALL UART SCRIPT RUNS COMPLETED")


if __name__ == "__main__":
    main()
