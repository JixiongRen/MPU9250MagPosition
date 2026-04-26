# -*- coding: utf-8 -*-
"""
Run smoke tests for tools/Python scripts without requiring UART hardware.

What it checks:
- Protocol utilities (`uart_frame.selftest`)
- All UART-dependent scripts support extended payloads via `--selftest`
- Syntax compilation for the entire tools/Python tree
"""

from __future__ import annotations

import compileall
import subprocess
import sys
from pathlib import Path


def run(cmd: list[str]) -> None:
    p = subprocess.run(cmd, text=True)
    if p.returncode != 0:
        raise SystemExit(p.returncode)


def main() -> None:
    root = Path(__file__).resolve().parent

    ok = compileall.compile_dir(str(root), quiet=1)
    if not ok:
        raise SystemExit("compileall failed")

    sys.path.insert(0, str(root))
    from uart_frame import selftest  # noqa: E402

    selftest()

    scripts = [
        root / "main.py",
        root / "calibrate" / "mag_cali.py",
        root / "sequences_rate_check" / "check_sequence_rate.py",
        root / "signal_reception" / "plot_sensor_raw_data.py",
        root / "lms" / "lms_realtime.py",
        root / "lms" / "lms_realtime_adaptive.py",
    ]

    for script in scripts:
        run([sys.executable, str(script), "--selftest"])

    print("ALL SELFTESTS OK")


if __name__ == "__main__":
    main()

