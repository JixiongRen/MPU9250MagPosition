import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional

import serial
import serial.tools.list_ports

import re


SECTION_RE = re.compile(r"^\[(INIT-G\d|RUN-G\d)\]\s*$")
TASK_RE = re.compile(r"^\[TASK\]\s+g1=(\d+)\s+g2=(\d+)\s+g3=(\d+)\s*$")
SENSOR_RE = re.compile(
    r"^S(\d+)\s+mpu=0x([0-9A-Fa-f]{2})\s+ak=0x([0-9A-Fa-f]{2})\s+"
    r"init=(\d+)\s+mag=(\d+)\s+ok=(\d+)/(\d+)\s*$"
)

FRAME_HEADER = b"\xAA\x55\xAA\x55"
SENSOR_COUNT = 12
MAG_FLOAT_COUNT = 36
FRAME_PAYLOAD_LEN_V1 = MAG_FLOAT_COUNT * 4
FRAME_PAYLOAD_LEN_V2 = (
    FRAME_PAYLOAD_LEN_V1
    + (3 * 4)
    + (4 * SENSOR_COUNT)
    + (2 * SENSOR_COUNT * 4)
)
FRAME_PAYLOAD_LEN_V3 = (
    FRAME_PAYLOAD_LEN_V1
    + (3 * 4)
    + (8 * SENSOR_COUNT)
    + (2 * SENSOR_COUNT * 4)
)
FRAME_PAYLOAD_LEN_V4 = (
    FRAME_PAYLOAD_LEN_V1
    + (3 * 4)
    + (10 * SENSOR_COUNT)
    + (2 * SENSOR_COUNT * 4)
)
FRAME_PAYLOAD_LEN_V5 = (
    FRAME_PAYLOAD_LEN_V1
    + (3 * 4)
    + (13 * SENSOR_COUNT)
    + (2 * SENSOR_COUNT * 4)
)
FRAME_MIN_TOTAL_LEN = 4 + 2 + 4 + FRAME_PAYLOAD_LEN_V1 + 2
AUTO_BAUDRATES = (115200, 1024000)

INIT_REASON = {
    0: "init ok",
    1: "mpu whoami fail",
    2: "ak8963 whoami fail",
    3: "first mag sample fail",
}

MAG_REASON = {
    0: "mag read ok",
    4: "mag empty",
    5: "mag overflow",
}


def safe_print(text: str) -> None:
    sys.stdout.write(text.encode("utf-8", errors="backslashreplace").decode("utf-8") + "\n")
    sys.stdout.flush()


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    poly = 0x1021
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class SensorStatus:
    sensor_id: int
    mpu_whoami: int
    ak_whoami: int
    init_code: int
    mag_code: int
    success_count: int
    attempt_count: int

    def verdict(self) -> str:
        init_msg = INIT_REASON.get(self.init_code, f"init unknown({self.init_code})")
        mag_msg = MAG_REASON.get(self.mag_code, f"mag unknown({self.mag_code})")
        if self.init_code != 0:
            return f"{init_msg}; mpu=0x{self.mpu_whoami:02X}, ak=0x{self.ak_whoami:02X}"
        if self.attempt_count == 0:
            return "task running but no mag attempts yet"
        if self.success_count == 0:
            return f"{mag_msg}; no successful mag read"
        return f"{mag_msg}; ok {self.success_count}/{self.attempt_count}"


class DiagMonitor:
    def __init__(self) -> None:
        self.current_section = ""
        self.task_counts = (0, 0, 0)
        self.last_task_counts: Optional[tuple[int, int, int]] = None

    def handle_line(self, line: str) -> None:
        section_match = SECTION_RE.match(line)
        if section_match:
            self.current_section = section_match.group(1)
            safe_print(f"\n{self.current_section}")
            return

        task_match = TASK_RE.match(line)
        if task_match:
            self.task_counts = tuple(int(v) for v in task_match.groups())
            self._print_task_summary()
            return

        sensor_match = SENSOR_RE.match(line)
        if sensor_match:
            status = SensorStatus(
                sensor_id=int(sensor_match.group(1)),
                mpu_whoami=int(sensor_match.group(2), 16),
                ak_whoami=int(sensor_match.group(3), 16),
                init_code=int(sensor_match.group(4)),
                mag_code=int(sensor_match.group(5)),
                success_count=int(sensor_match.group(6)),
                attempt_count=int(sensor_match.group(7)),
            )
            safe_print(f"S{status.sensor_id}: {status.verdict()}")
            return

        if line:
            safe_print(f"RAW: {line.encode('unicode_escape').decode('ascii')}")

    def _print_task_summary(self) -> None:
        summary = f"TASK counts g1={self.task_counts[0]} g2={self.task_counts[1]} g3={self.task_counts[2]}"
        if self.last_task_counts is None:
            safe_print(summary)
        else:
            delta = tuple(now - old for now, old in zip(self.task_counts, self.last_task_counts))
            safe_print(f"{summary} delta={delta}")
            if delta == (0, 0, 0):
                safe_print("WARN: task counters not increasing")
        self.last_task_counts = self.task_counts


def list_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


def detect_text_score(sample: bytes) -> int:
    score = 0
    for raw_line in sample.splitlines():
        line = raw_line.decode("utf-8", errors="ignore").strip()
        if SECTION_RE.match(line) or TASK_RE.match(line) or SENSOR_RE.match(line):
            score += 1
    return score


def extract_first_valid_frame(sample: bytes) -> Optional[bytes]:
    for index in range(0, max(0, len(sample) - FRAME_MIN_TOTAL_LEN + 1)):
        if sample[index:index + 4] != FRAME_HEADER:
            continue
        payload_len = struct.unpack_from("<H", sample, index + 4)[0]
        total_len = 4 + 2 + 4 + payload_len + 2
        if payload_len not in (FRAME_PAYLOAD_LEN_V1, FRAME_PAYLOAD_LEN_V2, FRAME_PAYLOAD_LEN_V3, FRAME_PAYLOAD_LEN_V4, FRAME_PAYLOAD_LEN_V5) or index + total_len > len(sample):
            continue
        frame = sample[index:index + total_len]
        expected_crc = struct.unpack_from("<H", frame, total_len - 2)[0]
        actual_crc = crc16_ccitt_false(frame[:-2])
        if expected_crc == actual_crc:
            return frame
    return None


def detect_protocol(port: str, baudrates: tuple[int, ...], timeout_s: float = 8.0) -> tuple[str, int]:
    best_mode = "unknown"
    best_baudrate = baudrates[0]
    best_score = -1

    for baudrate in baudrates:
        try:
            with serial.Serial(port, baudrate, timeout=0.2) as ser:
                ser.reset_input_buffer()
                start = time.time()
                sample = bytearray()
                while time.time() - start < timeout_s:
                    sample.extend(ser.read(512))
        except serial.SerialException:
            continue

        text_score = detect_text_score(bytes(sample))
        frame = extract_first_valid_frame(bytes(sample))
        if frame is not None and best_score < 100:
            best_mode = "frame"
            best_baudrate = baudrate
            best_score = 100
        elif text_score > best_score:
            best_mode = "diag"
            best_baudrate = baudrate
            best_score = text_score

    return best_mode, best_baudrate


def monitor_diag(port: str, baudrate: int, duration: float) -> int:
    monitor = DiagMonitor()
    safe_print(f"Mode: diag text, port={port}, baudrate={baudrate}")
    with serial.Serial(port, baudrate, timeout=0.5) as ser:
        start_time = time.time()
        while time.time() - start_time < duration:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            monitor.handle_line(line)
    return 0


def summarize_frame_v1(payload: bytes, frame_count: int, timestamp: int) -> None:
    values = struct.unpack("<36f", payload)
    zero_like = []
    first_three = []
    for sensor_idx in range(SENSOR_COUNT):
        x, y, z = values[sensor_idx * 3:sensor_idx * 3 + 3]
        if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
            zero_like.append(sensor_idx + 1)
        if sensor_idx < 3:
            first_three.append(f"S{sensor_idx + 1}=({x:.2f},{y:.2f},{z:.2f})")

    safe_print(
        f"FRAME #{frame_count} tick={timestamp} zero_sensors={zero_like} "
        + " ".join(first_three)
    )


def summarize_frame_v2_or_v3(payload: bytes, frame_count: int, timestamp: int) -> None:
    offset = 0
    mag_values = struct.unpack_from("<36f", payload, offset)
    offset += FRAME_PAYLOAD_LEN_V1

    group_counts = struct.unpack_from("<3I", payload, offset)
    offset += 12

    mpu_whoami = list(payload[offset:offset + SENSOR_COUNT])
    offset += SENSOR_COUNT
    ak_whoami = list(payload[offset:offset + SENSOR_COUNT])
    offset += SENSOR_COUNT
    init_error = list(payload[offset:offset + SENSOR_COUNT])
    offset += SENSOR_COUNT
    last_mag_status = list(payload[offset:offset + SENSOR_COUNT])
    offset += SENSOR_COUNT
    last_i2c_status = [0] * SENSOR_COUNT
    last_aux_data = [0] * SENSOR_COUNT
    user_ctrl = [0] * SENSOR_COUNT
    i2c_mst_ctrl = [0] * SENSOR_COUNT
    last_aux_op = [0] * SENSOR_COUNT
    last_slv4_ctrl = [0] * SENSOR_COUNT
    last_slv4_addr = [0] * SENSOR_COUNT
    last_slv4_reg = [0] * SENSOR_COUNT
    last_slv4_do = [0] * SENSOR_COUNT
    if len(payload) >= FRAME_PAYLOAD_LEN_V3:
        last_i2c_status = list(payload[offset:offset + SENSOR_COUNT])
        offset += SENSOR_COUNT
        last_aux_data = list(payload[offset:offset + SENSOR_COUNT])
        offset += SENSOR_COUNT
        user_ctrl = list(payload[offset:offset + SENSOR_COUNT])
        offset += SENSOR_COUNT
        i2c_mst_ctrl = list(payload[offset:offset + SENSOR_COUNT])
        offset += SENSOR_COUNT
    if len(payload) >= FRAME_PAYLOAD_LEN_V4:
        last_aux_op = list(payload[offset:offset + SENSOR_COUNT])
        offset += SENSOR_COUNT
        last_slv4_ctrl = list(payload[offset:offset + SENSOR_COUNT])
        offset += SENSOR_COUNT
    if len(payload) >= FRAME_PAYLOAD_LEN_V5:
        last_slv4_addr = list(payload[offset:offset + SENSOR_COUNT])
        offset += SENSOR_COUNT
        last_slv4_reg = list(payload[offset:offset + SENSOR_COUNT])
        offset += SENSOR_COUNT
        last_slv4_do = list(payload[offset:offset + SENSOR_COUNT])
        offset += SENSOR_COUNT
    success_counts = list(struct.unpack_from("<12I", payload, offset))
    offset += 12 * 4
    attempt_counts = list(struct.unpack_from("<12I", payload, offset))

    zero_like = []
    init_fail = []
    mag_fail = []
    first_three = []
    for sensor_idx in range(SENSOR_COUNT):
        x, y, z = mag_values[sensor_idx * 3:sensor_idx * 3 + 3]
        if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
            zero_like.append(sensor_idx + 1)
        valid_mpu = mpu_whoami[sensor_idx] in (0x71, 0x73)
        if init_error[sensor_idx] != 0 or not valid_mpu or ak_whoami[sensor_idx] != 0x48:
            init_fail.append(
                f"S{sensor_idx + 1}(mpu=0x{mpu_whoami[sensor_idx]:02X},"
                f"ak=0x{ak_whoami[sensor_idx]:02X},init={init_error[sensor_idx]},"
                f"aux=0x{last_i2c_status[sensor_idx]:02X},data=0x{last_aux_data[sensor_idx]:02X},"
                f"uc=0x{user_ctrl[sensor_idx]:02X},mc=0x{i2c_mst_ctrl[sensor_idx]:02X},"
                f"op=0x{last_aux_op[sensor_idx]:02X},s4=0x{last_slv4_ctrl[sensor_idx]:02X},"
                f"a4=0x{last_slv4_addr[sensor_idx]:02X},r4=0x{last_slv4_reg[sensor_idx]:02X},"
                f"d4=0x{last_slv4_do[sensor_idx]:02X})"
            )
        elif last_mag_status[sensor_idx] != 0 or success_counts[sensor_idx] == 0:
            mag_fail.append(
                f"S{sensor_idx + 1}(mag={last_mag_status[sensor_idx]},"
                f"ok={success_counts[sensor_idx]}/{attempt_counts[sensor_idx]})"
            )
        if sensor_idx < 3:
            first_three.append(f"S{sensor_idx + 1}=({x:.2f},{y:.2f},{z:.2f})")

    safe_print(
        f"FRAME #{frame_count} tick={timestamp} groups={group_counts} "
        f"zero_sensors={zero_like} {' '.join(first_three)}"
    )
    if init_fail:
        safe_print("INIT_FAIL: " + ", ".join(init_fail))
    if mag_fail:
        safe_print("MAG_FAIL: " + ", ".join(mag_fail))


def monitor_frames(port: str, baudrate: int, duration: float) -> int:
    buffer = bytearray()
    frame_count = 0
    safe_print(f"Mode: binary frame, port={port}, baudrate={baudrate}")
    with serial.Serial(port, baudrate, timeout=0.2) as ser:
        ser.reset_input_buffer()
        start_time = time.time()
        while time.time() - start_time < duration:
            chunk = ser.read(1024)
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
                if payload_len not in (FRAME_PAYLOAD_LEN_V1, FRAME_PAYLOAD_LEN_V2, FRAME_PAYLOAD_LEN_V3, FRAME_PAYLOAD_LEN_V4, FRAME_PAYLOAD_LEN_V5):
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

                timestamp = struct.unpack_from("<I", frame, 6)[0]
                payload = frame[10:-2]
                frame_count += 1
                if payload_len in (FRAME_PAYLOAD_LEN_V2, FRAME_PAYLOAD_LEN_V3, FRAME_PAYLOAD_LEN_V4, FRAME_PAYLOAD_LEN_V5):
                    summarize_frame_v2_or_v3(payload, frame_count, timestamp)
                else:
                    summarize_frame_v1(payload, frame_count, timestamp)
                del buffer[:total_len]

    if frame_count == 0:
        safe_print("WARN: no valid binary frame decoded")
        return 2
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Auto-detect and monitor MPU9250 serial output.")
    parser.add_argument("--port", default="COM7", help="Serial port, default: COM7")
    parser.add_argument(
        "--baudrate",
        default="auto",
        help="Baudrate or auto. Supported auto candidates: 115200, 1024000",
    )
    parser.add_argument(
        "--mode",
        choices=("auto", "diag", "frame"),
        default="auto",
        help="Protocol mode. Default: auto",
    )
    parser.add_argument("--duration", type=float, default=5.0, help="Monitor duration in seconds")
    return parser.parse_args()


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    args = parse_args()
    ports = list_ports()
    safe_print(f"Detected ports: {ports}")
    if args.port not in ports:
        safe_print(f"ERROR: {args.port} not found")
        return 1

    if args.mode == "auto" or args.baudrate == "auto":
        mode, baudrate = detect_protocol(args.port, AUTO_BAUDRATES)
        if mode == "unknown":
            safe_print("ERROR: failed to detect serial protocol on COM port")
            return 1
        safe_print(f"Auto-detected mode={mode}, baudrate={baudrate}")
    else:
        mode = args.mode
        baudrate = int(args.baudrate)

    try:
        if mode == "diag":
            return monitor_diag(args.port, baudrate, args.duration)
        if mode == "frame":
            return monitor_frames(args.port, baudrate, args.duration)
        safe_print(f"ERROR: unsupported mode {mode}")
        return 1
    except serial.SerialException as exc:
        safe_print(f"ERROR: cannot open {args.port}: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
