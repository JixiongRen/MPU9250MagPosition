# -*- coding: utf-8 -*-
"""
uart_frame.py

Utilities for the project's UART frame protocol.

Frame format (wire):
    Header        : 0xAA 0x55 0xAA 0x55 (4 bytes)
    Payload length: uint16, little-endian (2 bytes)
    Tick          : uint32, little-endian (4 bytes, FreeRTOS tick)
    Payload       : variable length (payload_len bytes)
    CRC16         : uint16, little-endian (2 bytes, CRC16-CCITT-FALSE over header..payload)

Notes:
- Firmware may append diagnostic fields after the first 12*3 float32 magnetometer values.
  Python tools should decode the first 144 bytes as magnetometer data and ignore the rest.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


FRAME_HEADER = bytes([0xAA, 0x55, 0xAA, 0x55])
NUM_SENSORS_DEFAULT = 12
NUM_AXES_DEFAULT = 3
MAG_FLOAT_COUNT_DEFAULT = NUM_SENSORS_DEFAULT * NUM_AXES_DEFAULT
MAG_PAYLOAD_LEN_DEFAULT = MAG_FLOAT_COUNT_DEFAULT * 4


def crc16_ccitt_false(data: bytes) -> int:
    """
    CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, refin=False, refout=False, xorout=0x0000)
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


def build_frame(payload: bytes, tick: int) -> bytes:
    """
    Build a full UART frame with given payload and tick, including CRC16.
    """
    if not (0 <= tick <= 0xFFFFFFFF):
        raise ValueError("tick must be uint32")
    if len(payload) > 0xFFFF:
        raise ValueError("payload too large for uint16 length")

    out = bytearray()
    out.extend(FRAME_HEADER)
    out.extend(struct.pack("<H", len(payload)))
    out.extend(struct.pack("<I", int(tick)))
    out.extend(payload)
    out.extend(struct.pack("<H", crc16_ccitt_false(bytes(out))))
    return bytes(out)


@dataclass(frozen=True)
class DecodedFrame:
    tick: int
    payload_len: int
    payload: bytes
    mag_values: np.ndarray  # shape: (num_sensors, num_axes), dtype float32
    extra_payload: bytes


def decode_frame(
    frame: bytes,
    *,
    num_sensors: int = NUM_SENSORS_DEFAULT,
    num_axes: int = NUM_AXES_DEFAULT,
    require_crc: bool = True,
) -> DecodedFrame:
    """
    Decode and validate a single UART frame.
    Accepts payload_len >= (num_sensors*num_axes*4) and ignores extra payload bytes.
    """
    if len(frame) < 12:
        raise ValueError("frame too short")
    if frame[:4] != FRAME_HEADER:
        raise ValueError("bad header")

    payload_len = struct.unpack("<H", frame[4:6])[0]
    expected_min = num_sensors * num_axes * 4
    frame_len = 4 + 2 + 4 + int(payload_len) + 2
    if len(frame) != frame_len:
        raise ValueError("frame length mismatch")
    if payload_len < expected_min:
        raise ValueError(f"payload too short: {payload_len} < {expected_min}")

    if require_crc:
        crc_rx = struct.unpack("<H", frame[-2:])[0]
        crc_calc = crc16_ccitt_false(frame[:-2])
        if crc_rx != crc_calc:
            raise ValueError("crc mismatch")

    tick = struct.unpack("<I", frame[6:10])[0]
    payload = frame[10 : 10 + payload_len]

    mag_payload = payload[:expected_min]
    mag_values = np.frombuffer(mag_payload, dtype="<f4").reshape((num_sensors, num_axes))
    extra_payload = payload[expected_min:]

    return DecodedFrame(
        tick=int(tick),
        payload_len=int(payload_len),
        payload=payload,
        mag_values=mag_values,
        extra_payload=extra_payload,
    )


def make_test_payload(
    *,
    num_sensors: int = NUM_SENSORS_DEFAULT,
    num_axes: int = NUM_AXES_DEFAULT,
    extra_len: int = 264,
) -> bytes:
    """
    Create a synthetic payload:
    - First (num_sensors*num_axes) float32 are deterministic ramp values
    - Followed by extra_len bytes (0..255 cycling) to simulate appended diagnostics
    """
    mag_count = num_sensors * num_axes
    mag = (np.arange(mag_count, dtype=np.float32) * 0.5).astype(np.float32)
    extra = bytes([i & 0xFF for i in range(int(extra_len))])
    return mag.tobytes(order="C") + extra


def selftest() -> None:
    """
    Quick protocol smoke test for both legacy (144B) and extended payloads.
    """
    # Legacy payload: exactly 144 bytes (36 float32)
    payload_legacy = make_test_payload(extra_len=0)
    frame_legacy = build_frame(payload_legacy, tick=123)
    d0 = decode_frame(frame_legacy)
    if d0.tick != 123:
        raise AssertionError("tick decode failed (legacy)")
    if d0.payload_len != MAG_PAYLOAD_LEN_DEFAULT:
        raise AssertionError("payload_len decode failed (legacy)")
    if d0.mag_values.shape != (NUM_SENSORS_DEFAULT, NUM_AXES_DEFAULT):
        raise AssertionError("mag shape decode failed (legacy)")
    if d0.extra_payload != b"":
        raise AssertionError("extra payload should be empty (legacy)")

    # Extended payload: mag (144) + extra
    payload_ext = make_test_payload(extra_len=264)
    frame_ext = build_frame(payload_ext, tick=456)
    d1 = decode_frame(frame_ext)
    if d1.tick != 456:
        raise AssertionError("tick decode failed (ext)")
    if d1.payload_len != (MAG_PAYLOAD_LEN_DEFAULT + 264):
        raise AssertionError("payload_len decode failed (ext)")
    if len(d1.extra_payload) != 264:
        raise AssertionError("extra payload length decode failed (ext)")

