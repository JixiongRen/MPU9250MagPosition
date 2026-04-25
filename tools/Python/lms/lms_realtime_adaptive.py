# -*- coding: utf-8 -*-
"""
Real-time Adaptive Noise Filtering for 12 MPU9250 Sensors

Uses frequency-aware filtering that preserves magnetic field signals
while removing high-frequency sensor noise.

Key Features:
- Preserves low-frequency magnetic signals (0-15 Hz)
- Removes high-frequency noise (>15 Hz)
- Independent filtering for each sensor
- Real-time and batch processing modes
"""

import time
import struct
from datetime import datetime

import numpy as np
import serial
import matplotlib.pyplot as plt
from scipy.io import savemat

from adaptive_noise_filter import MultiSensorAdaptiveFilter, estimate_noise_characteristics


def crc16_ccitt_false_matlab(data: bytes) -> int:
    """CRC16-CCITT-FALSE"""
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


class RealtimeAdaptiveFilter:
    """Real-time adaptive filtering system for 12 sensors"""
    
    def __init__(self, serial_port: str, baud_rate: int,
                 sampling_rate: float = 38.0, signal_cutoff: float = 15.0,
                 filter_order: int = 4, tick_hz: int = 1000):
        """
        Parameters:
            serial_port : str
                Serial port name
            baud_rate : int
                UART baud rate
            sampling_rate : float
                Expected sampling rate in Hz
            signal_cutoff : float
                Cutoff frequency for signal band (Hz)
            filter_order : int
                Butterworth filter order
            tick_hz : int
                FreeRTOS tick frequency
        """
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.tick_hz = tick_hz
        self.sampling_rate = sampling_rate
        self.signal_cutoff = signal_cutoff
        
        self.adaptive_filter = MultiSensorAdaptiveFilter(
            num_sensors=12,
            sampling_rate=sampling_rate,
            signal_cutoff=signal_cutoff,
            filter_order=filter_order
        )
        
        self.FRAME_HEADER = bytes([0xAA, 0x55, 0xAA, 0x55])
        self.NUM_SENSORS = 12
        self.NUM_AXES = 3
        self.PAYLOAD_LEN_EXPECTED = self.NUM_SENSORS * self.NUM_AXES * 4
        self.MAX_PAYLOAD_LEN = 2048  # accept diagnostic extensions, reject corrupted length
        
        self.raw_data_buffer = []
        self.filtered_data_buffer = []
        self.timestamps_buffer = []
        self.tick_timestamps_buffer = []
        
        self.sample_count = 0
        self.crc_fail_count = 0
        self.payload_len_mismatch_count = 0
        self.header_resync_count = 0
        
        self.serial_conn = None
        self.is_running = False
        
    def open_serial(self):
        """Open serial connection"""
        print(f"Opening serial port {self.serial_port}...")
        try:
            self.serial_conn = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=10
            )
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            print("Serial port opened successfully")
        except Exception as e:
            raise RuntimeError(f"Unable to open serial port: {e}") from e
    
    def close_serial(self):
        """Close serial connection"""
        if self.serial_conn is not None:
            self.serial_conn.close()
            print("Serial port closed")
    
    def acquire_and_filter(self, max_samples: int = 1000, max_wait_s: float = 30.0):
        """Acquire data and apply real-time adaptive filtering"""
        self.open_serial()
        
        print("\nStarting real-time adaptive filtering...")
        print(f"Sampling rate: {self.sampling_rate} Hz")
        print(f"Signal cutoff: {self.signal_cutoff} Hz")
        print("Press Ctrl+C to stop acquisition\n")
        
        rx_buf = bytearray()
        self.is_running = True
        open_time = time.perf_counter()
        last_wait_report = open_time
        first_tick = None
        
        try:
            while self.sample_count < max_samples and self.is_running:
                n_avail = self.serial_conn.in_waiting
                if n_avail > 0:
                    new_bytes = self.serial_conn.read(n_avail)
                    rx_buf.extend(new_bytes)
                else:
                    time.sleep(0.005)
                
                while True:
                    header_idx = rx_buf.find(self.FRAME_HEADER)
                    if header_idx == -1:
                        if len(rx_buf) > 4096:
                            rx_buf = rx_buf[-3:]
                        break
                    
                    h = header_idx
                    if h > 0:
                        self.header_resync_count += 1
                        rx_buf = rx_buf[h:]
                    
                    if len(rx_buf) < 10:
                        break
                    
                    payload_len = struct.unpack("<H", rx_buf[4:6])[0]
                    if payload_len > self.MAX_PAYLOAD_LEN:
                        self.payload_len_mismatch_count += 1
                        rx_buf = rx_buf[1:]
                        continue
                    frame_len = 4 + 2 + 4 + int(payload_len) + 2
                    
                    if len(rx_buf) < frame_len:
                        break
                    
                    frame = bytes(rx_buf[:frame_len])
                    rx_buf = rx_buf[frame_len:]
                    
                    if payload_len < self.PAYLOAD_LEN_EXPECTED:
                        self.payload_len_mismatch_count += 1
                        continue
                    
                    crc_rx = struct.unpack("<H", frame[-2:])[0]
                    crc_calc = crc16_ccitt_false_matlab(frame[:-2])
                    if crc_rx != crc_calc:
                        self.crc_fail_count += 1
                        continue
                    
                    tick = struct.unpack("<I", frame[6:10])[0]
                    payload = frame[10:10 + payload_len]
                    values = np.frombuffer(payload[:self.PAYLOAD_LEN_EXPECTED], dtype="<f4")
                    
                    if values.size != self.NUM_SENSORS * self.NUM_AXES:
                        continue

                    if first_tick is None:
                        first_tick = tick
                        print("First valid frame received, acquisition started.")
                    
                    raw_sample = values.reshape((self.NUM_SENSORS, self.NUM_AXES))
                    
                    filtered_sample = self.adaptive_filter.process_sample(raw_sample)
                    
                    self.raw_data_buffer.append(raw_sample.copy())
                    self.filtered_data_buffer.append(filtered_sample.copy())
                    self.tick_timestamps_buffer.append(tick)
                    self.timestamps_buffer.append(float(tick - first_tick) / float(self.tick_hz))
                    
                    self.sample_count += 1
                    
                    if self.sample_count % 50 == 0:
                        print(
                            f"Samples: {self.sample_count} | "
                            f"Time: {self.timestamps_buffer[-1]:.3f}s | "
                            f"Sensor1-Z raw: {raw_sample[0, 2]:.2f} -> "
                            f"filtered: {filtered_sample[0, 2]:.2f}"
                        )
                    
                    if self.sample_count >= max_samples:
                        break

                if self.sample_count == 0:
                    now = time.perf_counter()
                    if now - last_wait_report >= 5.0:
                        print(f"Waiting for first valid frame... {now - open_time:.1f}s")
                        last_wait_report = now
                    if (now - open_time) >= float(max_wait_s):
                        print(f"Timeout waiting for first valid frame ({max_wait_s:.1f}s).")
                        self.is_running = False
                        break
        
        except KeyboardInterrupt:
            print("\nData acquisition stopped: KeyboardInterrupt")
        except Exception as e:
            print(f"\nData acquisition stopped: {e}")
        finally:
            self.close_serial()
        
        print("\n=== Acquisition Summary ===")
        print(f"Total valid samples: {self.sample_count}")
        print(f"CRC failures: {self.crc_fail_count}")
        print(f"Payload length mismatches: {self.payload_len_mismatch_count}")
        print(f"Header resync count: {self.header_resync_count}")
        
        if self.sample_count > 100:
            self._apply_batch_filtering()
            self._analyze_noise_characteristics()
    
    def _apply_batch_filtering(self):
        """Apply batch filtering for better results"""
        print("\n=== Applying Batch Filtering (Zero-Phase) ===")
        
        raw_data = self.get_raw_data()
        
        filtered_data_batch = self.adaptive_filter.process_batch(raw_data)
        
        self.filtered_data_buffer = [filtered_data_batch[i, :, :] for i in range(len(filtered_data_batch))]
        
        print("Batch filtering completed")
    
    def _analyze_noise_characteristics(self):
        """Analyze noise characteristics of the data"""
        print("\n=== Noise Characteristics Analysis ===")
        
        raw_data = self.get_raw_data()
        
        sensor_idx = 0
        axis_idx = 2
        
        signal = raw_data[:, sensor_idx, axis_idx]
        
        noise_info = estimate_noise_characteristics(signal, self.sampling_rate)
        
        print(f"Analysis for Sensor {sensor_idx+1}, Z-axis:")
        print(f"  Dominant signal frequency: {noise_info['signal_freq']:.2f} Hz")
        print(f"  Noise frequency range: {noise_info['noise_freq'][0]:.2f} - {noise_info['noise_freq'][1]:.2f} Hz")
        print(f"  Recommended cutoff: {noise_info['recommended_cutoff']:.2f} Hz")
        print(f"  Current cutoff: {self.signal_cutoff:.2f} Hz")
        
        if noise_info['recommended_cutoff'] < self.signal_cutoff * 0.5:
            print(f"\n  ⚠️  Warning: Current cutoff may be too high!")
            print(f"     Consider using {noise_info['recommended_cutoff']:.1f} Hz")
        elif noise_info['recommended_cutoff'] > self.signal_cutoff * 2:
            print(f"\n  ℹ️  Info: You could increase cutoff to {noise_info['recommended_cutoff']:.1f} Hz")
            print(f"     for better signal preservation")
    
    def get_raw_data(self) -> np.ndarray:
        """Get raw data as numpy array [N, 12, 3]"""
        if len(self.raw_data_buffer) == 0:
            return np.zeros((0, 12, 3), dtype=np.float32)
        return np.array(self.raw_data_buffer, dtype=np.float32)
    
    def get_filtered_data(self) -> np.ndarray:
        """Get filtered data as numpy array [N, 12, 3]"""
        if len(self.filtered_data_buffer) == 0:
            return np.zeros((0, 12, 3), dtype=np.float32)
        return np.array(self.filtered_data_buffer, dtype=np.float32)
    
    def get_timestamps(self) -> np.ndarray:
        """Get timestamps as numpy array [N]"""
        return np.array(self.timestamps_buffer, dtype=np.float64)
    
    def visualize_results(self, sensor_indices: list = None):
        """Visualize filtering results"""
        if self.sample_count == 0:
            print("No data to visualize")
            return
        
        if sensor_indices is None:
            sensor_indices = list(range(min(3, 12)))
        
        raw_data = self.get_raw_data()
        filtered_data = self.get_filtered_data()
        timestamps = self.get_timestamps()
        
        n_sensors = len(sensor_indices)
        fig, axes = plt.subplots(n_sensors, 3, figsize=(15, 3*n_sensors))
        fig.suptitle("Adaptive Filtering Results: Raw (blue) vs Filtered (red)", 
                     fontsize=14, fontweight='bold')
        
        if n_sensors == 1:
            axes = axes.reshape(1, -1)
        
        axis_names = ['X Axis', 'Y Axis', 'Z Axis']
        
        for plot_idx, sensor_idx in enumerate(sensor_indices):
            for axis_idx in range(3):
                ax = axes[plot_idx, axis_idx]
                
                raw_signal = raw_data[:, sensor_idx, axis_idx]
                filtered_signal = filtered_data[:, sensor_idx, axis_idx]
                
                ax.plot(timestamps, raw_signal, 'b-', linewidth=0.8, 
                       alpha=0.6, label='Raw')
                ax.plot(timestamps, filtered_signal, 'r-', linewidth=1.5, 
                       label='Filtered')
                
                ax.set_title(f"Sensor {sensor_idx+1} - {axis_names[axis_idx]}")
                ax.set_xlabel("Time (s)")
                ax.set_ylabel("Magnetic Field")
                ax.grid(True, alpha=0.3)
                ax.legend(loc='upper right')
                
                noise_std = np.std(raw_signal - filtered_signal)
                ax.text(0.02, 0.02, 
                       f"Noise σ: {noise_std:.2f}",
                       transform=ax.transAxes,
                       bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'),
                       fontsize=8)
        
        plt.tight_layout()
        plt.show()
    
    def save_data(self, filename: str = None):
        """Save raw and filtered data to .mat file"""
        if self.sample_count == 0:
            print("No data to save")
            return
        
        if filename is None:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"adaptive_filtered_data_{timestamp_str}.mat"
        
        save_dict = {
            "rawData": self.get_raw_data(),
            "filteredData": self.get_filtered_data(),
            "timestamps": self.get_timestamps(),
            "tickTimestamps": np.array(self.tick_timestamps_buffer, dtype=np.uint32),
            "sampleCount": self.sample_count,
            "samplingRate": self.sampling_rate,
            "signalCutoff": self.signal_cutoff,
        }
        
        savemat(filename, save_dict)
        print(f"Data saved to: {filename}")


def main(
    serial_port: str = "COM7",
    baud_rate: int = 1024000,
    max_samples: int = 3000,
    tick_hz: int = 1000,
    sampling_rate: float = 38.0,
    signal_cutoff: float = 15.0,
    filter_order: int = 4,
    max_wait_s: float = 30.0,
    no_plot: bool = False,
    no_save: bool = False,
):
    """Main function"""
    print("=== Real-time Adaptive Noise Filtering for 12 Sensors ===\n")
    
    SERIAL_PORT = serial_port
    BAUD_RATE = baud_rate
    MAX_SAMPLES = max_samples
    TICK_HZ = tick_hz

    SAMPLING_RATE = sampling_rate
    SIGNAL_CUTOFF = signal_cutoff
    FILTER_ORDER = filter_order
    
    print("Configuration:")
    print(f"  Serial Port: {SERIAL_PORT}")
    print(f"  Baud Rate: {BAUD_RATE}")
    print(f"  Max Samples: {MAX_SAMPLES}")
    print(f"  Sampling Rate: {SAMPLING_RATE} Hz")
    print(f"  Signal Cutoff: {SIGNAL_CUTOFF} Hz (preserves 0-{SIGNAL_CUTOFF} Hz)")
    print(f"  Filter Order: {FILTER_ORDER}\n")
    
    print("📌 Filter Behavior:")
    print(f"  ✓ Preserves: Magnetic field signals (0-{SIGNAL_CUTOFF} Hz)")
    print(f"  ✗ Removes: High-frequency noise (>{SIGNAL_CUTOFF} Hz)")
    print(f"  → Real magnetic field changes will be preserved!\n")
    
    realtime_filter = RealtimeAdaptiveFilter(
        serial_port=SERIAL_PORT,
        baud_rate=BAUD_RATE,
        sampling_rate=SAMPLING_RATE,
        signal_cutoff=SIGNAL_CUTOFF,
        filter_order=FILTER_ORDER,
        tick_hz=TICK_HZ
    )
    
    realtime_filter.acquire_and_filter(max_samples=MAX_SAMPLES, max_wait_s=max_wait_s)
    
    if realtime_filter.sample_count > 0 and (not no_plot):
        print("\n=== Visualization ===")
        
        print("\nGenerating filtering results comparison...")
        realtime_filter.visualize_results(sensor_indices=[0, 1, 2])
        
        if not no_save:
            save_choice = input("\nSave filtered data? (yes/no) [no]: ").strip().lower()
            if save_choice == "yes":
                realtime_filter.save_data()
    
    print("\nProgram completed")


if __name__ == "__main__":
    import argparse
    from pathlib import Path
    import sys

    parser = argparse.ArgumentParser(description="Real-time adaptive noise filtering (UART)")
    parser.add_argument("--selftest", action="store_true", help="Run protocol/decoder self-test and exit")
    parser.add_argument("--port", default="COM7", help="Serial port (e.g. COM5)")
    parser.add_argument("--baud", type=int, default=1024000, help="Baud rate")
    parser.add_argument("--max-samples", type=int, default=3000, help="Max samples to acquire")
    parser.add_argument("--tick-hz", type=int, default=1000, help="FreeRTOS tick rate (Hz)")
    parser.add_argument("--max-wait", type=float, default=30.0, help="Max seconds to wait for first valid frame")
    parser.add_argument("--sampling-rate", type=float, default=38.0, help="Expected sampling rate (Hz)")
    parser.add_argument("--signal-cutoff", type=float, default=15.0, help="Signal cutoff frequency (Hz)")
    parser.add_argument("--filter-order", type=int, default=4, help="Filter order")
    parser.add_argument("--no-plot", action="store_true", help="Skip visualization/GUI")
    parser.add_argument("--no-save", action="store_true", help="Do not prompt/save output")
    args = parser.parse_args()

    if args.selftest:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from uart_frame import build_frame, make_test_payload

        NUM_SENSORS = 12
        NUM_AXES = 3
        PAYLOAD_LEN_EXPECTED = NUM_SENSORS * NUM_AXES * 4

        payload = make_test_payload(extra_len=264)
        frame = build_frame(payload, tick=8)

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
        sampling_rate=args.sampling_rate,
        signal_cutoff=args.signal_cutoff,
        filter_order=args.filter_order,
        max_wait_s=args.max_wait,
        no_plot=args.no_plot,
        no_save=args.no_save,
    )
