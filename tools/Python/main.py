# -*- coding: utf-8 -*-
"""
LMS UART Data Reception and Filtering with Visualization

This script combines UART data acquisition with LMS adaptive filtering
and provides comprehensive visualization of time-domain waveforms and
power spectral density (Welch PSD).

Features:
- Real-time UART data reception with CRC validation
- Independent LMS filtering for 12 sensors
- Time-domain waveform visualization
- Welch power spectral density analysis
- Automatic frame rate detection
- Data quality diagnostics

Usage:
    from lms_uart_filter import acquire_and_filter_uart_data
    
    raw_data, filtered_data, timestamps, info = acquire_and_filter_uart_data(
        serial_port="COM7",
        baud_rate=1024000,
        max_samples=1000
    )
"""

import time
import struct
from datetime import datetime

import numpy as np
import serial
import matplotlib.pyplot as plt
from scipy.signal import welch

# Import LMS processor from lms module
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / 'lms'))
from sensor_lms_processor import MultiSensorLMSProcessor


def crc16_ccitt_false(data: bytes) -> int:
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


def acquire_and_filter_uart_data(
    serial_port: str = "COM7",
    baud_rate: int = 1024000,
    max_samples: int = 1000,
    filter_order: int = 8,
    step_size: float = 0.001,
    reference_strategy: str = 'moving_average',
    reference_window: int = 10,
    tick_hz: int = 1000,
    enable_visualization: bool = True,
    max_wait_s: float = 30.0,
    plots_dir: str | None = None,
    show_plots: bool = True,
):
    """
    Acquire UART data and apply LMS adaptive filtering with visualization.

    Parameters
    ----------
    serial_port : str, optional
        Serial port name (default: "COM7")
    baud_rate : int, optional
        UART baud rate (default: 1024000)
    max_samples : int, optional
        Maximum number of samples to acquire (default: 1000)
    filter_order : int, optional
        LMS filter order (default: 8)
    step_size : float, optional
        LMS step size / learning rate (default: 0.001)
    reference_strategy : str, optional
        Reference signal strategy: 'moving_average', 'delayed', or 'zero'
        (default: 'moving_average')
    reference_window : int, optional
        Moving average window size (default: 10)
    tick_hz : int, optional
        FreeRTOS tick frequency in Hz (default: 1000)
    enable_visualization : bool, optional
        Enable visualization plots (default: True)

    Returns
    -------
    dict
        Dictionary containing:
        - 'raw_data': [N, 12, 3] raw sensor data
        - 'filtered_data': [N, 12, 3] LMS filtered data
        - 'timestamps': [N] timestamps in seconds
        - 'tick_timestamps': [N] raw tick timestamps
        - 'info': dict with acquisition metadata
        - 'sampling_rate': Measured sampling rate in Hz

    Examples
    --------
    >>> result = acquire_and_filter_uart_data(
    ...     serial_port="COM7",
    ...     max_samples=1000
    ... )
    >>> raw_data = result['raw_data']
    >>> filtered_data = result['filtered_data']
    """
    
    # Frame format constants
    FRAME_HEADER = bytes([0xAA, 0x55, 0xAA, 0x55])
    NUM_SENSORS = 12
    NUM_AXES = 3
    PAYLOAD_LEN_EXPECTED = NUM_SENSORS * NUM_AXES * 4
    MAX_PAYLOAD_LEN = 2048  # accept diagnostic extensions, reject corrupted length
    
    print("=" * 70)
    print("LMS UART Data Reception and Filtering")
    print("=" * 70)
    print(f"\nConfiguration:")
    print(f"  Serial Port: {serial_port}")
    print(f"  Baud Rate: {baud_rate}")
    print(f"  Max Samples: {max_samples}")
    print(f"  Filter Order: {filter_order}")
    print(f"  Step Size: {step_size}")
    print(f"  Reference Strategy: {reference_strategy}")
    print(f"  Reference Window: {reference_window}\n")
    
    # Initialize LMS processor
    lms_processor = MultiSensorLMSProcessor(
        num_sensors=NUM_SENSORS,
        filter_order=filter_order,
        step_size=step_size,
        reference_strategy=reference_strategy,
        reference_delay=1,
        reference_window=reference_window
    )
    
    # Open serial port
    print(f"Opening serial port {serial_port}...")
    try:
        s = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=10
        )
        s.reset_input_buffer()
        s.reset_output_buffer()
        print("Serial port opened successfully\n")
    except Exception as e:
        raise RuntimeError(f"Unable to open serial port: {e}") from e
    
    # Data buffers
    raw_data_buffer = []
    filtered_data_buffer = []
    timestamps_buffer = []
    tick_timestamps_buffer = []
    
    # Statistics
    sample_count = 0
    crc_fail_count = 0
    payload_len_mismatch_count = 0
    header_resync_count = 0
    open_time = time.perf_counter()
    last_wait_report = open_time
    first_tick = None
    
    print("Starting data acquisition...")
    print("Press Ctrl+C to stop early\n")
    
    rx_buf = bytearray()
    
    try:
        while sample_count < max_samples:
            n_avail = s.in_waiting
            if n_avail > 0:
                new_bytes = s.read(n_avail)
                rx_buf.extend(new_bytes)
            else:
                time.sleep(0.005)
            
            # Frame parsing loop
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
                
                # Validate payload length
                if payload_len < PAYLOAD_LEN_EXPECTED:
                    payload_len_mismatch_count += 1
                    continue
                
                # Validate CRC
                crc_rx = struct.unpack("<H", frame[-2:])[0]
                crc_calc = crc16_ccitt_false(frame[:-2])
                if crc_rx != crc_calc:
                    crc_fail_count += 1
                    continue
                
                # Extract data
                tick = struct.unpack("<I", frame[6:10])[0]
                payload = frame[10:10 + payload_len]
                values = np.frombuffer(payload[:PAYLOAD_LEN_EXPECTED], dtype="<f4")
                
                if values.size != NUM_SENSORS * NUM_AXES:
                    continue

                if first_tick is None:
                    first_tick = tick
                    print("First valid frame received, acquisition started.")
                
                raw_sample = values.reshape((NUM_SENSORS, NUM_AXES))
                
                # Apply LMS filtering
                filtered_sample = lms_processor.process_sample(raw_sample)
                
                # Store data
                raw_data_buffer.append(raw_sample.copy())
                filtered_data_buffer.append(filtered_sample.copy())
                tick_timestamps_buffer.append(tick)
                timestamps_buffer.append(float(tick - first_tick) / float(tick_hz))
                
                sample_count += 1
                
                # Progress display
                if sample_count % 100 == 0:
                    print(
                        f"Samples: {sample_count:4d} | "
                        f"Time: {timestamps_buffer[-1]:8.3f}s | "
                        f"Sensor1-Z raw: {raw_sample[0, 2]:7.2f} -> "
                        f"filtered: {filtered_sample[0, 2]:7.2f}"
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
    
    # Convert to numpy arrays
    raw_data = np.array(raw_data_buffer, dtype=np.float32)
    filtered_data = np.array(filtered_data_buffer, dtype=np.float32)
    timestamps = np.array(timestamps_buffer, dtype=np.float64)
    tick_timestamps = np.array(tick_timestamps_buffer, dtype=np.uint32)
    
    # Calculate sampling rate
    if len(timestamps) > 1:
        total_duration = timestamps[-1] - timestamps[0]
        if total_duration > 0:
            sampling_rate = (len(timestamps) - 1) / total_duration
        else:
            sampling_rate = np.nan
    else:
        sampling_rate = np.nan
    
    # Print summary
    print("\n" + "=" * 70)
    print("Acquisition Summary")
    print("=" * 70)
    print(f"Total valid samples: {sample_count}")
    print(f"CRC failures: {crc_fail_count}")
    print(f"Payload length mismatches: {payload_len_mismatch_count}")
    print(f"Header resync count: {header_resync_count}")
    print(f"Average sampling rate: {sampling_rate:.2f} Hz")
    if len(timestamps) > 1:
        print(f"Total duration: {timestamps[-1] - timestamps[0]:.2f} s")
    
    # Create info dictionary
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
        "samplingRate": sampling_rate,
        "filterOrder": filter_order,
        "stepSize": step_size,
        "referenceStrategy": reference_strategy,
    }
    
    # Visualization
    if enable_visualization and sample_count > 0:
        _visualize_results(
            raw_data, filtered_data, timestamps, sampling_rate, info,
            save_dir=plots_dir, show_plots=show_plots,
        )
    
    # Return results
    return {
        'raw_data': raw_data,
        'filtered_data': filtered_data,
        'timestamps': timestamps,
        'tick_timestamps': tick_timestamps,
        'info': info,
        'sampling_rate': sampling_rate
    }


def _visualize_results(raw_data, filtered_data, timestamps, sampling_rate, info, save_dir=None, show_plots: bool = True):
    """
    Generate visualization plots for raw and filtered data.
    
    Parameters
    ----------
    raw_data : ndarray
        [N, 12, 3] raw sensor data
    filtered_data : ndarray
        [N, 12, 3] filtered sensor data
    timestamps : ndarray
        [N] timestamps in seconds
    sampling_rate : float
        Measured sampling rate in Hz
    info : dict
        Acquisition metadata
    save_dir : str, optional
        Directory to save plots (default: project root)
    """
    
    if save_dir is None:
        save_dir = str(Path(__file__).parent)
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    
    print("\n" + "=" * 70)
    print("Generating Visualizations")
    print("=" * 70)
    print(f"Plots will be saved to: {save_dir}")
    
    num_samples = raw_data.shape[0]
    num_sensors = raw_data.shape[1]
    
    # Figure 1: Time-domain comparison for all sensors (Z-axis)
    print("\nGenerating Figure 1: Z-Axis Time-Domain Waveforms...")
    fig1 = plt.figure("Z-Axis Time-Domain: Raw vs Filtered", figsize=(16, 10))
    fig1.suptitle("Z-Axis Time-Domain Waveforms: Raw (Blue) vs Filtered (Red)", 
                  fontsize=14, fontweight='bold')
    
    for sensor_idx in range(num_sensors):
        ax = fig1.add_subplot(3, 4, sensor_idx + 1)
        
        raw_z = raw_data[:, sensor_idx, 2]
        filtered_z = filtered_data[:, sensor_idx, 2]
        
        ax.plot(timestamps, raw_z, 'b-', linewidth=0.8, alpha=0.6, label='Raw')
        ax.plot(timestamps, filtered_z, 'r-', linewidth=1.2, label='Filtered')
        
        ax.set_title(f"Sensor {sensor_idx + 1}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Magnetic Field")
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
        
        # Statistics
        raw_mean = np.mean(raw_z)
        raw_std = np.std(raw_z)
        filtered_mean = np.mean(filtered_z)
        filtered_std = np.std(filtered_z)
        
        stats_text = (
            f"Raw: μ={raw_mean:.1f}, σ={raw_std:.2f}\n"
            f"Filt: μ={filtered_mean:.1f}, σ={filtered_std:.2f}"
        )
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
                verticalalignment='top', fontsize=7,
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'))
    
    plt.tight_layout()
    
    # Save Figure 1
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    fig1_path = Path(save_dir) / f"01_Z_Axis_TimeDomain_{timestamp_str}.png"
    fig1.savefig(fig1_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {fig1_path.name}")
    
    # Figure 2: Welch PSD for all sensors (Z-axis)
    print("Generating Figure 2: Z-Axis Welch PSD...")
    fig2 = plt.figure("Z-Axis Welch PSD: Raw vs Filtered", figsize=(16, 10))
    fig2.suptitle("Z-Axis Welch Power Spectral Density: Raw (Blue) vs Filtered (Red)", 
                  fontsize=14, fontweight='bold')
    
    for sensor_idx in range(num_sensors):
        ax = fig2.add_subplot(3, 4, sensor_idx + 1)
        
        raw_z = raw_data[:, sensor_idx, 2]
        filtered_z = filtered_data[:, sensor_idx, 2]
        
        if num_samples >= 8 and np.isfinite(sampling_rate) and sampling_rate > 0:
            # Welch PSD parameters
            nfft = min(256, max(64, int(2 ** np.ceil(np.log2(num_samples)))))
            window_len = min(256, num_samples)
            overlap_len = window_len // 2
            
            # Raw signal PSD
            f_raw, pxx_raw = welch(
                raw_z, fs=sampling_rate, window='hamming',
                nperseg=window_len, noverlap=overlap_len, nfft=nfft,
                detrend=False, return_onesided=True, scaling='density'
            )
            
            # Filtered signal PSD
            f_filt, pxx_filt = welch(
                filtered_z, fs=sampling_rate, window='hamming',
                nperseg=window_len, noverlap=overlap_len, nfft=nfft,
                detrend=False, return_onesided=True, scaling='density'
            )
            
            ax.plot(f_raw, 10 * np.log10(pxx_raw + np.finfo(float).eps), 
                   'b-', linewidth=0.8, label='Raw')
            ax.plot(f_filt, 10 * np.log10(pxx_filt + np.finfo(float).eps), 
                   'r-', linewidth=1.2, label='Filtered')
            
            ax.set_xlabel("Frequency (Hz)")
            ax.set_ylabel("PSD (dB/Hz)")
            ax.set_title(f"Sensor {sensor_idx + 1}")
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize=8)
        else:
            ax.text(0.5, 0.5, "Not enough data", transform=ax.transAxes,
                   horizontalalignment='center', verticalalignment='center')
            ax.set_title(f"Sensor {sensor_idx + 1}")
            ax.set_xlabel("Frequency (Hz)")
            ax.set_ylabel("PSD (dB/Hz)")
    
    plt.tight_layout()
    
    # Save Figure 2
    fig2_path = Path(save_dir) / f"02_Z_Axis_Welch_PSD_{timestamp_str}.png"
    fig2.savefig(fig2_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {fig2_path.name}")
    
    # Figure 3: Sensor 1 three-axis time-domain
    print("Generating Figure 3: Sensor 1 Three-Axis Time-Domain...")
    fig3 = plt.figure("Sensor 1 Three-Axis Time-Domain", figsize=(14, 8))
    fig3.suptitle("Sensor 1: Three-Axis Time-Domain (Raw vs Filtered)", 
                  fontsize=14, fontweight='bold')
    
    axis_names = ['X Axis', 'Y Axis', 'Z Axis']
    colors_raw = ['#1f77b4', '#1f77b4', '#1f77b4']  # Blue
    colors_filt = ['#d62728', '#d62728', '#d62728']  # Red
    
    for axis_idx in range(3):
        ax = fig3.add_subplot(3, 1, axis_idx + 1)
        
        raw_data_axis = raw_data[:, 0, axis_idx]
        filtered_data_axis = filtered_data[:, 0, axis_idx]
        
        ax.plot(timestamps, raw_data_axis, color=colors_raw[axis_idx], 
               linewidth=0.8, alpha=0.6, label='Raw')
        ax.plot(timestamps, filtered_data_axis, color=colors_filt[axis_idx], 
               linewidth=1.2, label='Filtered')
        
        ax.set_title(f"Sensor 1 - {axis_names[axis_idx]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Magnetic Field")
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        
        # Statistics
        raw_mean = np.mean(raw_data_axis)
        raw_std = np.std(raw_data_axis)
        filtered_mean = np.mean(filtered_data_axis)
        filtered_std = np.std(filtered_data_axis)
        
        stats_text = (
            f"Raw: μ={raw_mean:.2f}, σ={raw_std:.3f} | "
            f"Filtered: μ={filtered_mean:.2f}, σ={filtered_std:.3f}"
        )
        ax.text(0.02, 0.95, stats_text, transform=ax.transAxes,
                verticalalignment='top', fontsize=9,
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'))
    
    plt.tight_layout()
    
    # Save Figure 3
    fig3_path = Path(save_dir) / f"03_Sensor1_ThreeAxis_TimeDomain_{timestamp_str}.png"
    fig3.savefig(fig3_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {fig3_path.name}")
    
    # Figure 4: Sensor 1 three-axis Welch PSD
    print("Generating Figure 4: Sensor 1 Three-Axis Welch PSD...")
    fig4 = plt.figure("Sensor 1 Three-Axis Welch PSD", figsize=(14, 8))
    fig4.suptitle("Sensor 1: Three-Axis Welch Power Spectral Density", 
                  fontsize=14, fontweight='bold')
    
    for axis_idx in range(3):
        ax = fig4.add_subplot(3, 1, axis_idx + 1)
        
        raw_data_axis = raw_data[:, 0, axis_idx]
        filtered_data_axis = filtered_data[:, 0, axis_idx]
        
        if num_samples >= 8 and np.isfinite(sampling_rate) and sampling_rate > 0:
            nfft = min(256, max(64, int(2 ** np.ceil(np.log2(num_samples)))))
            window_len = min(256, num_samples)
            overlap_len = window_len // 2
            
            f_raw, pxx_raw = welch(
                raw_data_axis, fs=sampling_rate, window='hamming',
                nperseg=window_len, noverlap=overlap_len, nfft=nfft,
                detrend=False, return_onesided=True, scaling='density'
            )
            
            f_filt, pxx_filt = welch(
                filtered_data_axis, fs=sampling_rate, window='hamming',
                nperseg=window_len, noverlap=overlap_len, nfft=nfft,
                detrend=False, return_onesided=True, scaling='density'
            )
            
            ax.plot(f_raw, 10 * np.log10(pxx_raw + np.finfo(float).eps), 
                   'b-', linewidth=0.8, label='Raw')
            ax.plot(f_filt, 10 * np.log10(pxx_filt + np.finfo(float).eps), 
                   'r-', linewidth=1.2, label='Filtered')
        
        ax.set_title(f"Sensor 1 - {axis_names[axis_idx]} PSD")
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("PSD (dB/Hz)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
    
    plt.tight_layout()
    
    # Save Figure 4
    fig4_path = Path(save_dir) / f"04_Sensor1_ThreeAxis_Welch_PSD_{timestamp_str}.png"
    fig4.savefig(fig4_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {fig4_path.name}")

    # Figure 5: Sensor 8 three-axis time-domain
    print("Generating Figure 5: Sensor 8 Three-Axis Time-Domain...")
    fig5 = plt.figure("Sensor 8 Three-Axis Time-Domain", figsize=(14, 8))
    fig5.suptitle("Sensor 8: Three-Axis Time-Domain (Raw vs Filtered)",
                  fontsize=14, fontweight='bold')

    sensor_8_idx = 11  # Sensor 8 uses zero-based index 7

    for axis_idx in range(3):
        ax = fig5.add_subplot(3, 1, axis_idx + 1)

        raw_data_axis = raw_data[:, sensor_8_idx, axis_idx]
        filtered_data_axis = filtered_data[:, sensor_8_idx, axis_idx]

        ax.plot(timestamps, raw_data_axis, color=colors_raw[axis_idx],
               linewidth=0.8, alpha=0.6, label='Raw')
        ax.plot(timestamps, filtered_data_axis, color=colors_filt[axis_idx],
               linewidth=1.2, label='Filtered')

        ax.set_title(f"Sensor 8 - {axis_names[axis_idx]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Magnetic Field")
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')

        # Statistics
        raw_mean = np.mean(raw_data_axis)
        raw_std = np.std(raw_data_axis)
        filtered_mean = np.mean(filtered_data_axis)
        filtered_std = np.std(filtered_data_axis)

        stats_text = (
            f"Raw: 渭={raw_mean:.2f}, 蟽={raw_std:.3f} | "
            f"Filtered: 渭={filtered_mean:.2f}, 蟽={filtered_std:.3f}"
        )
        ax.text(0.02, 0.95, stats_text, transform=ax.transAxes,
                verticalalignment='top', fontsize=9,
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'))

    plt.tight_layout()

    # Save Figure 5
    fig5_path = Path(save_dir) / f"05_Sensor8_ThreeAxis_TimeDomain_{timestamp_str}.png"
    fig5.savefig(fig5_path, dpi=150, bbox_inches='tight')
    print(f"  Saved: {fig5_path.name}")
    
    print("\n" + "=" * 70)
    print("All plots saved to root directory:")
    print("=" * 70)
    print(f"  01_Z_Axis_TimeDomain_{timestamp_str}.png")
    print(f"  02_Z_Axis_Welch_PSD_{timestamp_str}.png")
    print(f"  03_Sensor1_ThreeAxis_TimeDomain_{timestamp_str}.png")
    print(f"  04_Sensor1_ThreeAxis_Welch_PSD_{timestamp_str}.png")
    print(f"  05_Sensor8_ThreeAxis_TimeDomain_{timestamp_str}.png")
    if show_plots:
        print("\nDisplaying plots...")
        plt.show()
    else:
        plt.close("all")


def main():
    """
    Main function demonstrating usage of acquire_and_filter_uart_data.
    """
    
    # Acquire and filter data
    result = acquire_and_filter_uart_data(
        serial_port="COM7",
        baud_rate=1024000,
        max_samples=1000,
        filter_order=8,
        step_size=0.001,
        reference_strategy='moving_average',
        reference_window=10,
        enable_visualization=True
    )
    
    # Extract results
    raw_data = result['raw_data']
    filtered_data = result['filtered_data']
    timestamps = result['timestamps']
    info = result['info']
    
    print("\n" + "=" * 70)
    print("Data Processing Complete")
    print("=" * 70)
    print(f"Raw data shape: {raw_data.shape}")
    print(f"Filtered data shape: {filtered_data.shape}")
    print(f"Timestamps shape: {timestamps.shape}")
    
    # Optional: Save data
    save_choice = input("\nSave data to .mat file? (yes/no) [no]: ").strip().lower()
    if save_choice == 'yes':
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"lms_uart_data_{timestamp_str}.mat"
        
        from scipy.io import savemat
        save_dict = {
            "rawData": raw_data,
            "filteredData": filtered_data,
            "timestamps": timestamps,
            "tickTimestamps": result['tick_timestamps'],
            "info": info,
        }
        savemat(filename, save_dict)
        print(f"Data saved to: {filename}")
    
    print("\nScript execution completed")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="LMS UART data reception and filtering")
    parser.add_argument("--selftest", action="store_true", help="Run protocol/decoder self-test and exit")
    parser.add_argument("--port", default="COM7", help="Serial port (e.g. COM5)")
    parser.add_argument("--baud", type=int, default=1024000, help="Baud rate")
    parser.add_argument("--max-samples", type=int, default=1000, help="Max samples to acquire")
    parser.add_argument("--tick-hz", type=int, default=1000, help="FreeRTOS tick rate (Hz)")
    parser.add_argument("--max-wait", type=float, default=30.0, help="Max seconds to wait for first valid frame")
    parser.add_argument("--no-vis", action="store_true", help="Disable visualization plots")
    parser.add_argument("--no-show", action="store_true", help="Save plots but do not display GUI windows")
    parser.add_argument("--plots-dir", default="", help="Directory to save plot PNGs (default: tools/Python)")
    parser.add_argument("--no-save", action="store_true", help="Do not prompt/save .mat output")
    args = parser.parse_args()

    if args.selftest:
        from uart_frame import build_frame, make_test_payload

        NUM_SENSORS = 12
        NUM_AXES = 3
        PAYLOAD_LEN_EXPECTED = NUM_SENSORS * NUM_AXES * 4

        payload = make_test_payload(extra_len=264)
        frame = build_frame(payload, tick=42)

        payload_len = struct.unpack("<H", frame[4:6])[0]
        if payload_len < PAYLOAD_LEN_EXPECTED:
            raise SystemExit("SELFTEST FAIL: payload_len too short")

        payload_bytes = frame[10:10 + payload_len]
        values = np.frombuffer(payload_bytes[:PAYLOAD_LEN_EXPECTED], dtype="<f4")
        if values.size != NUM_SENSORS * NUM_AXES:
            raise SystemExit("SELFTEST FAIL: float decode size mismatch")

        values = values.reshape((NUM_SENSORS, NUM_AXES))
        if values.shape != (NUM_SENSORS, NUM_AXES):
            raise SystemExit("SELFTEST FAIL: reshape mismatch")

        print("SELFTEST OK")
        raise SystemExit(0)

    # Run acquisition
    result = acquire_and_filter_uart_data(
        serial_port=args.port,
        baud_rate=args.baud,
        max_samples=args.max_samples,
        filter_order=8,
        step_size=0.001,
        reference_strategy="moving_average",
        reference_window=10,
        tick_hz=args.tick_hz,
        enable_visualization=(not args.no_vis),
        max_wait_s=args.max_wait,
        plots_dir=(args.plots_dir or None),
        show_plots=(not args.no_show),
    )

    raw_data = result["raw_data"]
    filtered_data = result["filtered_data"]
    timestamps = result["timestamps"]
    info = result["info"]

    print("\n" + "=" * 70)
    print("Data Processing Complete")
    print("=" * 70)
    print(f"Raw data shape: {raw_data.shape}")
    print(f"Filtered data shape: {filtered_data.shape}")
    print(f"Timestamps shape: {timestamps.shape}")

    if (not args.no_save) and raw_data.size > 0:
        save_choice = input("\nSave data to .mat file? (yes/no) [no]: ").strip().lower()
        if save_choice == "yes":
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"lms_uart_data_{timestamp_str}.mat"

            from scipy.io import savemat

            save_dict = {
                "rawData": raw_data,
                "filteredData": filtered_data,
                "timestamps": timestamps,
                "tickTimestamps": result["tick_timestamps"],
                "info": info,
            }
            savemat(filename, save_dict)
            print(f"Data saved to: {filename}")

    print("\nScript execution completed")
