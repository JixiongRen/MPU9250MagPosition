# Magnetic Positioning Platform - Python Module

## Overview

This is the Python module for the Magnetic Positioning Platform, a comprehensive system for acquiring, filtering, and analyzing magnetometer data from 12 independent MPU9250 sensors. The system provides real-time UART data reception, adaptive noise filtering, and advanced signal analysis capabilities.

**Key Features:**
- Real-time UART data acquisition from 12 MPU9250 magnetometer sensors
- LMS (Least Mean Squares) adaptive filtering with moving average reference strategy
- Frequency-aware adaptive noise filtering
- Comprehensive visualization (time-domain and frequency-domain analysis)
- Data quality diagnostics and frame rate verification
- Magnetometer calibration system (hard iron and soft iron correction)
- MATLAB-compatible data export

---

## Project Structure

```
Python/
├── main.py                           # Main entry point with integrated workflow
├── README.md                         # This file
├── requirements.txt                  # Python dependencies
│
├── lms/                              # LMS Adaptive Filtering Module
│   ├── lms_filter.py                # Core LMS filter implementation
│   ├── sensor_lms_processor.py      # 12-sensor independent processor
│   ├── lms_realtime.py              # Real-time UART filtering
│   ├── adaptive_noise_filter.py     # Frequency-aware filtering (v1.1+)
│   ├── lms_realtime_adaptive.py     # Real-time adaptive filtering
│   ├── README.md                    # LMS module documentation (Chinese)
│   └── __init__.py                  # Module initialization
│
├── calibrate/                        # Magnetometer Calibration Module
│   ├── mag_cali.py                  # Calibration system
│   └── README.md                    # Calibration documentation (English)
│
├── sequences_rate_check/             # UART Frame Rate Verification
│   ├── check_sequence_rate.py       # Frame rate diagnostic tool
│   └── README.md                    # Sequence check documentation (English)
│
├── signal_reception/                 # Signal Reception and Visualization
│   ├── plot_sensor_raw_data.py      # Raw data acquisition and plotting
│   └── README.md                    # Signal reception documentation (English)
│
└── [Output Files]
    ├── *.png                        # Generated visualization plots
    ├── *.mat                        # MATLAB data files
    └── *.h                          # Embedded C header files (calibration)
```

---

## Quick Start

### Installation

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Verify conda environment (if using Anaconda):**
   ```bash
   conda activate mag_position_env
   ```

### Basic Usage

#### Option 1: Using the Main Script

```bash
python main.py
```

This runs the integrated workflow:
1. Acquires UART data from 12 sensors
2. Applies LMS adaptive filtering
3. Generates 4 visualization plots
4. Saves plots to root directory
5. Optionally saves data to MATLAB format

#### Option 2: Using Individual Modules

**Real-time LMS Filtering:**
```bash
cd lms
python lms_realtime.py
```

**Frequency-Aware Adaptive Filtering (Recommended for Dynamic Scenarios):**
```bash
cd lms
python lms_realtime_adaptive.py
```

**Raw Signal Reception and Analysis:**
```bash
cd signal_reception
python plot_sensor_raw_data.py
```

**UART Frame Rate Verification:**
```bash
cd sequences_rate_check
python check_sequence_rate.py
```

**Magnetometer Calibration:**
```bash
cd calibrate
python mag_cali.py
```

---

## Module Documentation

### 1. LMS Adaptive Filtering (`lms/`)

**Purpose:** Provides independent LMS adaptive filtering for 12 sensors with no cross-dependencies.

**Key Components:**
- `lms_filter.py`: Core LMS algorithm with normalized step size
- `sensor_lms_processor.py`: Multi-sensor processor with reference signal generation
- `lms_realtime.py`: Real-time UART data acquisition and filtering
- `adaptive_noise_filter.py`: Frequency-aware Butterworth filtering (v1.1+)

**Algorithm Update (v1.1):**
- Changed reference signal strategy from `'delayed'` to `'moving_average'`
- Ensures real magnetic field signals are preserved, not filtered as noise
- Supports both static and dynamic magnetic field applications

**Documentation:** See `lms/README.md` (Chinese) for detailed information.

---

### 2. Magnetometer Calibration (`calibrate/`)

**Purpose:** Computes hard iron and soft iron calibration parameters for 12 sensors.

**Features:**
- Real-time data acquisition during figure-eight motion
- Stable ellipsoid fitting algorithm
- Automatic C header file generation for embedded firmware
- 3D visualization of raw and calibrated data
- Data quality diagnostics

**Usage:**
```bash
cd calibrate
python mag_cali.py
```

**Output Files:**
- `mag_calibration_data_YYYYMMDD_HHMMSS.h`: Embedded C header file
- `mag_calibration_YYYYMMDD_HHMMSS.npz`: Python calibration data

**Documentation:** See `calibrate/README.md` for detailed information.

---

### 3. UART Frame Rate Verification (`sequences_rate_check/`)

**Purpose:** Diagnoses UART transmission quality and frame rate stability.

**Features:**
- Automatic dominant sequence increment detection
- Frame loss detection
- Inter-frame timing analysis
- Four-plot visualization
- No assumptions about sequence increment value

**Usage:**
```bash
cd sequences_rate_check
python check_sequence_rate.py
```

**Key Metrics:**
- Actual UART frame rate (Hz)
- Sequence increment consistency
- Estimated lost frames
- Inter-frame interval statistics

**Documentation:** See `sequences_rate_check/README.md` for detailed information.

---

### 4. Signal Reception and Visualization (`signal_reception/`)

**Purpose:** Acquires raw magnetometer data and provides comprehensive visualization.

**Features:**
- Real-time UART data reception with CRC validation
- Time-domain waveform visualization (12 sensors)
- Welch power spectral density (PSD) analysis
- Per-sensor and per-axis statistics
- MATLAB `.mat` file export

**Usage:**
```bash
cd signal_reception
python plot_sensor_raw_data.py
```

**Visualization Outputs:**
1. Z-Axis Time-Domain Waveforms (12 subplots)
2. Z-Axis Welch PSD (12 subplots)
3. Sensor 1 Three-Axis Time-Domain
4. Sensor 1 Three-Axis Welch PSD

**Documentation:** See `signal_reception/README.md` for detailed information.

---

## System Configuration

### Hardware Requirements

- **Sensors:** 12 MPU9250 magnetometer sensors
- **Communication:** UART at 1024000 baud rate
- **Embedded System:** FreeRTOS-based with 1000 Hz tick frequency
- **USB:** For serial communication to PC

### Software Requirements

- Python 3.6+
- Dependencies (see `requirements.txt`):
  - `numpy`: Numerical computations
  - `scipy`: Signal processing and scientific computing
  - `matplotlib`: Data visualization
  - `pyserial`: Serial communication

### Installation of Dependencies

```bash
pip install numpy scipy matplotlib pyserial
```

---

## UART Frame Format

All modules use the same UART frame format:

```
Byte Offset | Field              | Size | Type
0-3         | Header             | 4    | 0xAA 0x55 0xAA 0x55
4-5         | Payload Length     | 2    | uint16_t (little-endian)
6-9         | Timestamp (tick)   | 4    | uint32_t (little-endian)
10-153      | Sensor Payload     | 144  | 12×3 float32 (little-endian)
154-155     | CRC16              | 2    | uint16_t (little-endian)
```

**Total Frame Size:** `4 + 2 + 4 + payload_len + 2` bytes (variable)

**Important:** Firmware may append diagnostic fields after the first 144 bytes of magnetometer data.
All Python tools decode the first 144 bytes as 12×3 float32 mag values and ignore any remaining payload bytes.

**CRC Algorithm:** CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF)

---

## Data Output Formats

### MATLAB Format (`.mat`)

Generated by all acquisition modules:

```matlab
rawData         [N, 12, 3] float32    Raw sensor measurements
filteredData    [N, 12, 3] float32    LMS filtered data
timestamps      [N] float64           Time in seconds
tickTimestamps  [N] uint32            Raw FreeRTOS tick values
info            struct                Acquisition metadata
```

**Load in MATLAB:**
```matlab
data = load('lms_uart_data_20240308_120000.mat');
raw = data.rawData;
filtered = data.filteredData;
```

**Load in Python:**
```python
from scipy.io import loadmat
data = loadmat('lms_uart_data_20240308_120000.mat')
raw = data['rawData']
filtered = data['filteredData']
```

### C Header Format (`.h`)

Generated by calibration module:

```c
const float mag_hard_iron[12][3] = { ... };
const float mag_soft_iron[12][3][3] = { ... };
```

Ready for embedding in firmware.

### PNG Image Format

Generated by all visualization modules:

```
01_Z_Axis_TimeDomain_YYYYMMDD_HHMMSS.png
02_Z_Axis_Welch_PSD_YYYYMMDD_HHMMSS.png
03_Sensor1_ThreeAxis_TimeDomain_YYYYMMDD_HHMMSS.png
04_Sensor1_ThreeAxis_Welch_PSD_YYYYMMDD_HHMMSS.png
```

All plots are saved to the project root directory with timestamps.

---

## Workflow Examples

### Example 1: Complete Data Acquisition and Filtering

```python
from lms_uart_filter import acquire_and_filter_uart_data

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
raw_data = result['raw_data']           # [1000, 12, 3]
filtered_data = result['filtered_data'] # [1000, 12, 3]
timestamps = result['timestamps']       # [1000]
sampling_rate = result['sampling_rate'] # ~38 Hz
```

### Example 2: Frequency-Aware Filtering (Dynamic Scenarios)

```bash
python lms/lms_realtime_adaptive.py
```

This uses Butterworth low-pass filtering to:
- Preserve low-frequency magnetic field signals (0-15 Hz)
- Remove high-frequency sensor noise (>15 Hz)
- Provide zero-phase filtering with batch processing

### Example 3: Magnetometer Calibration

```bash
python calibrate/mag_cali.py
```

Steps:
1. Connect 12 sensors to embedded system
2. Run script and perform figure-eight motion for 60 seconds
3. Script computes calibration parameters
4. Generates C header file for firmware
5. Displays 3D visualization of calibration quality

### Example 4: UART Diagnostics

```bash
python sequences_rate_check/check_sequence_rate.py
```

Verifies:
- Frame rate stability
- Sequence increment consistency
- Frame loss detection
- Timing jitter analysis

---

## Troubleshooting

### Serial Connection Issues

**Problem:** "Serial connection failed"

**Solutions:**
1. Verify serial port name (check Device Manager)
2. Ensure no other application is using the port
3. Check USB cable connection
4. Try a different USB port

### No Data Received

**Problem:** "Total valid samples: 0"

**Solutions:**
1. Verify UART is transmitting data
2. Check serial port configuration (baud rate, parity)
3. Verify frame header (0xAA 0x55 0xAA 0x55)
4. Check CRC calculation in firmware

### Poor Filtering Results

**Problem:** Filtered output doesn't match expectations

**Solutions:**
1. Check reference signal strategy (use `'moving_average'` for v1.1+)
2. Adjust filter order (try 8-16)
3. Adjust step size (try 0.001-0.01)
4. For dynamic scenarios, use `lms_realtime_adaptive.py`

### High Noise Levels

**Problem:** "Std: 3.5" (expected ~0.5-1.0)

**Solutions:**
1. Move away from EMI sources
2. Check sensor connections
3. Verify power supply stability
4. Try frequency-aware filtering

---

## Performance Metrics

### Typical Performance

- **Sampling Rate:** ~38 Hz (26.3 ms per frame)
- **Processing Time:** 2-5 seconds for 1000 samples
- **Visualization Time:** 3-10 seconds for all plots
- **File Size:** ~500 KB per 1000 samples (MATLAB format)

### Memory Requirements

- **RAM:** ~20 MB for 1000 samples × 12 sensors × 3 axes
- **Disk:** ~500 KB per acquisition session

### Computational Complexity

- **CRC Validation:** O(N)
- **LMS Filtering:** O(N × M) where M = filter order
- **Welch PSD:** O(N log N) per sensor/axis
- **Visualization:** O(N) for plotting

---

## Version History

### v1.1 (2024-03-08)

**Major Updates:**
- Fixed LMS reference signal strategy (delayed → moving_average)
- Ensures real magnetic field signals are preserved
- Added frequency-aware adaptive filtering option
- Supports both static and dynamic magnetic field applications

**New Features:**
- `adaptive_noise_filter.py`: Butterworth low-pass filtering
- `lms_realtime_adaptive.py`: Real-time frequency-aware filtering
- Automatic frequency analysis and cutoff recommendation

### v1.0 (2024-03-01)

**Initial Release:**
- Basic LMS filter implementation
- 12 independent sensor processing
- Real-time UART data acquisition
- Magnetometer calibration system
- Signal reception and visualization

---

## References

### Adaptive Filtering Theory
1. Widrow, B., & Hoff, M. E. (1960). Adaptive switching circuits.
2. Haykin, S. (2002). Adaptive Filter Theory (4th ed.).
3. Sayed, A. H. (2003). Fundamentals of Adaptive Filtering.

### Magnetometer Calibration
1. Renaudin, V., et al. (2010). Complete Triaxial Magnetometer Calibration in the Magnetic Domain.
2. Crassidis, J. L., & Junkins, J. L. (2012). Optimal Estimation of Attitude and Heading.

### Signal Processing
1. Welch, P. D. (1967). The use of fast Fourier transform for estimation of power spectra.
2. Harris, F. J. (1978). On the use of windows for harmonic analysis with the discrete Fourier transform.

### UART Communication
1. RS-232 Standard: EIA/TIA-232-F
2. CRC16: https://en.wikipedia.org/wiki/Cyclic_redundancy_check

### FreeRTOS
1. Official Documentation: https://www.freertos.org/
2. Tick Rate Configuration: `configTICK_RATE_HZ`

---

## Support and Documentation

### Module-Specific Documentation

- **LMS Module:** See `lms/README.md` for detailed algorithm documentation
- **Calibration:** See `calibrate/README.md` for calibration procedures
- **Sequence Check:** See `sequences_rate_check/README.md` for diagnostics
- **Signal Reception:** See `signal_reception/README.md` for visualization details

### Common Tasks

**Acquire and filter data:**
```bash
python main.py
```

**Verify UART communication:**
```bash
cd sequences_rate_check
python check_sequence_rate.py
```

**Calibrate sensors:**
```bash
cd calibrate
python mag_cali.py
```

**Analyze raw signals:**
```bash
cd signal_reception
python plot_sensor_raw_data.py
```

### Getting Help

1. Check the module-specific README files
2. Review the Troubleshooting section above
3. Verify hardware connections and UART configuration
4. Check console output for diagnostic messages
5. Review generated plots for data quality assessment

---

## License

This project is part of the Magnetic Positioning Platform system.

---

## Project Information

**Platform:** Magnetic Positioning Platform  
**Component:** Python Data Processing Module  
**Version:** 1.1  
**Last Updated:** 2024-03-08  
**Status:** Active Development

---

## Quick Reference

### File Locations

| Component | Main File | Documentation |
|-----------|-----------|----------------|
| LMS Filtering | `lms/lms_realtime.py` | `lms/README.md` |
| Adaptive Filtering | `lms/lms_realtime_adaptive.py` | `lms/README.md` |
| Calibration | `calibrate/mag_cali.py` | `calibrate/README.md` |
| Sequence Check | `sequences_rate_check/check_sequence_rate.py` | `sequences_rate_check/README.md` |
| Signal Reception | `signal_reception/plot_sensor_raw_data.py` | `signal_reception/README.md` |

### Default Parameters

| Parameter | Default | Range |
|-----------|---------|-------|
| Serial Port | COM7 | COM1-COM255 |
| Baud Rate | 1024000 | 9600-1024000 |
| Max Samples | 1000 | 10-10000 |
| Filter Order | 8 | 4-32 |
| Step Size | 0.001 | 0.0001-0.1 |
| Reference Window | 10 | 5-50 |
| Sampling Rate | ~38 Hz | ~35-40 Hz |

### Output Files

| File Type | Location | Format |
|-----------|----------|--------|
| Plots | Root directory | PNG (150 DPI) |
| Data | Root directory | MATLAB `.mat` |
| Calibration | `calibrate/` | C header `.h` |
| Calibration Data | `calibrate/` | NumPy `.npz` |

---

## Acknowledgments

This module integrates multiple signal processing techniques including:
- Least Mean Squares (LMS) adaptive filtering
- Butterworth low-pass filtering
- Welch power spectral density estimation
- Ellipsoid fitting for magnetometer calibration
- FreeRTOS tick-based timing

All components are designed for real-time embedded systems with independent sensor processing and no cross-dependencies.
