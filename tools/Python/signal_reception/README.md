# Signal Reception and Visualization Tool

## Overview

This tool acquires raw MPU9250 magnetometer data from UART and provides comprehensive visualization including time-domain waveforms and Welch power spectral density (PSD) analysis. It supports real-time data acquisition from 12 independent sensors with automatic CRC validation and data quality diagnostics.

**Key Features:**
- Real-time UART data acquisition from 12 sensors
- Automatic CRC16 validation
- Time-domain waveform visualization
- Welch PSD analysis for frequency domain inspection
- Per-sensor and per-axis analysis
- Data export to MATLAB `.mat` format
- Comprehensive acquisition statistics

---

## File Structure

```
signal_reception/
├── README.md                    # This file
└── plot_sensor_raw_data.py     # Main acquisition and visualization script
```

---

## System Requirements

### Hardware
- 12 MPU9250 magnetometer sensors connected via UART
- Serial communication at 1024000 baud rate
- FreeRTOS-based embedded system with tick frequency of 1000 Hz

### Software
- Python 3.6+
- Dependencies:
  - `numpy`: Numerical computations
  - `scipy`: Signal processing (Welch PSD)
  - `matplotlib`: Data visualization
  - `pyserial`: Serial communication

### Installation

```bash
pip install numpy scipy matplotlib pyserial
```

---

## Usage

### Basic Workflow

1. **Configure the script:**
   - Edit `main()` function to set serial port and acquisition parameters
   - Default: `COM7`, 1000 samples

2. **Run the acquisition:**
   ```bash
   python plot_sensor_raw_data.py
   ```

3. **Monitor progress:**
   - Console shows real-time acquisition status
   - Press Ctrl+C to stop early if needed

4. **Review visualizations:**
   - Four figure windows open automatically
   - Examine time-domain and frequency-domain data

5. **Save data (optional):**
   - When prompted, enter "yes" to save data to `.mat` file
   - Data can be loaded in MATLAB or Python for further analysis

### Configuration Parameters

Edit the `main()` function to customize:

```python
SERIAL_PORT = "COM7"              # Serial port name
BAUD_RATE = 1024000               # UART baud rate
MAX_SAMPLES = 1000                # Number of frames to acquire
TICK_HZ = 1000                    # FreeRTOS tick frequency
```

---

## Output Interpretation

### Console Output Example

```
Opening serial port COM7...
Serial port opened successfully

Starting data acquisition...
Press Ctrl+C to stop acquisition

Collected 100 samples | tick: 2465866 | t: 2465.866 s
Collected 200 samples | tick: 2467166 | t: 2467.166 s
...
Collected 1000 samples | tick: 2489266 | t: 2489.266 s
Serial port closed

=== Acquisition Summary ===
Total valid samples: 1000
CRC failures: 0
Payload length mismatches: 0
Header resync count: 0

Average sampling rate: 38.00 Hz
Total duration: 26.32 s
```

### Key Metrics

#### Acquisition Statistics

- **Total valid samples:** Number of successfully acquired frames
- **CRC failures:** Frames with invalid CRC (data corruption)
- **Payload length mismatches:** Frames with incorrect payload size
- **Header resync count:** Number of times header was re-synchronized

**Interpretation:**
- CRC failures = 0: Perfect data integrity
- CRC failures > 0: Potential UART transmission issues
- Header resync > 0: Normal (indicates frame boundaries)

#### Sampling Rate

- **Average sampling rate:** Measured frame rate in Hz
- **Total duration:** Time span of acquisition
- **Expected:** ~38 Hz for typical configuration

**Interpretation:**
- 38 ± 1 Hz: Normal operation
- < 35 Hz: Potential timing issues
- > 40 Hz: Possible clock drift

---

## Visualization Outputs

The script generates four figure windows:

### Figure 1: Z-Axis Time-Domain Waveforms (All 12 Sensors)

**Layout:** 4×3 grid (12 subplots)

**Content per subplot:**
- **X-axis:** Time (seconds)
- **Y-axis:** Magnetic field magnitude
- **Blue line:** Z-axis raw data
- **Statistics box:** Mean and standard deviation

**Interpretation:**
- Flat line: Stable magnetic field
- Oscillations: Dynamic magnetic field or noise
- Mean value: DC offset (hard iron error)
- Std value: Noise level

**Example readings:**
```
Sensor 1 (Z Axis)
Mean: 40.23
Std: 0.85
```

### Figure 2: Z-Axis Welch PSD (All 12 Sensors)

**Layout:** 4×3 grid (12 subplots)

**Content per subplot:**
- **X-axis:** Frequency (Hz)
- **Y-axis:** Power spectral density (dB/Hz)
- **Blue line:** Welch PSD estimate

**Interpretation:**
- Peaks: Dominant frequency components
- Flat baseline: White noise
- DC component (0 Hz): Hard iron offset
- High-frequency content: Sensor noise

**Example analysis:**
```
Sensor 1 (Z Axis PSD)
- Peak at 0 Hz: ~40 dB (DC component)
- Flat baseline: ~-20 dB (noise floor)
- No significant peaks above 1 Hz: Good signal quality
```

### Figure 3: Sensor 1 Three-Axis Time-Domain Data

**Layout:** 3×1 grid (3 subplots)

**Content per subplot:**
- **X-axis:** Time (seconds)
- **Y-axis:** Magnetic field magnitude
- **Red/Green/Blue line:** X/Y/Z axis data
- **Legend:** Mean and standard deviation

**Interpretation:**
- Compare relative magnitudes of X, Y, Z axes
- Identify which axis has most noise
- Detect correlated motion between axes

**Example:**
```
Sensor 1 - X Axis
Mean: 15.32, Std: 0.45

Sensor 1 - Y Axis
Mean: 22.18, Std: 0.52

Sensor 1 - Z Axis
Mean: 40.23, Std: 0.85
```

### Figure 4: Sensor 1 Three-Axis Welch PSD

**Layout:** 3×1 grid (3 subplots)

**Content per subplot:**
- **X-axis:** Frequency (Hz)
- **Y-axis:** Power spectral density (dB/Hz)
- **Red/Green/Blue line:** X/Y/Z axis PSD

**Interpretation:**
- Compare noise characteristics across axes
- Identify axis-specific noise sources
- Detect frequency-dependent effects

---

## Data Quality Assessment

### Noise Analysis

**Standard deviation (Std):**
- Typical value: 0.5-2.0 units
- < 0.5: Very low noise (excellent)
- 0.5-1.0: Low noise (good)
- 1.0-2.0: Moderate noise (acceptable)
- > 2.0: High noise (investigate)

### Signal-to-Noise Ratio (SNR)

For a stable magnetic field:
```
SNR = mean / std
```

**Interpretation:**
- SNR > 40: Excellent signal quality
- SNR 20-40: Good signal quality
- SNR 10-20: Acceptable signal quality
- SNR < 10: Poor signal quality

**Example:**
```
Sensor 1 Z-axis:
Mean: 40.23, Std: 0.85
SNR = 40.23 / 0.85 ≈ 47 dB (excellent)
```

### Frequency Content

**DC component (0 Hz):**
- Represents hard iron offset
- Should be consistent across samples
- Typical value: 10-50 units depending on sensor location

**Noise floor (high frequency):**
- Represents sensor noise
- Typically flat above 5 Hz
- Typical level: -20 to -30 dB/Hz

**Signal content (0-5 Hz):**
- Represents actual magnetic field variations
- Should be minimal for static field
- Indicates dynamic motion if present

---

## Data Export

### MATLAB Format (`.mat`)

When prompted, enter "yes" to save data:

```
Do you want to save the acquired data? (Yes/No) [No]: yes
Data saved to: sensor_data_20240308_120000.mat
```

**File contents:**
```
sensorData      [N, 12, 3] float32    Raw sensor measurements
timestamps      [N] float64            Time in seconds
tickTimestamps  [N] uint32             Raw FreeRTOS tick values
info            struct                 Acquisition metadata
```

**Load in MATLAB:**
```matlab
data = load('sensor_data_20240308_120000.mat');
sensor_data = data.sensorData;      % [N, 12, 3]
timestamps = data.timestamps;       % [N]
```

**Load in Python:**
```python
from scipy.io import loadmat

data = loadmat('sensor_data_20240308_120000.mat')
sensor_data = data['sensorData']    # [N, 12, 3]
timestamps = data['timestamps']     # [N]
```

### Data Structure

**sensorData dimensions:**
- First dimension (N): Number of samples
- Second dimension (12): Sensor index (0-11)
- Third dimension (3): Axis index (0=X, 1=Y, 2=Z)

**Access examples:**
```python
# Get all data from sensor 0, Z-axis
z_data = sensor_data[:, 0, 2]

# Get X, Y, Z for sensor 5 at sample 100
sample = sensor_data[100, 5, :]  # [x, y, z]

# Get all sensors at sample 50
frame = sensor_data[50, :, :]    # [12, 3]
```

---

## Algorithm Details

### CRC16 Validation

The script validates each frame using CRC16-CCITT-FALSE:

```
Polynomial: 0x1021
Initial value: 0xFFFF
Reflection: None
Final XOR: 0x0000
```

**Process:**
1. Extract CRC from last 2 bytes of frame
2. Calculate CRC over all bytes except CRC field
3. Compare calculated vs. received CRC
4. Accept frame only if CRCs match

### Welch PSD Estimation

Power spectral density is estimated using Welch's method:

**Parameters:**
- Window: Hamming window
- Segment length: min(256, N/2)
- Overlap: 50%
- FFT length: Next power of 2 ≥ segment length

**Advantages:**
- Reduced variance compared to raw FFT
- Better frequency resolution
- Robust to short data segments

### Frame Parsing

UART frames are parsed according to the following structure:

```
Byte Offset | Field              | Size | Type
0-3         | Header             | 4    | 0xAA 0x55 0xAA 0x55
4-5         | Payload Length     | 2    | uint16_t (little-endian)
6-9         | Timestamp (tick)   | 4    | uint32_t (little-endian)
10-153      | Sensor Payload     | 144  | 12×3 float32 (little-endian)
154-155     | CRC16              | 2    | uint16_t (little-endian)
```

**Total frame size:** 156 bytes

---

## Common Issues and Solutions

### Issue 1: CRC Failures

**Symptom:** "CRC failures: 5"

**Causes:**
- UART transmission errors
- Baud rate mismatch
- Cable quality issues
- Electromagnetic interference

**Solutions:**
1. Check UART baud rate matches firmware (1024000)
2. Verify cable connections and quality
3. Move USB cable away from EMI sources
4. Try a different USB port
5. Reduce baud rate if errors persist

### Issue 2: Low Sampling Rate

**Symptom:** "Average sampling rate: 25.3 Hz" (expected ~38 Hz)

**Causes:**
- System load too high
- UART buffer overflow
- Baud rate too low
- USB timing issues

**Solutions:**
1. Reduce other system tasks
2. Increase UART buffer size in firmware
3. Verify baud rate is 1024000
4. Try a different USB port
5. Check system CPU load

### Issue 3: High Noise Levels

**Symptom:** "Std: 3.5" (expected ~0.5-1.0)

**Causes:**
- Electromagnetic interference
- Sensor malfunction
- Loose connections
- Inadequate power supply

**Solutions:**
1. Move away from EMI sources (motors, power supplies, etc.)
2. Check sensor connections
3. Verify power supply voltage and stability
4. Try a different sensor location
5. Check for loose cables

### Issue 4: Inconsistent Data

**Symptom:** Different results on repeated runs

**Causes:**
- Environmental magnetic field variations
- Electromagnetic interference
- Temperature changes
- Sensor drift

**Solutions:**
1. Ensure stable environment during acquisition
2. Allow sensors to warm up before acquisition
3. Move away from EMI sources
4. Repeat acquisition multiple times
5. Check for loose connections

---

## Advanced Usage

### Batch Acquisition

To acquire data from multiple sessions:

```python
import subprocess
import time

for session in range(5):
    print(f"Session {session + 1}/5")
    subprocess.run(['python', 'plot_sensor_raw_data.py'])
    time.sleep(60)  # Wait 1 minute between sessions
```

### Custom Analysis

Load and analyze saved data:

```python
from scipy.io import loadmat
import numpy as np

# Load data
data = loadmat('sensor_data_20240308_120000.mat')
sensor_data = data['sensorData']
timestamps = data['timestamps']

# Compute statistics per sensor
for sensor_idx in range(12):
    for axis_idx in range(3):
        signal = sensor_data[:, sensor_idx, axis_idx]
        mean = np.mean(signal)
        std = np.std(signal)
        print(f"Sensor {sensor_idx}, Axis {axis_idx}: "
              f"Mean={mean:.2f}, Std={std:.2f}")
```

### Real-Time Monitoring

Modify the script to display real-time statistics:

```python
# In the acquisition loop, add:
if sample_count % 50 == 0:
    current_data = sensor_data[:sample_count, :, :]
    mean_z = np.mean(current_data[:, 0, 2])
    std_z = np.std(current_data[:, 0, 2])
    print(f"Sensor 1 Z: Mean={mean_z:.2f}, Std={std_z:.2f}")
```

---

## Performance Metrics

### Typical Performance

- **Acquisition rate:** ~38 Hz (26.3 ms per frame)
- **Processing time:** ~2-5 seconds for 1000 samples
- **Visualization time:** ~3-10 seconds for all plots
- **File size:** ~500 KB for 1000 samples in `.mat` format

### Memory Requirements

- **RAM:** ~20 MB for 1000 samples × 12 sensors × 3 axes
- **Disk:** ~500 KB per acquisition session

### Computational Complexity

- **CRC validation:** O(N) where N = number of frames
- **Welch PSD:** O(N log N) per sensor/axis
- **Visualization:** O(N) for plotting

---

## Welch PSD Parameters

### Window Configuration

The script automatically selects window parameters based on data length:

```python
nfft = min(256, max(64, int(2 ** np.ceil(np.log2(z_data.size)))))
window_len = min(256, z_data.size)
overlap_len = window_len // 2
```

**Rationale:**
- Minimum FFT: 64 points (good frequency resolution)
- Maximum FFT: 256 points (computational efficiency)
- 50% overlap: Standard Welch method

### Frequency Resolution

Frequency resolution depends on FFT length and sampling rate:

```
Δf = fs / nfft
```

**Example for 38 Hz sampling rate:**
- nfft = 256: Δf = 38/256 ≈ 0.15 Hz
- nfft = 128: Δf = 38/128 ≈ 0.30 Hz

---

## UART Frame Format Details

### Header (Bytes 0-3)

```
0xAA 0x55 0xAA 0x55
```

Fixed synchronization pattern for frame alignment.

### Payload Length (Bytes 4-5)

```
Little-endian uint16_t
Expected value: 144 (12 sensors × 3 axes × 4 bytes)
```

### Timestamp (Bytes 6-9)

```
Little-endian uint32_t
FreeRTOS tick count
Conversion to seconds: tick / TICK_HZ
```

### Sensor Payload (Bytes 10-153)

```
12 sensors × 3 axes × 4 bytes (float32)
Order: [S0X, S0Y, S0Z, S1X, S1Y, S1Z, ..., S11X, S11Y, S11Z]
Little-endian IEEE 754 single precision
```

### CRC16 (Bytes 154-155)

```
Little-endian uint16_t
Calculated over bytes 0-153
Algorithm: CRC16-CCITT-FALSE
```

---

## Troubleshooting

### Serial Connection Issues

**Problem:** "Unable to open serial port"

**Solutions:**
1. Verify serial port name (check Device Manager)
2. Ensure no other application is using the port
3. Check USB cable connection
4. Try a different USB port
5. Restart the embedded system

### No Data Received

**Problem:** "Total valid samples: 0"

**Solutions:**
1. Verify UART is transmitting data
2. Check serial port configuration
3. Verify frame header (0xAA 0x55 0xAA 0x55)
4. Check CRC calculation in firmware
5. Verify baud rate matches firmware

### Visualization Issues

**Problem:** "Not enough data" in PSD plots

**Solutions:**
1. Increase MAX_SAMPLES to at least 100
2. Verify data is being acquired
3. Check for CRC failures
4. Ensure sampling rate is > 0

---

## References

### Signal Processing
- Welch, P. D. (1967). "The use of fast Fourier transform for estimation of power spectra"
- Harris, F. J. (1978). "On the use of windows for harmonic analysis with the discrete Fourier transform"

### UART Communication
- RS-232 Standard: EIA/TIA-232-F
- CRC16: https://en.wikipedia.org/wiki/Cyclic_redundancy_check

### FreeRTOS
- Official documentation: https://www.freertos.org/
- Tick rate configuration: `configTICK_RATE_HZ`

---

## Version History

- **v1.0** (2024-03-08): Initial release
  - Real-time UART acquisition
  - Time-domain and frequency-domain visualization
  - Automatic CRC validation
  - MATLAB data export
  - Comprehensive diagnostics
