# LMS Adaptive Filtering for 12-Sensor Magnetometer System

## Overview

This module provides independent LMS (Least Mean Squares) adaptive filtering for 12 MPU9250 magnetometer sensors. Each sensor's three axes (X, Y, Z) has an independent filter instance, with no cross-dependencies between sensors.

## File Structure

```
lms/
├── lms_filter.py              # LMS filter core implementation
├── sensor_lms_processor.py    # 12-sensor independent processor
├── lms_realtime.py            # Real-time UART data acquisition and filtering
├── adaptive_noise_filter.py   # Frequency-aware adaptive filtering (v1.1+)
├── lms_realtime_adaptive.py   # Real-time adaptive filtering main program (v1.1+)
├── FILTER_COMPARISON.md       # Comparison of filtering strategies
└── README.md                  # Documentation (Chinese)
```

## Core Components

### 1. `lms_filter.py` - LMS Filter Core

**LMSFilter Class**
- Standard LMS adaptive filtering algorithm implementation
- Configurable filter order and step size
- Support for Leaky LMS variant
- Provides error history, output history, and weight norm history

**MultiChannelLMSFilter Class**
- Multi-channel independent LMS filtering
- Each channel has independent filter instance
- Suitable for multi-axis sensor data

**Key Parameters:**
- `filter_order`: Filter order (recommended: 8-16)
- `step_size`: Step size μ (learning rate, recommended: 0.001-0.01)
- `leak_factor`: Leak factor (default: 1.0, standard LMS)

### 2. `sensor_lms_processor.py` - Sensor Processor

**ReferenceSignalGenerator Class**
- Generates reference signals for LMS filtering
- Supports multiple strategies:
  - `'delayed'`: Delayed input signal (deprecated, smooths real signals)
  - `'moving_average'`: Moving average (recommended for signal preservation) ⭐
  - `'zero'`: Zero reference (pure noise cancellation mode)

**SensorLMSProcessor Class**
- LMS processor for single sensor (3 axes)
- Independent filtering per axis
- Records raw and filtered data history

**MultiSensorLMSProcessor Class**
- Independent processor for 12 sensors
- Each sensor completely independent, no cross-dependencies
- Provides batch and single-sample processing interfaces

### 3. `lms_realtime.py` - Real-time Filtering Main Program

**RealtimeLMSFilter Class**
- Real-time reading of 12 sensor data from UART
- Applies LMS adaptive filtering
- Real-time display of filtering statistics
- Supports data saving and visualization

**Main Features:**
- Real-time data acquisition and filtering
- Pre/post-filtering comparison visualization
- Learning curve visualization (error evolution)
- MSE statistical analysis
- Data saving in `.mat` format

### 4. `adaptive_noise_filter.py` - Frequency-Aware Filtering (v1.1+)

**AdaptiveNoiseFilter Class**
- Butterworth low-pass filter for signal/noise separation
- Preserves low-frequency magnetic field signals (0-15 Hz)
- Removes high-frequency sensor noise (>15 Hz)
- Zero-phase filtering for batch processing

**MultiAxisAdaptiveFilter Class**
- Independent filtering for 3D magnetometer data
- Per-axis configuration

**MultiSensorAdaptiveFilter Class**
- Frequency-aware filtering for 12 independent sensors

---

## Usage

### Basic Usage

```python
from lms_realtime import RealtimeLMSFilter

# Create real-time filter
realtime_filter = RealtimeLMSFilter(
    serial_port="COM7",
    baud_rate=1024000,
    filter_order=8,              # Filter order
    step_size=0.001,             # Step size
    reference_strategy='moving_average',
    reference_window=10,
    tick_hz=1000
)

# Acquire and filter data
realtime_filter.acquire_and_filter(max_samples=1000)

# Visualize results
realtime_filter.visualize_results(sensor_indices=[0, 1, 2])
realtime_filter.visualize_learning_curves(sensor_indices=[0, 1, 2])

# Save data
realtime_filter.save_data()
```

### Run Main Program Directly

```bash
cd lms
python lms_realtime.py
```

### Using Frequency-Aware Filtering (Recommended for Dynamic Scenarios)

```bash
python lms_realtime_adaptive.py
```

### Custom Configuration

Edit parameters in `main()` function of `lms_realtime.py`:

```python
SERIAL_PORT = "COM7"                    # Serial port
BAUD_RATE = 1024000                     # Baud rate
MAX_SAMPLES = 1000                      # Maximum samples
FILTER_ORDER = 8                        # Filter order
STEP_SIZE = 0.001                       # Step size
REFERENCE_STRATEGY = 'moving_average'   # Reference signal strategy
REFERENCE_WINDOW = 10                   # Moving average window
```

---

## LMS Algorithm Theory

### Standard LMS Algorithm

```
y[n] = w[n]^T * x[n]              (filter output)
e[n] = d[n] - y[n]                (error signal)
w[n+1] = w[n] + μ * e[n] * x[n]   (weight update)
```

**Symbol Definitions:**
- `x[n]`: Input signal vector (filter taps)
- `d[n]`: Desired signal (reference signal)
- `y[n]`: Filter output
- `e[n]`: Error signal
- `w[n]`: Filter weights
- `μ`: Step size (learning rate)

### Parameter Tuning Guide

**Filter Order (filter_order)**
- Small values (4-8): Fast convergence, limited filtering
- Medium values (8-16): Balanced performance and computation (recommended)
- Large values (16-32): Better filtering, slow convergence

**Step Size (step_size)**
- Small values (0.001-0.003): Stable convergence, slow speed
- Medium values (0.003-0.008): Balanced convergence and stability (recommended)
- Large values (0.008-0.015): Fast convergence, potential instability

**Reference Signal Strategy**
- `'delayed'`: For periodic or correlated noise (deprecated, smooths real signals)
- `'moving_average'`: For preserving real magnetic field signals, removing high-frequency noise (recommended) ⭐
- `'zero'`: For pure noise cancellation (assumes desired signal is zero)

**Important Update (v1.1):**
- Reference signal strategy changed from `'delayed'` to `'moving_average'`
- Filter output logic changed to: `filtered = desired_signals` (moving average)
- This ensures **real magnetic field signals are preserved**, not filtered as noise

---

## Algorithm Update (v1.1)

### Core Problem Fixed

**Problem:** The original moving average LMS filter used delayed signals as reference, causing the filter to learn the input signal itself rather than remove noise. In real magnetic field applications, this would **filter out real magnetic field changes as noise**.

**Solution:**
1. Reference signal strategy changed to `'moving_average'`
2. Filter output changed to directly use moving average (desired signal)
3. LMS filter used for further optimization, not signal pattern learning

### New Workflow

```
Raw signal (40±noise)
    ↓
Moving average (window=10) → Desired signal (~40)
    ↓
LMS adaptive learning → Further optimization
    ↓
Filtered output: Stable around 40 ✓
```

### Performance Comparison

| Feature | Old Version (Delayed) | New Version (Moving Average) |
|---------|----------------------|------------------------------|
| Preserve real signals | ❌ Smoothed out | ✅ Fully preserved |
| Remove noise | ✅ Effective | ✅ Effective |
| Dynamic response | ❌ Delay + attenuation | ✅ Fast and accurate |
| Use cases | Static only | **Static + Dynamic** |

### Recommended Configuration

```python
# Recommended configuration (v1.1)
REFERENCE_STRATEGY = 'moving_average'  # Changed to moving average
REFERENCE_WINDOW = 10                  # Moving average window size
STEP_SIZE = 0.001                      # Conservative step size
```

---

## Sensor Independence Design

The core design principle of this system is **sensor independence**:

1. **Each sensor filters independently**: 12 sensors each have independent LMS filter instances
2. **No cross-dependencies**: Sensors don't share weights or reference signals
3. **Parallel processing**: Each sensor can be processed independently, enabling parallelization
4. **Independent parameters**: Each sensor can be configured with different filtering parameters if needed

---

## Performance Metrics

The system automatically calculates and displays the following performance metrics:

- **MSE (Mean Squared Error)**: Mean squared error for each sensor and axis
- **Noise Reduction**: Difference in standard deviation before and after filtering
- **Convergence Speed**: Visualized through learning curves
- **Weight Norm**: Evolution of filter weights

---

## Data Format

### Input Data Format (UART Frame)

```
Frame header: 0xAA 0x55 0xAA 0x55 (4 bytes)
Payload length: uint16 (2 bytes, little-endian)
Timestamp: uint32 (4 bytes, little-endian, FreeRTOS tick)
Sensor data: 12 sensors × 3 axes × 4 bytes = 144 bytes (float32)
CRC16: uint16 (2 bytes, little-endian)
Total length: 156 bytes/frame
```

### Output Data Format (.mat file)

```matlab
rawData:         [N, 12, 3] Raw data
filteredData:    [N, 12, 3] Filtered data
timestamps:      [N] Timestamps (seconds)
tickTimestamps:  [N] Raw tick timestamps
sampleCount:     Scalar, number of samples
filterOrder:     Scalar, filter order
stepSize:        Scalar, step size
mseMatrix:       [12, 3] MSE matrix
```

---

## Visualization Features

### 1. Filtering Results Comparison
- Display raw signal (blue) vs filtered signal (red)
- Show 3 axes for each sensor separately
- Display noise reduction amount

### 2. Learning Curves
- Show squared error evolution over time
- Include moving average curve
- Log-scale display of convergence process

---

## Important Notes

1. **Step size selection**: Too large step size may cause divergence, too small causes slow convergence
2. **Reference signal**: Choice of reference signal greatly affects filtering performance
3. **Initialization**: Filter needs certain number of samples to converge
4. **Real-time capability**: Filter computation is small, suitable for real-time applications
5. **Memory usage**: Long-running operation should monitor memory usage of historical data

---

## Extension Features

To extend functionality, consider:

1. **Normalized LMS (NLMS)**: Adaptive step size adjustment
2. **Variable Step-Size LMS**: Dynamic step size adjustment based on error
3. **Frequency-Domain LMS**: Frequency domain implementation for higher efficiency
4. **RLS Filter**: Recursive Least Squares, faster convergence
5. **Kalman Filter**: More complex state estimation

---

## Troubleshooting

**Problem: Filter doesn't converge**
- Check if step size is too large
- Check if reference signal is appropriate
- Increase filter order

**Problem: Filtering effect not obvious**
- Increase filter order
- Adjust reference signal strategy
- Check input data quality

**Problem: Serial connection failed**
- Check if serial port number is correct
- Check baud rate setting
- Confirm device is connected

**Problem: Real signals being filtered out (v1.0)**
- Upgrade to v1.1 which uses moving average strategy
- Or use `lms_realtime_adaptive.py` for frequency-aware filtering

---

## Comparison: LMS vs Frequency-Aware Filtering

See `FILTER_COMPARISON.md` for detailed comparison of:
- Moving average LMS filter (`lms_realtime.py`)
- Frequency-aware adaptive filter (`lms_realtime_adaptive.py`)

**Recommendation:** For real magnetic positioning applications with dynamic magnetic field changes, use `lms_realtime_adaptive.py` which preserves real signals while removing noise.

---

## Version History

- **v1.1** (2024-03-08): Major update
  - Fixed reference signal strategy: changed from delayed to moving average
  - Fixed filter output logic: directly use desired signal instead of error difference
  - Ensures real magnetic field signals are preserved, not filtered out
  - Supports dynamic magnetic field applications (magnet movement scenarios)
  - Added frequency-aware adaptive filtering option
  
- **v1.0** (2024-03-01): Initial version
  - Basic LMS filter implementation
  - 12 independent sensor processing
  - Real-time UART data acquisition

---

## References

1. Widrow, B., & Hoff, M. E. (1960). Adaptive switching circuits.
2. Haykin, S. (2002). Adaptive Filter Theory (4th ed.).
3. Sayed, A. H. (2003). Fundamentals of Adaptive Filtering.
4. Renaudin, V., et al. (2010). Complete Triaxial Magnetometer Calibration in the Magnetic Domain.

---

## System Requirements

### Hardware
- 12 MPU9250 magnetometer sensors connected via UART
- Serial communication at 1024000 baud rate
- FreeRTOS-based embedded system with 1000 Hz tick frequency

### Software
- Python 3.6+
- Dependencies:
  - `numpy`: Numerical computations
  - `scipy`: Signal processing
  - `matplotlib`: Data visualization
  - `pyserial`: Serial communication

### Installation

```bash
pip install numpy scipy matplotlib pyserial
```

---

## Support

For issues or questions:
1. Check the Troubleshooting section above
2. Review diagnostic output for data quality issues
3. Verify hardware connections and UART communication
4. Consult the UART frame format documentation
5. See `FILTER_COMPARISON.md` for filtering strategy selection guidance
