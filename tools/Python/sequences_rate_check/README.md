# UART Sequence Rate Check Tool

## Overview

This diagnostic tool verifies the actual UART output frame rate and sequence increment behavior for the MPU9250 magnetometer data stream. It automatically detects the dominant sequence increment pattern and identifies frame losses or irregular transmission events.

**Key Features:**
- Real-time UART frame rate measurement
- Automatic dominant sequence increment detection
- Frame loss detection relative to dominant increment
- Inter-frame timing analysis
- Comprehensive visualization and statistics
- No assumptions about sequence increment value

---

## File Structure

```
sequences_rate_check/
├── README.md                  # This file
└── check_sequence_rate.py    # Main diagnostic script
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
  - `matplotlib`: Data visualization
  - `pyserial`: Serial communication

### Installation

```bash
pip install numpy matplotlib pyserial
```

---

## Usage

### Basic Workflow

1. **Configure the script:**
   - Edit `main()` function to set serial port and test duration
   - Default: `COM7`, 10 seconds test duration

2. **Run the diagnostic:**
   ```bash
   python check_sequence_rate.py
   ```

3. **Review the results:**
   - Console output shows frame rate and sequence statistics
   - Four visualization plots are generated
   - Check for frame losses or timing irregularities

### Configuration Parameters

Edit the `main()` function to customize:

```python
serial_port = "COM7"              # Serial port name
baud_rate = 1024000               # UART baud rate
test_duration = 10                # Test duration in seconds
expected_frame_rate = 38          # Expected frame rate in Hz
frame_rate_tol_hz = 1.0           # Allowed frame rate error in Hz
```

---

## Output Interpretation

### Console Output Example

```
=== UART Sequence Increment Rate Check ===
Target UART frame rate: 38.0 Hz (period: 26.316 ms)
Test duration: 10 seconds

Serial port COM7 connected successfully

Starting data acquisition...
Received 20 frames | Current seq: 520 | Time: 0.53 s
Received 40 frames | Current seq: 1040 | Time: 1.05 s
...

=== Data Analysis Results ===
Total received frames: 380
CRC failures: 0
Length mismatches: 0
Header resync count: 0

Actual UART frame rate: 38.000 Hz (expected: 38.0 Hz)
Frame rate error: 0.00%

Dominant sequence increment: 26
This means the sequence counter advances by about 26 counts per UART frame.

Sequence increment consistency analysis:
  Normal increments (=26): 378
  Drop events (integer multiples > 1): 0
  Estimated lost UART frames: 0
  Irregular increments: 1

=== Conclusion ===
UART frame rate check passed: 38.00 Hz ≈ 38.0 Hz
Dominant sequence increment is 26, not necessarily 1.
Sequence behavior is consistent with a dominant increment of 26.
No dropped UART frames were inferred relative to the dominant increment.

Test completed
```

### Key Metrics

#### Frame Rate Analysis

- **Actual UART frame rate:** Measured frame rate in Hz
- **Expected frame rate:** Configured expected rate
- **Frame rate error:** Percentage deviation from expected

**Interpretation:**
- Error < 1%: Excellent timing stability
- Error 1-5%: Acceptable, minor timing variations
- Error > 5%: Potential timing issues, investigate

#### Sequence Increment Analysis

- **Dominant sequence increment:** Most common increment between consecutive sequence numbers
- **Normal increments:** Frames with expected increment (no loss)
- **Drop events:** Frames with integer multiples of dominant increment (indicates lost frames)
- **Estimated lost frames:** Total number of inferred lost frames
- **Irregular increments:** Frames with non-integer multiple increments (anomalies)

**Interpretation:**
- Dominant increment is typically 26 for 38 Hz frame rate
  - Formula: `dominant_increment ≈ tick_rate / frame_rate = 1000 / 38 ≈ 26`
- Drop events > 0: Some UART frames were lost
- Irregular increments > 0: Potential transmission errors or timing anomalies

#### Inter-Frame Timing

- **Mean interval:** Average time between consecutive frames
- **Std interval:** Standard deviation of inter-frame intervals
- **Min/Max interval:** Minimum and maximum observed intervals

**Interpretation:**
- Expected period: `1000 / frame_rate` ms
- For 38 Hz: ~26.3 ms
- Std < 1 ms: Excellent timing stability
- Std > 5 ms: Timing jitter, potential issues

#### Sequence Growth Rate

- **Measured sequence growth rate:** Actual rate of sequence counter increment
- **Predicted sequence growth rate:** Expected rate based on frame rate and dominant increment
- **Difference:** Deviation between measured and predicted

**Interpretation:**
- Difference < 0.1 counts/s: Excellent synchronization
- Difference 0.1-1 counts/s: Minor timing drift
- Difference > 1 counts/s: Significant timing issues

---

## Visualization

The script generates four plots:

### Plot 1: Sequence Number vs Time

- **X-axis:** Time (seconds)
- **Y-axis:** Sequence number
- **Blue dots:** Measured sequence numbers
- **Red dashed line:** Linear fit (expected behavior)

**Interpretation:**
- Linear trend: Good timing stability
- Deviations from line: Timing irregularities

### Plot 2: Sequence Increment

- **X-axis:** Frame index
- **Y-axis:** Sequence increment between consecutive frames
- **Green dots:** Measured increments
- **Red dashed line:** Dominant increment value

**Interpretation:**
- Constant value: Normal operation
- Spikes above line: Dropped frames
- Irregular values: Transmission anomalies

### Plot 3: Inter-Frame Interval Histogram

- **X-axis:** Time interval (ms)
- **Y-axis:** Count (number of occurrences)
- **Blue bars:** Histogram of inter-frame intervals
- **Red dashed line:** Expected period
- **Green dashed line:** Measured mean

**Interpretation:**
- Single sharp peak: Excellent timing stability
- Broad distribution: Timing jitter
- Multiple peaks: Irregular frame transmission

### Plot 4: Inter-Frame Interval vs Time

- **X-axis:** Time (seconds)
- **Y-axis:** Inter-frame interval (ms)
- **Blue dots:** Measured intervals
- **Red dashed line:** Expected period

**Interpretation:**
- Flat line: Consistent timing
- Variations: Timing jitter or frame losses
- Spikes: Dropped frames (longer intervals)

---

## Algorithm Details

### Dominant Increment Detection

The script automatically detects the dominant sequence increment using mode statistics:

1. Compute differences between consecutive sequence numbers
2. Filter out negative or non-finite values
3. Find the most common positive difference
4. Use this as the "dominant increment"

**Why this is needed:**
- Sequence counter may not increment by 1 per frame
- Depends on FreeRTOS tick rate and frame rate
- Typical value: 26 for 1000 Hz tick rate and 38 Hz frame rate

### Frame Loss Detection

Frame losses are detected by checking if sequence increments are integer multiples of the dominant increment:

```
For each frame-to-frame increment:
  ratio = increment / dominant_increment
  if ratio ≈ integer and ratio > 1:
    estimated_lost_frames += ratio - 1
```

**Example:**
- Dominant increment: 26
- Observed increment: 52
- Ratio: 52 / 26 = 2
- Interpretation: 1 frame was lost (2 - 1 = 1)

### Timing Analysis

Inter-frame intervals are computed as differences between consecutive timestamps:

```
interval[i] = timestamp[i+1] - timestamp[i]
```

Statistics include:
- Mean: Average interval
- Std: Standard deviation (jitter)
- Min/Max: Extreme values

---

## Common Issues and Solutions

### Issue 1: Frame Rate Deviation

**Symptom:** "Frame rate error: 5.2%"

**Causes:**
- Embedded system clock drift
- USB timing variations
- High system load on embedded device

**Solutions:**
1. Check embedded system clock configuration
2. Verify FreeRTOS tick rate is correctly set
3. Reduce other system tasks during UART transmission
4. Use a longer test duration (60+ seconds) for better averaging

### Issue 2: Dropped Frames

**Symptom:** "Estimated lost UART frames: 5"

**Causes:**
- UART buffer overflow
- High system load
- Baud rate too high for system
- USB cable issues

**Solutions:**
1. Reduce baud rate (try 921600 or 460800)
2. Increase UART buffer size in firmware
3. Reduce other system tasks
4. Check USB cable quality and connections
5. Move USB cable away from EMI sources

### Issue 3: Irregular Increments

**Symptom:** "Irregular increments: 3"

**Causes:**
- Timing anomalies
- Sequence counter overflow/reset
- Transmission errors

**Solutions:**
1. Repeat the test to see if issue is consistent
2. Check for sequence counter overflow in firmware
3. Verify CRC is being calculated correctly
4. Check for EMI near UART connections

### Issue 4: High Timing Jitter

**Symptom:** "Std: 5.234 ms" (expected ~0.5 ms)

**Causes:**
- System load variations
- USB timing variations
- Interrupt latency

**Solutions:**
1. Reduce other system tasks
2. Use a different USB port
3. Check system CPU load during test
4. Consider using a dedicated UART interface instead of USB

---

## Detailed Analysis Examples

### Example 1: Perfect System

```
Actual UART frame rate: 38.000 Hz (expected: 38.0 Hz)
Frame rate error: 0.00%

Dominant sequence increment: 26

Sequence increment consistency analysis:
  Normal increments (=26): 379
  Drop events (integer multiples > 1): 0
  Estimated lost UART frames: 0
  Irregular increments: 0

Inter-frame interval statistics:
  Mean: 26.316 ms (expected: 26.316 ms)
  Std: 0.123 ms
  Min: 26.100 ms
  Max: 26.500 ms

Sequence growth rate: 988.000 counts/s
Predicted sequence growth rate: 988.000 counts/s
Difference: 0.000 counts/s

CONCLUSION: System is operating perfectly
```

### Example 2: System with Occasional Frame Loss

```
Actual UART frame rate: 37.500 Hz (expected: 38.0 Hz)
Frame rate error: 1.32%

Dominant sequence increment: 26

Sequence increment consistency analysis:
  Normal increments (=26): 368
  Drop events (integer multiples > 1): 2
  Estimated lost UART frames: 2
  Irregular increments: 0

Examples of inferred dropped-frame events:
  Seq 9620 -> 9698 | increment = 78 = 3 x 26 | estimated lost frames = 2
  Seq 10140 -> 10218 | increment = 78 = 3 x 26 | estimated lost frames = 2

CONCLUSION: 2 frames were lost during 10-second test
            Investigate UART buffer or system load issues
```

### Example 3: System with Timing Issues

```
Actual UART frame rate: 37.800 Hz (expected: 38.0 Hz)
Frame rate error: 0.58%

Dominant sequence increment: 26

Inter-frame interval statistics:
  Mean: 26.471 ms (expected: 26.316 ms)
  Std: 2.345 ms
  Min: 23.100 ms
  Max: 31.500 ms

CONCLUSION: High timing jitter detected
            Check system load and USB stability
```

---

## Advanced Usage

### Continuous Monitoring

To monitor UART performance over extended periods:

```python
import time

for hour in range(24):
    print(f"Running test {hour + 1}/24...")
    main()
    time.sleep(3600)  # Wait 1 hour before next test
```

### Batch Testing

To test multiple serial ports:

```python
ports = ["COM7", "COM8", "COM9"]

for port in ports:
    print(f"\nTesting port {port}...")
    serial_port = port
    main()
```

### Custom Analysis

Load and re-analyze saved data:

```python
import numpy as np

# Data is available in the main() function scope
# Can be modified to save and reload for offline analysis
```

---

## Performance Benchmarks

### Typical Performance

- **Test duration:** 10 seconds
- **Frames acquired:** 380-400 frames
- **Processing time:** < 1 second
- **Timing accuracy:** ±0.1 ms

### Expected Values for 38 Hz Frame Rate

| Metric | Expected | Tolerance |
|--------|----------|-----------|
| Frame rate | 38.0 Hz | ±1.0 Hz |
| Dominant increment | 26 | ±1 |
| Mean interval | 26.3 ms | ±0.5 ms |
| Std interval | < 1.0 ms | - |
| Lost frames | 0 | - |
| Irregular increments | 0 | - |

---

## UART Frame Format

```
Byte Offset | Field              | Size | Type
0-3         | Header             | 4    | 0xAA 0x55 0xAA 0x55
4-5         | Payload Length     | 2    | uint16_t (little-endian)
6-9         | Timestamp (tick)   | 4    | uint32_t (little-endian)
10-153      | Sensor Payload     | 144  | 12×3 float32 (little-endian)
154-155     | CRC16              | 2    | uint16_t (little-endian)
```

**Notes:**
- Timestamp is in FreeRTOS ticks (default 1000 Hz)
- Sequence increment depends on tick rate and frame rate
- CRC is calculated over all bytes except the CRC field itself

---

## Troubleshooting

### Serial Connection Issues

**Problem:** "Serial connection failed"

**Solutions:**
1. Verify serial port name in Device Manager
2. Ensure no other application is using the port
3. Check USB cable connection
4. Try a different USB port

### No Data Received

**Problem:** "Total received frames: 0"

**Solutions:**
1. Verify UART is transmitting data
2. Check serial port configuration (baud rate, parity, etc.)
3. Verify frame header (0xAA 0x55 0xAA 0x55)
4. Check CRC calculation

### Inconsistent Results

**Problem:** Different results on repeated runs

**Solutions:**
1. Increase test duration for better statistics
2. Reduce system load during test
3. Check for USB power issues
4. Verify stable power supply to embedded system

---

## References

### UART Communication
- RS-232 Standard: EIA/TIA-232-F
- Baud rate: 1024000 (non-standard, verify firmware support)

### Timing Analysis
- FreeRTOS: https://www.freertos.org/
- Tick rate configuration: `configTICK_RATE_HZ`

### Statistical Methods
- Mode detection: Most common value in dataset
- Linear regression: Least squares fitting

---

## Version History

- **v1.0** (2024-03-08): Initial release
  - Automatic dominant increment detection
  - Frame loss detection
  - Comprehensive timing analysis
  - Four-plot visualization
