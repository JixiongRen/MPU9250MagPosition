# MPU9250 Magnetometer Calibration System

## Overview

This module provides a comprehensive calibration system for 12 independent MPU9250 magnetometer sensors. It acquires raw magnetic field data, computes hard iron and soft iron calibration parameters, and generates embedded-ready C header files for firmware integration.

**Key Features:**
- Real-time UART data acquisition from 12 sensors
- Stable ellipsoid fitting algorithm for calibration
- Hard iron (offset) and soft iron (scale/rotation) correction
- 3D visualization of raw and calibrated data
- Automatic C header file generation for embedded systems
- Comprehensive data quality diagnostics

---

## File Structure

```
calibrate/
├── README.md              # This file
└── mag_cali.py           # Main calibration script
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
  - `scipy`: Scientific computing
  - `matplotlib`: Data visualization
  - `pyserial`: Serial communication

### Installation

```bash
pip install numpy scipy matplotlib pyserial
```

---

## Usage

### Basic Workflow

1. **Prepare the hardware:**
   - Connect all 12 sensors to the embedded system
   - Ensure UART communication is working at 1024000 baud
   - Configure the serial port in the script (default: `COM7`)

2. **Run the calibration:**
   ```bash
   python mag_cali.py
   ```

3. **Perform calibration motion:**
   - When prompted, hold the board with all 12 sensors
   - Perform a **figure-eight motion** for 60 seconds
   - This ensures the sensors experience the full range of Earth's magnetic field
   - Move the board in all three dimensions to maximize data coverage

4. **Review results:**
   - The script displays 3D plots of raw and calibrated data
   - Check the calibration quality metrics (residuals, condition numbers)
   - Verify that calibrated data forms a sphere

5. **Extract calibration parameters:**
   - A `.h` file is automatically generated with calibration coefficients
   - A `.npz` file stores all raw data and parameters for future reference

### Configuration Parameters

Edit the `main()` function to customize:

```python
serial_port = "COM7"              # Serial port name
baud_rate = 1024000               # UART baud rate
num_sensors = 12                  # Number of sensors
calibration_duration = 60         # Acquisition time in seconds
fs = 100                          # Expected sampling rate (Hz)
```

---

## Algorithm Details

### Calibration Model

Raw magnetic field measurements are corrupted by two types of errors:

**Hard Iron Error (Offset):**
- Caused by permanent magnets and DC magnetic fields near the sensor
- Manifests as a constant offset in all measurements
- Correction: `B_corrected = B_raw - b_hard_iron`

**Soft Iron Error (Scale/Rotation):**
- Caused by ferromagnetic materials that distort the magnetic field
- Manifests as non-uniform scaling and rotation of measurements
- Correction: `B_corrected = M_soft_iron @ (B_raw - b_hard_iron)`

### Ellipsoid Fitting Algorithm

The calibration uses a **stable whitening method** to fit an ellipsoid to the raw magnetic field data:

1. **Compute hard iron offset:**
   ```
   b = mean(B_raw)
   X = B_raw - b
   ```

2. **Covariance matrix:**
   ```
   C = cov(X)
   ```

3. **Eigen decomposition:**
   ```
   [V, D] = eig(C)
   ```

4. **Whitening matrix (soft iron):**
   ```
   M = V @ diag(1/sqrt(D)) @ V^T
   ```

5. **Scale restoration:**
   - Restore the magnitude to match the Earth's magnetic field strength
   - Ensures calibrated vectors have consistent magnitude

6. **Residual calculation:**
   - Measures the standard deviation of calibrated vector magnitudes
   - Lower residuals indicate better calibration quality

### Numerical Stability

The algorithm includes safeguards:
- Eigenvalue clipping: `D[D < 1e-6] = 1e-6`
- Covariance regularization for rank-deficient matrices
- Condition number monitoring (warns if > 100)

---

## Output Files

### 1. Header File (`.h`)

Example: `mag_calibration_data_20240308_120000.h`

Contains C arrays ready for embedded firmware:

```c
const float mag_hard_iron[12][3] = {
    {0.123456, -0.234567, 0.345678},
    ...
};

const float mag_soft_iron[12][3][3] = {
    {
        {1.023456, 0.001234, -0.002345},
        {0.001234, 1.034567, 0.003456},
        {-0.002345, 0.003456, 1.045678}
    },
    ...
};
```

**Usage in firmware:**
```c
// Apply calibration
float B_raw[3] = {/* raw sensor data */};
float B_corrected[3];

// Subtract hard iron
for (int i = 0; i < 3; i++) {
    B_corrected[i] = B_raw[i] - mag_hard_iron[sensor_id][i];
}

// Apply soft iron matrix
float B_final[3] = {0, 0, 0};
for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        B_final[i] += mag_soft_iron[sensor_id][i][j] * B_corrected[j];
    }
}
```

### 2. Python Data File (`.npz`)

Example: `mag_calibration_20240308_120000.npz`

Contains all raw data and calibration parameters for offline analysis:
- `raw_mag_data_0` to `raw_mag_data_11`: Raw sensor measurements
- `hard_iron_0` to `hard_iron_11`: Hard iron offsets
- `soft_iron_0` to `soft_iron_11`: Soft iron matrices
- `residual_0` to `residual_11`: Fitting residuals
- `data_points_0` to `data_points_11`: Number of samples per sensor
- `cond_num_0` to `cond_num_11`: Condition numbers

**Load in Python:**
```python
import numpy as np

data = np.load('mag_calibration_20240308_120000.npz')
hard_iron_0 = data['hard_iron_0']
soft_iron_0 = data['soft_iron_0']
```

---

## Data Quality Diagnostics

The script performs automatic data quality checks:

### Diagnostic Output

```
Sensor ID | Data Count | X Range | Y Range | Z Range | Outlier Count
---       | ---        | ---     | ---     | ---     | ---
Sensor  1 |       6000 | [45]    | [52]    | [48]    |        12
```

### Quality Metrics

- **Data Count:** Number of valid samples acquired
  - Minimum recommended: 1000 samples
  - Better: 3000+ samples for robust calibration

- **Range (X/Y/Z):** Difference between max and min values
  - Indicates data coverage in each axis
  - Larger ranges = better calibration accuracy
  - Minimum recommended: 20 units per axis

- **Outlier Count:** Number of samples > 3σ from mean
  - Indicates data quality and noise level
  - Should be < 1% of total samples

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| Insufficient data | Acquisition time too short | Increase `calibration_duration` |
| Small data range | Limited motion during calibration | Perform more vigorous figure-eight motion |
| High outlier count | Electromagnetic interference | Move away from EMI sources |
| High condition number | Singular covariance matrix | Repeat calibration with better motion |

---

## Visualization

The script generates three visualization figures:

### 1. Raw Magnetic Field Data (3D Trajectories)

- 12 subplots showing 3D scatter plots of raw sensor data
- Blue dots: Measured data points
- Red star: Center (hard iron offset)
- Expected shape: Distorted ellipsoid

### 2. Calibrated Magnetic Field Data (3D Trajectories)

- 12 subplots showing 3D scatter plots of calibrated data
- Green dots: Calibrated data points
- Red star: Center (should be near origin)
- Red sphere: Reference sphere (unit radius)
- Expected shape: Sphere centered at origin

### 3. Time-Domain Comparison

- 12 subplots showing time-domain waveforms
- Blue line: Raw X-axis data
- Green line: Calibrated X-axis data
- Shows the effect of calibration over time

---

## Calibration Quality Assessment

### Residual Analysis

The **residual** is the standard deviation of calibrated vector magnitudes:

```
residual = std(||B_calibrated||)
```

**Interpretation:**
- `residual < 0.05`: Excellent calibration
- `residual < 0.1`: Good calibration
- `residual < 0.2`: Acceptable calibration
- `residual > 0.2`: Poor calibration, consider recalibrating

### Condition Number

The condition number of the soft iron matrix indicates numerical stability:

```
cond_num = max_eigenvalue / min_eigenvalue
```

**Interpretation:**
- `cond_num < 10`: Excellent (well-conditioned)
- `cond_num < 50`: Good
- `cond_num < 100`: Acceptable (warning issued)
- `cond_num > 100`: Poor (may indicate over-amplification)

---

## Troubleshooting

### Serial Connection Issues

**Problem:** "Serial connection failed"

**Solutions:**
1. Verify the serial port name (check Device Manager on Windows)
2. Ensure no other application is using the port
3. Check USB cable connection
4. Try a different USB port

### Insufficient Data

**Problem:** "Insufficient data" warnings for some sensors

**Solutions:**
1. Increase `calibration_duration` to 120 seconds
2. Ensure all 12 sensors are properly connected
3. Check for UART transmission errors (CRC failures)

### Poor Calibration Quality

**Problem:** High residuals or condition numbers

**Solutions:**
1. Repeat calibration with more vigorous motion
2. Ensure figure-eight motion covers all three dimensions
3. Move away from sources of electromagnetic interference
4. Check for hardware issues (loose connections, damaged sensors)

### CRC or Frame Errors

**Problem:** Many "CRC failures" or "Payload length mismatches"

**Solutions:**
1. Check UART cable quality and connections
2. Reduce baud rate if errors persist (change to 921600)
3. Check for EMI near the serial cable
4. Verify firmware is sending correct frame format

---

## Advanced Usage

### Batch Calibration

To calibrate multiple sets of sensors:

```python
for batch_num in range(num_batches):
    print(f"Calibrating batch {batch_num + 1}...")
    main()
    # Rename output files manually or programmatically
```

### Custom Calibration Duration

For sensors with different noise characteristics:

```python
calibration_duration = 120  # Longer for noisier sensors
```

### Offline Calibration

Load previously acquired data:

```python
import numpy as np

# Load raw data
data = np.load('mag_calibration_20240308_120000.npz')
raw_mag_data = [data[f'raw_mag_data_{i}'] for i in range(12)]

# Recompute calibration with different algorithm
# ... custom processing ...
```

---

## Performance Metrics

### Typical Performance

- **Acquisition rate:** ~38 Hz per sensor (1000 samples in 26 seconds)
- **Processing time:** ~2-5 seconds for 12 sensors
- **Calibration accuracy:** ±0.5-1% of Earth's magnetic field magnitude
- **Output file size:** ~50 KB for `.h` file, ~500 KB for `.npz` file

### Memory Requirements

- **RAM:** ~50 MB for 6000 samples × 12 sensors
- **Disk:** ~1 MB per calibration session

---

## References

### Magnetometer Calibration Theory

- Renaudin, V., et al. (2010). "Complete Triaxial Magnetometer Calibration in the Magnetic Domain"
- Crassidis, J. L., & Junkins, J. L. (2012). "Optimal Estimation of Attitude and Heading"

### UART Frame Format

```
Byte Offset | Field              | Size | Type
0-3         | Header             | 4    | 0xAA 0x55 0xAA 0x55
4-5         | Payload Length     | 2    | uint16_t (little-endian)
6-9         | Timestamp (tick)   | 4    | uint32_t (little-endian)
10-153      | Sensor Payload     | 144  | 12×3 float32 (little-endian)
154-155     | CRC16              | 2    | uint16_t (little-endian)
```

---

## License

This calibration system is part of the Magnetic Positioning Platform project.

---

## Support

For issues or questions:
1. Check the Troubleshooting section above
2. Review the diagnostic output for data quality issues
3. Verify hardware connections and UART communication
4. Consult the UART frame format documentation

---

## Version History

- **v1.0** (2024-03-08): Initial release
  - Stable ellipsoid fitting algorithm
  - Support for 12 independent sensors
  - C header file generation
  - Comprehensive visualization and diagnostics
