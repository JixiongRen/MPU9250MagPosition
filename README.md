# STM32F4 12-Sensor Magnetometer Acquisition Firmware

## 1. Overview
This firmware runs on STM32F4 (FreeRTOS based) and acquires magnetometer data from 12 MPU9250 sensors (AK8963 via MPU9250 internal I2C master), then streams framed telemetry through UART2 DMA.

The current project has been updated from a basic multi-sensor demo to a diagnostics-oriented telemetry firmware with:
- fixed binary frame protocol (header + length + tick + payload + CRC16)
- runtime sensor diagnostics packed in payload
- automatic DC bias (offset) elimination after startup
- host-side Python tooling for reception, checks, plots, and filtering

## 2. Hardware Topology
- MCU: STM32F4 series
- Sensors: 12 x MPU9250, grouped as 3 SPI groups x 4 sensors
- SPI buses:
    - Group1 on SPI1
    - Group2 on SPI2
    - Group3 on SPI3
- UART:
    - UART2 is telemetry output (DMA TX)

## 3. Runtime Architecture (Firmware)

### 3.1 FreeRTOS Tasks
- rSpi01MagTask: read magnetometers for group 1
- rSpi02MagTask: read magnetometers for group 2
- rSpi03MagTask: read magnetometers for group 3
- UsartTransTask: waits for all groups, applies auto-zero logic, builds frame, sends UART DMA

### 3.2 Synchronization
- TIM1 ISR releases three semaphores:
    - samplingStartTask01
    - samplingStartTask02
    - samplingStartTask03
- each SPI task pushes latest group pointer via queue overwrite
- event bits (xSensorEventGroup) are used so UsartTransTask waits until all 3 groups are ready

### 3.3 Sampling Rate
TIM1 config:
- prescaler = 8399
- period = 199

With current clock tree, this results in 100 Hz sampling/telemetry cycle.

## 4. Auto-Zero (DC Bias Elimination)
The firmware uses a startup auto-zero state machine:
- WAIT_VALID: wait until all 12 sensors report valid mag output
- COLLECTING: accumulate 150 frames of 36 channels (12 x 3)
- READY: compute per-channel offset and subtract it from outgoing Mag values

Only after entering READY state does UsartTransTask transmit telemetry frames.

LED behavior is tied to state:
- INIT_BLINK: initialization / waiting valid data
- AUTO_ZERO_BLINK: collecting offset samples
- READY_ON: auto-zero done, normal streaming

## 5. UART Frame Protocol

### 5.1 Frame Layout
- Header: 4 bytes = AA 55 AA 55
- Payload length: uint16 little-endian
- Timestamp: uint32 little-endian (FreeRTOS tick)
- Payload: variable-length telemetry payload
- CRC16: uint16 little-endian

CRC algorithm:
- CRC16-CCITT-FALSE
- poly = 0x1021
- init = 0xFFFF

Total frame length:
- 4 + 2 + 4 + payload_len + 2

### 5.2 Current Payload (V5)
Current firmware payload length is 408 bytes, composed of:
- Magnetometer data: 36 float32 = 144 bytes
- Group sample counters: 3 x uint32 = 12 bytes
- Per-sensor diag uint8 fields: 13 x 12 = 156 bytes
- Per-sensor counters: 2 x 12 x uint32 = 96 bytes

Per-sensor diag fields include:
- whoami and init status
- mag read status
- I2C master/aux/slave4 debug bytes
- mag read success/attempt counters

## 6. Key Source Files

### Firmware
- Core/Src/freertosTasks.c
    - telemetry payload construction
    - auto-zero logic
    - task main loops
- Core/Src/frameProtocol.c
    - frame build and CRC16
- Core/Inc/MPU9250.h
    - MPU9250 struct and diagnostic fields
- Core/Src/main.c
    - sensor group mapping
    - TIM1 ISR semaphore release
    - LED state update

### Host Tools
- tools/usart_monitor.py
    - protocol auto-detection (diag text / frame)
    - frame decoding and diagnostics
- tools/capture_mag_plots.py
    - framed data capture and SVG report generation
- tools/Python/
    - LMS/adaptive filtering, sequence-rate check, calibration workflows

## 7. Typical Data Flow
1. TIM1 interrupt triggers all 3 sampling semaphores.
2. Three SPI tasks read magnetometer values from each group.
3. UsartTransTask waits until all group bits are set.
4. Auto-zero state machine updates; before READY, frames are not transmitted.
5. READY state: build telemetry payload, wrap with frame header/CRC.
6. UART2 DMA transmits frame.

## 8. Notes for Integration
- Host parsers should always trust payload_len and CRC, not a hardcoded frame size.
- First 144 payload bytes are 12 x 3 float32 magnetometer channels.
- Remaining payload bytes are diagnostics/counters and may evolve.
- If adding new payload fields, update both firmware builder and host decoder constants.

## 9. Build and Run
- Open with STM32CubeIDE or use the provided CMake workflow in this repository.
- Flash target board.
- Connect UART2 to host.
- Use tools/usart_monitor.py or tools/capture_mag_plots.py for validation.