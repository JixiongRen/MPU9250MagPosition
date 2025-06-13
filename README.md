# STM32F4 Multi-Sensor Data Acquisition System

## Project Overview
This project is a multi-sensor data acquisition system based on the STM32F4 series microcontroller, utilizing the FreeRTOS real-time operating system to manage various tasks. The system connects multiple MPU9250 sensors via SPI bus to read accelerometer, gyroscope, and magnetometer data, and transmits processed data through UART interface.

## Hardware Configuration
- Main Controller: STM32F4 series
- Sensors: Multiple MPU9250 (9-axis motion sensors)
- Communication Interfaces:
    - 3 SPI buses (SPI1/2/3)
    - 2 I2C buses (I2C1/2)
    - 2 UART interfaces (UART1/2)

## Software Architecture
### Main Components
1. **Sensor Driver Layer**
    - MPU9250 driver (`MPU9250.c/h`): Implements communication and data reading from MPU9250 sensors
    - Software SPI driver: For simulating SPI communication (visible from code fragments)

2. **Data Management Layer**
    - Ring buffer (`ringbuffer.c/h`): For temporary data storage and transmission

3. **Task Management Layer**
    - FreeRTOS tasks (`freertos.c`):
        - Sensor data collection tasks (`rSpi01MagTask`, `rSpi02MagTask`, `rSpi03MagTask`)
        - UART data transmission task (`UsartTransTask`)

4. **Peripheral Configuration Layer**
    - GPIO configuration (`gpio.c`)
    - SPI/I2C/UART configuration (`spi.c`, `i2c.c`, `usart.c`)
    - DMA configuration (`dma.c`)

## Key Functionality Implementation

### Sensor Organization Structure
```
SPI_SensorsGroup
|-- MPU9250 sensor 1
|-- MPU9250 sensor 2
|-- MPU9250 sensor 3
|-- MPU9250 sensor 4
```

### Sensor Data Reading
The system supports reading the following data:
- Accelerometer data (`MPU9250_ReadAccel`)
- Gyroscope data (`MPU9250_ReadGyro`)
- Magnetometer data (`MPU9250_ReadMag`)

### Task Synchronization Mechanism
- Using semaphores (`samplingStartTask01/02/03`) to trigger sensor sampling tasks
- Using message queues (`sensorGroupQueue01/02/03`) to transfer sensor data
- Using event groups (`xSensorEventGroup`) for inter-task synchronization

### Ring Buffer
- Supports standard and interrupt-safe read/write operations
- Supports pointer operations for DMA data reception
- Configurable semaphores for synchronization

## Usage Instructions
1. Initialize sensor group
```c
SensorGroup_StructInit(spi_sensorsgroup, sensornum, ghspix, gcs_port, gcs_pin);
SensorGroup_Init(spi_sensorsgroup);
```

2. Start timer to trigger sampling
```c
HAL_TIM_Base_Start_IT(&htim1);
```

3. Read sensor data
```c
MPU9250_ReadData(mpu); // Read all data from a single sensor
// or
SensorsGroup_ReadData(spi_sensorsgroup); // Read data from an entire sensor group
```

## Important Notes
- The system uses timer interrupts to trigger data sampling
- MPU9250 is configured in SPI mode to access the internal AK8963 magnetometer
- Ring buffer requires attention to memory management and thread safety

## Extended Features
- Supports parallel data acquisition from up to three sensor arrays
- Different sampling frequencies and data precision can be achieved through configuration
- DMA transfer mechanism improves data transmission efficiency

## Project Structure
- `main.c`: Main program entry, system initialization
- `freertos.c`: FreeRTOS task creation and management
- `MPU9250.c`: MPU9250 sensor driver
- `ringbuffer.c`: Ring buffer implementation
- Peripheral configuration files: `gpio.c`, `spi.c`, `i2c.c`, `dma.c`, `usart.c`

## Software SPI Interface
The system also implements a software SPI interface by directly controlling SPI signal lines through GPIO, for specialized communication requirements.