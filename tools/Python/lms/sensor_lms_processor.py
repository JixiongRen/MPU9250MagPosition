# -*- coding: utf-8 -*-
"""
12-Sensor Independent LMS Processor

Manages independent LMS adaptive filtering for 12 magnetometer sensors.
Each sensor has 3 axes (X, Y, Z) with independent filtering.

Design:
- 12 sensors × 3 axes = 36 independent LMS filters
- No cross-sensor dependencies
- Configurable reference signal generation strategies
"""

import numpy as np
from lms_filter import MultiChannelLMSFilter


class ReferenceSignalGenerator:
    """
    Generate reference signals for LMS adaptive filtering
    
    Strategies:
        1. 'delayed': Use delayed version of input signal
        2. 'moving_average': Use moving average as reference
        3. 'zero': Use zero reference (noise cancellation mode)
    """
    
    def __init__(self, strategy: str = 'delayed', delay: int = 1, window_size: int = 5):
        """
        Parameters:
            strategy : str
                Reference generation strategy
            delay : int
                Delay samples for 'delayed' strategy
            window_size : int
                Window size for 'moving_average' strategy
        """
        self.strategy = strategy
        self.delay = delay
        self.window_size = window_size
        
        self.buffer = []
        
    def generate(self, input_sample: float) -> float:
        """
        Generate reference signal for current input sample
        
        Parameters:
            input_sample : float
                Current input signal sample
                
        Returns:
            reference : float
                Reference signal for LMS
        """
        self.buffer.append(input_sample)
        
        if self.strategy == 'delayed':
            if len(self.buffer) <= self.delay:
                return 0.0
            return self.buffer[-self.delay - 1]
        
        elif self.strategy == 'moving_average':
            if len(self.buffer) < self.window_size:
                return np.mean(self.buffer)
            return np.mean(self.buffer[-self.window_size:])
        
        elif self.strategy == 'zero':
            return 0.0
        
        else:
            raise ValueError(f"Unknown strategy: {self.strategy}")
    
    def reset(self):
        """Reset internal buffer"""
        self.buffer = []


class SensorLMSProcessor:
    """
    LMS Processor for a single sensor (3 axes)
    
    Each axis has an independent LMS filter and reference generator.
    """
    
    def __init__(self, sensor_id: int, filter_order: int = 8, 
                 step_size: float = 0.001, reference_strategy: str = 'delayed',
                 reference_delay: int = 1, reference_window: int = 5):
        """
        Parameters:
            sensor_id : int
                Sensor identifier (1-12)
            filter_order : int
                LMS filter order
            step_size : float
                LMS step size (learning rate)
            reference_strategy : str
                Reference signal generation strategy
            reference_delay : int
                Delay for 'delayed' strategy
            reference_window : int
                Window size for 'moving_average' strategy
        """
        self.sensor_id = sensor_id
        self.filter_order = filter_order
        self.step_size = step_size
        
        self.lms_filter = MultiChannelLMSFilter(
            num_channels=3,
            filter_order=filter_order,
            step_size=step_size
        )
        
        self.ref_generators = [
            ReferenceSignalGenerator(
                strategy=reference_strategy,
                delay=reference_delay,
                window_size=reference_window
            )
            for _ in range(3)
        ]
        
        self.raw_data_history = []
        self.filtered_data_history = []
        
    def process_sample(self, raw_sample: np.ndarray) -> np.ndarray:
        """
        Process one sample (3 axes)
        
        Parameters:
            raw_sample : np.ndarray
                Raw sensor data [3] (X, Y, Z)
                
        Returns:
            filtered_sample : np.ndarray
                Filtered sensor data [3]
        """
        if len(raw_sample) != 3:
            raise ValueError("raw_sample must have 3 elements (X, Y, Z)")
        
        desired_signals = np.array([
            self.ref_generators[i].generate(raw_sample[i])
            for i in range(3)
        ], dtype=np.float64)
        
        noise_estimates, errors = self.lms_filter.update(raw_sample, desired_signals)
        
        filtered_sample = desired_signals
        
        self.raw_data_history.append(raw_sample.copy())
        self.filtered_data_history.append(filtered_sample.copy())
        
        return filtered_sample
    
    def process_batch(self, raw_data: np.ndarray) -> np.ndarray:
        """
        Process batch data
        
        Parameters:
            raw_data : np.ndarray
                Raw sensor data [N, 3]
                
        Returns:
            filtered_data : np.ndarray
                Filtered sensor data [N, 3]
        """
        n_samples = raw_data.shape[0]
        filtered_data = np.zeros_like(raw_data, dtype=np.float64)
        
        for i in range(n_samples):
            filtered_data[i, :] = self.process_sample(raw_data[i, :])
        
        return filtered_data
    
    def reset(self):
        """Reset filter state"""
        self.lms_filter.reset()
        for gen in self.ref_generators:
            gen.reset()
        self.raw_data_history = []
        self.filtered_data_history = []
    
    def get_raw_data_history(self) -> np.ndarray:
        """Get raw data history"""
        if len(self.raw_data_history) == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return np.array(self.raw_data_history, dtype=np.float64)
    
    def get_filtered_data_history(self) -> np.ndarray:
        """Get filtered data history"""
        if len(self.filtered_data_history) == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return np.array(self.filtered_data_history, dtype=np.float64)
    
    def get_mse_per_axis(self) -> np.ndarray:
        """Get MSE for each axis"""
        return self.lms_filter.get_all_mse()


class MultiSensorLMSProcessor:
    """
    LMS Processor for 12 independent sensors
    
    Each sensor is processed independently with no cross-sensor dependencies.
    """
    
    def __init__(self, num_sensors: int = 12, filter_order: int = 8,
                 step_size: float = 0.001, reference_strategy: str = 'delayed',
                 reference_delay: int = 1, reference_window: int = 5):
        """
        Parameters:
            num_sensors : int
                Number of sensors (default: 12)
            filter_order : int
                LMS filter order for each sensor
            step_size : float
                LMS step size for each sensor
            reference_strategy : str
                Reference signal generation strategy
            reference_delay : int
                Delay for 'delayed' strategy
            reference_window : int
                Window size for 'moving_average' strategy
        """
        self.num_sensors = num_sensors
        
        self.sensor_processors = [
            SensorLMSProcessor(
                sensor_id=i + 1,
                filter_order=filter_order,
                step_size=step_size,
                reference_strategy=reference_strategy,
                reference_delay=reference_delay,
                reference_window=reference_window
            )
            for i in range(num_sensors)
        ]
    
    def process_sample(self, raw_samples: np.ndarray) -> np.ndarray:
        """
        Process one sample from all sensors
        
        Parameters:
            raw_samples : np.ndarray
                Raw sensor data [num_sensors, 3]
                
        Returns:
            filtered_samples : np.ndarray
                Filtered sensor data [num_sensors, 3]
        """
        if raw_samples.shape != (self.num_sensors, 3):
            raise ValueError(f"Expected shape ({self.num_sensors}, 3)")
        
        filtered_samples = np.zeros_like(raw_samples, dtype=np.float64)
        
        for i in range(self.num_sensors):
            filtered_samples[i, :] = self.sensor_processors[i].process_sample(
                raw_samples[i, :]
            )
        
        return filtered_samples
    
    def process_batch(self, raw_data: np.ndarray) -> np.ndarray:
        """
        Process batch data from all sensors
        
        Parameters:
            raw_data : np.ndarray
                Raw sensor data [N, num_sensors, 3]
                
        Returns:
            filtered_data : np.ndarray
                Filtered sensor data [N, num_sensors, 3]
        """
        n_samples = raw_data.shape[0]
        filtered_data = np.zeros_like(raw_data, dtype=np.float64)
        
        for i in range(n_samples):
            filtered_data[i, :, :] = self.process_sample(raw_data[i, :, :])
        
        return filtered_data
    
    def reset(self):
        """Reset all sensor processors"""
        for proc in self.sensor_processors:
            proc.reset()
    
    def get_sensor_processor(self, sensor_idx: int) -> SensorLMSProcessor:
        """Get processor for a specific sensor"""
        if not (0 <= sensor_idx < self.num_sensors):
            raise ValueError(f"sensor_idx must be in [0, {self.num_sensors})")
        return self.sensor_processors[sensor_idx]
    
    def get_all_mse(self) -> np.ndarray:
        """
        Get MSE for all sensors and axes
        
        Returns:
            mse_matrix : np.ndarray
                MSE values [num_sensors, 3]
        """
        mse_matrix = np.zeros((self.num_sensors, 3), dtype=np.float64)
        for i in range(self.num_sensors):
            mse_matrix[i, :] = self.sensor_processors[i].get_mse_per_axis()
        return mse_matrix
