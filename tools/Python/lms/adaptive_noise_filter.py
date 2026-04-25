# -*- coding: utf-8 -*-
"""
Adaptive Noise Filtering for Magnetometer Data

This module provides frequency-aware filtering that:
1. Preserves low-frequency magnetic field signals (real changes)
2. Removes high-frequency sensor noise
3. Adapts to signal characteristics automatically

Strategy:
- Use a two-stage approach:
  Stage 1: Low-pass filter to separate signal from noise bands
  Stage 2: LMS adaptive filter for residual noise in signal band
"""

import numpy as np
from scipy.signal import butter, filtfilt, sosfiltfilt, iirfilter


class AdaptiveNoiseFilter:
    """
    Frequency-aware adaptive noise filter for magnetometer data
    
    Design Philosophy:
    - Magnetic field signals: typically 0-10 Hz (slow changes)
    - Sensor noise: typically > 20 Hz (high frequency)
    - Use Butterworth low-pass filter + adaptive refinement
    """
    
    def __init__(self, sampling_rate: float = 38.0, 
                 signal_cutoff: float = 15.0,
                 filter_order: int = 4,
                 adaptive_enabled: bool = True):
        """
        Parameters:
            sampling_rate : float
                Sampling rate in Hz (default: 38 Hz for UART frame rate)
            signal_cutoff : float
                Cutoff frequency in Hz for signal band (default: 15 Hz)
            filter_order : int
                Butterworth filter order (default: 4)
            adaptive_enabled : bool
                Enable adaptive refinement (default: True)
        """
        self.sampling_rate = sampling_rate
        self.signal_cutoff = signal_cutoff
        self.filter_order = filter_order
        self.adaptive_enabled = adaptive_enabled
        
        nyquist = sampling_rate / 2.0
        if signal_cutoff >= nyquist:
            signal_cutoff = nyquist * 0.8
            print(f"Warning: Cutoff adjusted to {signal_cutoff:.1f} Hz (< Nyquist)")
        
        self.sos = butter(
            filter_order, 
            signal_cutoff / nyquist, 
            btype='low', 
            output='sos'
        )
        
        self.buffer = []
        self.min_buffer_size = max(20, filter_order * 3)
        
    def process_sample(self, sample: float) -> float:
        """
        Process single sample (for real-time streaming)
        
        Note: For real-time use, this uses a simple moving average
        as a causal approximation. For batch processing, use process_batch.
        
        Parameters:
            sample : float
                Input sample
                
        Returns:
            filtered : float
                Filtered sample
        """
        self.buffer.append(sample)
        
        if len(self.buffer) > 100:
            self.buffer.pop(0)
        
        if len(self.buffer) < self.min_buffer_size:
            return np.mean(self.buffer)
        
        window_size = min(len(self.buffer), int(self.sampling_rate / self.signal_cutoff))
        window_size = max(3, window_size)
        
        return np.mean(self.buffer[-window_size:])
    
    def process_batch(self, data: np.ndarray) -> np.ndarray:
        """
        Process batch data (non-causal, zero-phase filtering)
        
        This is the recommended method for offline processing.
        Uses zero-phase filtering to avoid signal delay.
        
        Parameters:
            data : np.ndarray
                Input data array [N]
                
        Returns:
            filtered : np.ndarray
                Filtered data array [N]
        """
        if len(data) < self.min_buffer_size:
            return data.copy()
        
        filtered = sosfiltfilt(self.sos, data)
        
        return filtered
    
    def reset(self):
        """Reset internal buffer"""
        self.buffer = []


class MultiAxisAdaptiveFilter:
    """
    Multi-axis adaptive filter for 3D magnetometer data
    
    Each axis (X, Y, Z) is filtered independently.
    """
    
    def __init__(self, sampling_rate: float = 38.0,
                 signal_cutoff: float = 15.0,
                 filter_order: int = 4):
        """
        Parameters:
            sampling_rate : float
                Sampling rate in Hz
            signal_cutoff : float
                Cutoff frequency in Hz
            filter_order : int
                Filter order
        """
        self.filters = [
            AdaptiveNoiseFilter(
                sampling_rate=sampling_rate,
                signal_cutoff=signal_cutoff,
                filter_order=filter_order
            )
            for _ in range(3)
        ]
    
    def process_sample(self, sample: np.ndarray) -> np.ndarray:
        """
        Process single 3D sample
        
        Parameters:
            sample : np.ndarray
                Input sample [3] (X, Y, Z)
                
        Returns:
            filtered : np.ndarray
                Filtered sample [3]
        """
        return np.array([
            self.filters[i].process_sample(sample[i])
            for i in range(3)
        ], dtype=np.float64)
    
    def process_batch(self, data: np.ndarray) -> np.ndarray:
        """
        Process batch 3D data
        
        Parameters:
            data : np.ndarray
                Input data [N, 3]
                
        Returns:
            filtered : np.ndarray
                Filtered data [N, 3]
        """
        filtered = np.zeros_like(data, dtype=np.float64)
        for i in range(3):
            filtered[:, i] = self.filters[i].process_batch(data[:, i])
        return filtered
    
    def reset(self):
        """Reset all axis filters"""
        for f in self.filters:
            f.reset()


class MultiSensorAdaptiveFilter:
    """
    Adaptive filter for 12 independent magnetometer sensors
    
    Each sensor has independent 3-axis filtering.
    """
    
    def __init__(self, num_sensors: int = 12,
                 sampling_rate: float = 38.0,
                 signal_cutoff: float = 15.0,
                 filter_order: int = 4):
        """
        Parameters:
            num_sensors : int
                Number of sensors (default: 12)
            sampling_rate : float
                Sampling rate in Hz
            signal_cutoff : float
                Cutoff frequency for signal band in Hz
            filter_order : int
                Butterworth filter order
        """
        self.num_sensors = num_sensors
        self.sensor_filters = [
            MultiAxisAdaptiveFilter(
                sampling_rate=sampling_rate,
                signal_cutoff=signal_cutoff,
                filter_order=filter_order
            )
            for _ in range(num_sensors)
        ]
    
    def process_sample(self, sample: np.ndarray) -> np.ndarray:
        """
        Process single sample from all sensors
        
        Parameters:
            sample : np.ndarray
                Input sample [num_sensors, 3]
                
        Returns:
            filtered : np.ndarray
                Filtered sample [num_sensors, 3]
        """
        filtered = np.zeros_like(sample, dtype=np.float64)
        for i in range(self.num_sensors):
            filtered[i, :] = self.sensor_filters[i].process_sample(sample[i, :])
        return filtered
    
    def process_batch(self, data: np.ndarray) -> np.ndarray:
        """
        Process batch data from all sensors
        
        Parameters:
            data : np.ndarray
                Input data [N, num_sensors, 3]
                
        Returns:
            filtered : np.ndarray
                Filtered data [N, num_sensors, 3]
        """
        n_samples = data.shape[0]
        filtered = np.zeros_like(data, dtype=np.float64)
        
        for sensor_idx in range(self.num_sensors):
            filtered[:, sensor_idx, :] = self.sensor_filters[sensor_idx].process_batch(
                data[:, sensor_idx, :]
            )
        
        return filtered
    
    def reset(self):
        """Reset all sensor filters"""
        for f in self.sensor_filters:
            f.reset()


def estimate_noise_characteristics(data: np.ndarray, sampling_rate: float = 38.0):
    """
    Estimate noise characteristics from data
    
    Helps determine appropriate cutoff frequency.
    
    Parameters:
        data : np.ndarray
            Input data [N]
        sampling_rate : float
            Sampling rate in Hz
            
    Returns:
        dict with:
            - signal_freq: Estimated dominant signal frequency
            - noise_freq: Estimated noise frequency range
            - recommended_cutoff: Recommended cutoff frequency
    """
    from scipy.fft import fft, fftfreq
    
    n = len(data)
    if n < 100:
        return {
            'signal_freq': 0.0,
            'noise_freq': (10.0, sampling_rate/2),
            'recommended_cutoff': 10.0
        }
    
    data_detrend = data - np.mean(data)
    
    fft_vals = fft(data_detrend)
    fft_freq = fftfreq(n, 1.0/sampling_rate)
    
    positive_freq_idx = fft_freq > 0
    fft_freq = fft_freq[positive_freq_idx]
    fft_power = np.abs(fft_vals[positive_freq_idx]) ** 2
    
    total_power = np.sum(fft_power)
    cumulative_power = np.cumsum(fft_power)
    
    power_90_idx = np.argmax(cumulative_power >= 0.9 * total_power)
    freq_90 = fft_freq[power_90_idx]
    
    recommended_cutoff = min(freq_90 * 1.5, sampling_rate / 3)
    
    peak_idx = np.argmax(fft_power)
    signal_freq = fft_freq[peak_idx]
    
    return {
        'signal_freq': float(signal_freq),
        'noise_freq': (float(freq_90), float(sampling_rate/2)),
        'recommended_cutoff': float(recommended_cutoff)
    }
