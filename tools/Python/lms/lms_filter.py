# -*- coding: utf-8 -*-
"""
LMS Adaptive Filter Core Implementation

Implements the standard Least Mean Squares (LMS) adaptive filtering algorithm.
Supports both single-channel and multi-channel filtering.

Algorithm:
    y[n] = w[n]^T * x[n]           (filter output)
    e[n] = d[n] - y[n]              (error signal)
    w[n+1] = w[n] + μ * e[n] * x[n] (weight update)

where:
    x[n] : input signal vector (filter taps)
    d[n] : desired signal (reference)
    y[n] : filter output
    e[n] : error signal
    w[n] : filter weights
    μ    : step size (learning rate)
"""

import numpy as np


class LMSFilter:
    """
    Standard LMS Adaptive Filter
    
    Parameters:
        filter_order : int
            Number of filter taps (filter order)
        step_size : float
            Learning rate μ (0 < μ < 2/λmax, typically 0.01-0.1)
        leak_factor : float, optional
            Leaky LMS factor (0 < leak_factor <= 1), default=1.0 (standard LMS)
    """
    
    def __init__(self, filter_order: int, step_size: float, leak_factor: float = 1.0):
        if filter_order < 1:
            raise ValueError("filter_order must be >= 1")
        if step_size <= 0:
            raise ValueError("step_size must be > 0")
        if not (0 < leak_factor <= 1.0):
            raise ValueError("leak_factor must be in (0, 1]")
        
        self.filter_order = filter_order
        self.step_size = step_size
        self.leak_factor = leak_factor
        
        self.weights = np.zeros(filter_order, dtype=np.float64)
        self.input_buffer = np.zeros(filter_order, dtype=np.float64)
        
        self.error_history = []
        self.output_history = []
        self.weight_norm_history = []
        
    def reset(self):
        """Reset filter state"""
        self.weights = np.zeros(self.filter_order, dtype=np.float64)
        self.input_buffer = np.zeros(self.filter_order, dtype=np.float64)
        self.error_history = []
        self.output_history = []
        self.weight_norm_history = []
        
    def update(self, input_sample: float, desired_sample: float) -> tuple:
        """
        Update filter with one sample
        
        Parameters:
            input_sample : float
                Current input signal sample
            desired_sample : float
                Current desired signal sample (reference)
                
        Returns:
            output : float
                Filter output y[n]
            error : float
                Error signal e[n]
        """
        self.input_buffer = np.roll(self.input_buffer, 1)
        self.input_buffer[0] = input_sample
        
        output = np.dot(self.weights, self.input_buffer)
        
        error = desired_sample - output
        
        input_power = np.dot(self.input_buffer, self.input_buffer)
        if input_power < 1e-10:
            input_power = 1e-10
        
        normalized_step = self.step_size / (input_power + 1e-6)
        
        self.weights = self.leak_factor * self.weights + \
                       normalized_step * error * self.input_buffer
        
        weight_norm = np.linalg.norm(self.weights)
        if weight_norm > 1e6 or not np.all(np.isfinite(self.weights)):
            self.weights = np.zeros(self.filter_order, dtype=np.float64)
            weight_norm = 0.0
        
        self.error_history.append(error)
        self.output_history.append(output)
        self.weight_norm_history.append(weight_norm)
        
        return output, error
    
    def filter_batch(self, input_signal: np.ndarray, desired_signal: np.ndarray) -> tuple:
        """
        Filter a batch of samples
        
        Parameters:
            input_signal : np.ndarray
                Input signal array [N]
            desired_signal : np.ndarray
                Desired signal array [N]
                
        Returns:
            output_signal : np.ndarray
                Filter output array [N]
            error_signal : np.ndarray
                Error signal array [N]
        """
        if input_signal.shape != desired_signal.shape:
            raise ValueError("input_signal and desired_signal must have the same shape")
        
        n_samples = len(input_signal)
        output_signal = np.zeros(n_samples, dtype=np.float64)
        error_signal = np.zeros(n_samples, dtype=np.float64)
        
        for i in range(n_samples):
            output_signal[i], error_signal[i] = self.update(
                input_signal[i], desired_signal[i]
            )
        
        return output_signal, error_signal
    
    def get_weights(self) -> np.ndarray:
        """Get current filter weights"""
        return self.weights.copy()
    
    def get_error_history(self) -> np.ndarray:
        """Get error signal history"""
        return np.array(self.error_history, dtype=np.float64)
    
    def get_output_history(self) -> np.ndarray:
        """Get output signal history"""
        return np.array(self.output_history, dtype=np.float64)
    
    def get_weight_norm_history(self) -> np.ndarray:
        """Get weight norm history (for convergence analysis)"""
        return np.array(self.weight_norm_history, dtype=np.float64)
    
    def get_mse(self) -> float:
        """Get mean squared error over all processed samples"""
        if len(self.error_history) == 0:
            return 0.0
        return float(np.mean(np.array(self.error_history) ** 2))


class MultiChannelLMSFilter:
    """
    Multi-channel LMS Filter
    
    Each channel has an independent LMS filter instance.
    Useful for filtering multi-axis sensor data independently.
    
    Parameters:
        num_channels : int
            Number of independent channels
        filter_order : int
            Filter order for each channel
        step_size : float
            Step size for each channel
        leak_factor : float, optional
            Leaky LMS factor, default=1.0
    """
    
    def __init__(self, num_channels: int, filter_order: int, 
                 step_size: float, leak_factor: float = 1.0):
        if num_channels < 1:
            raise ValueError("num_channels must be >= 1")
        
        self.num_channels = num_channels
        self.filters = [
            LMSFilter(filter_order, step_size, leak_factor)
            for _ in range(num_channels)
        ]
    
    def reset(self):
        """Reset all channel filters"""
        for f in self.filters:
            f.reset()
    
    def update(self, input_samples: np.ndarray, desired_samples: np.ndarray) -> tuple:
        """
        Update all channels with one sample per channel
        
        Parameters:
            input_samples : np.ndarray
                Input samples [num_channels]
            desired_samples : np.ndarray
                Desired samples [num_channels]
                
        Returns:
            outputs : np.ndarray
                Filter outputs [num_channels]
            errors : np.ndarray
                Error signals [num_channels]
        """
        if len(input_samples) != self.num_channels:
            raise ValueError(f"Expected {self.num_channels} input samples")
        if len(desired_samples) != self.num_channels:
            raise ValueError(f"Expected {self.num_channels} desired samples")
        
        outputs = np.zeros(self.num_channels, dtype=np.float64)
        errors = np.zeros(self.num_channels, dtype=np.float64)
        
        for i in range(self.num_channels):
            outputs[i], errors[i] = self.filters[i].update(
                input_samples[i], desired_samples[i]
            )
        
        return outputs, errors
    
    def filter_batch(self, input_signals: np.ndarray, 
                     desired_signals: np.ndarray) -> tuple:
        """
        Filter batch data for all channels
        
        Parameters:
            input_signals : np.ndarray
                Input signals [N, num_channels]
            desired_signals : np.ndarray
                Desired signals [N, num_channels]
                
        Returns:
            output_signals : np.ndarray
                Filter outputs [N, num_channels]
            error_signals : np.ndarray
                Error signals [N, num_channels]
        """
        if input_signals.shape != desired_signals.shape:
            raise ValueError("input_signals and desired_signals must have the same shape")
        if input_signals.shape[1] != self.num_channels:
            raise ValueError(f"Expected {self.num_channels} channels")
        
        n_samples = input_signals.shape[0]
        output_signals = np.zeros_like(input_signals, dtype=np.float64)
        error_signals = np.zeros_like(input_signals, dtype=np.float64)
        
        for i in range(n_samples):
            output_signals[i, :], error_signals[i, :] = self.update(
                input_signals[i, :], desired_signals[i, :]
            )
        
        return output_signals, error_signals
    
    def get_channel_filter(self, channel_idx: int) -> LMSFilter:
        """Get filter instance for a specific channel"""
        if not (0 <= channel_idx < self.num_channels):
            raise ValueError(f"channel_idx must be in [0, {self.num_channels})")
        return self.filters[channel_idx]
    
    def get_all_mse(self) -> np.ndarray:
        """Get MSE for all channels"""
        return np.array([f.get_mse() for f in self.filters], dtype=np.float64)
