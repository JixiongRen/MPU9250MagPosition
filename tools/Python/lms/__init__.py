# -*- coding: utf-8 -*-
"""
LMS Adaptive Filtering Module for 12-Sensor Magnetometer System

This module provides independent LMS adaptive filtering for 12 MPU9250
magnetometer sensors with no cross-sensor dependencies.
"""

from .lms_filter import LMSFilter, MultiChannelLMSFilter
from .sensor_lms_processor import (
    ReferenceSignalGenerator,
    SensorLMSProcessor,
    MultiSensorLMSProcessor
)
from .lms_realtime import RealtimeLMSFilter

__version__ = "1.0.0"
__author__ = "Cascade AI"

__all__ = [
    "LMSFilter",
    "MultiChannelLMSFilter",
    "ReferenceSignalGenerator",
    "SensorLMSProcessor",
    "MultiSensorLMSProcessor",
    "RealtimeLMSFilter",
]
