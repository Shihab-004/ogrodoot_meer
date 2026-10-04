"""
rover_vision — Computer Vision Subsystem for Tree Vitality Detection
MEER Rover NAGC 2026 Autonomous Navigation Mission
"""

from rover_vision.classifier_interface import (
    TreeClassifierBase,
    VitalityResult,
    FoliarSpectralClassifier,
    DeepLearningClassifier,
    create_classifier,
)
from rover_vision.tree_detector import TreeDetector, DetectionResult

__all__ = [
    'TreeClassifierBase',
    'VitalityResult',
    'FoliarSpectralClassifier',
    'DeepLearningClassifier',
    'create_classifier',
    'TreeDetector',
    'DetectionResult',
]
