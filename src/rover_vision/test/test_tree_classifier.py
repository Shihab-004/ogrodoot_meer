"""
test_tree_classifier.py — Unit Tests for Tree Detection and Vitality Classification
Verifies ALIVE vs DEAD discrimination, confidence scores, and edge cases.
"""

import sys
import os
import unittest
import numpy as np
import cv2

# Ensure local package path is accessible during test execution
pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

from rover_vision.classifier_interface import (
    FoliarSpectralClassifier,
    DeepLearningClassifier,
    VitalityResult,
    create_classifier,
)
from rover_vision.tree_detector import TreeDetector


def generate_synthetic_tree_image(is_alive: bool = True) -> np.ndarray:
    """Generates synthetic 640x480 scene representing outdoor rover view of a tree."""
    # Sand/ground background
    img = np.full((480, 640, 3), (120, 150, 180), dtype=np.uint8)  # BGR sand/ground

    # Trunk: brown cylinder at center (x=290..350, y=220..460)
    # BGR brown: (25, 60, 110)
    cv2.rectangle(img, (290, 220), (350, 460), (25, 60, 110), -1)

    # Canopy: sphere/ellipse above trunk
    if is_alive:
        # Vibrant chlorophyll green foliage: BGR (25, 175, 30)
        cv2.circle(img, (320, 160), 100, (25, 175, 30), -1)
        cv2.circle(img, (260, 180), 60, (20, 160, 25), -1)
        cv2.circle(img, (380, 180), 60, (20, 160, 25), -1)
    else:
        # Dead / desiccated brown/grey foliage & bare branches: BGR (30, 60, 85)
        cv2.circle(img, (320, 160), 90, (35, 65, 90), -1)
        cv2.circle(img, (270, 180), 50, (30, 60, 85), -1)
        cv2.circle(img, (370, 180), 50, (30, 60, 85), -1)

    return img


class TestTreeVitalityClassification(unittest.TestCase):

    def test_alive_tree_classification(self):
        """Verify that an alive tree with green canopy is classified as ALIVE with high confidence."""
        img_alive = generate_synthetic_tree_image(is_alive=True)
        detector = TreeDetector()
        det = detector.detect(img_alive)

        self.assertTrue(det.tree_detected)
        self.assertGreater(det.bbox[2], 0)
        self.assertGreater(det.bbox[3], 0)

        classifier = FoliarSpectralClassifier()
        result = classifier.classify(img_alive, det.bbox)

        self.assertEqual(result.verdict, "ALIVE")
        self.assertGreaterEqual(result.confidence, 0.70)
        self.assertGreater(result.greenness_index, 0.0)
        self.assertGreater(result.foliage_ratio, 0.05)
        self.assertIn("Active photosynthetic canopy", result.details)

    def test_dead_tree_classification(self):
        """Verify that a dead tree with necrotic/desiccated canopy is classified as DEAD."""
        img_dead = generate_synthetic_tree_image(is_alive=False)
        detector = TreeDetector()
        det = detector.detect(img_dead)

        self.assertTrue(det.tree_detected)

        classifier = FoliarSpectralClassifier()
        result = classifier.classify(img_dead, det.bbox)

        self.assertEqual(result.verdict, "DEAD")
        self.assertGreaterEqual(result.confidence, 0.70)
        self.assertLess(result.foliage_ratio, 0.05)
        self.assertIn("Necrotic or leafless crown", result.details)

    def test_blank_image_handling(self):
        """Verify that empty or blank images are handled gracefully without exceptions."""
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        classifier = FoliarSpectralClassifier()
        result = classifier.classify(blank)

        self.assertEqual(result.verdict, "DEAD")
        self.assertGreater(result.confidence, 0.0)

        none_res = classifier.classify(None)
        self.assertEqual(none_res.verdict, "UNKNOWN")
        self.assertEqual(none_res.confidence, 0.0)

    def test_deep_learning_fallback_safety(self):
        """Verify that deep learning classifier safely falls back to spectral baseline when no weights."""
        img = generate_synthetic_tree_image(is_alive=True)
        dl_classifier = create_classifier(classifier_type="deep_learning", model_path="")
        result = dl_classifier.classify(img)

        self.assertEqual(result.verdict, "ALIVE")
        self.assertIn("Pluggable DL ready", result.details)


if __name__ == '__main__':
    unittest.main()
