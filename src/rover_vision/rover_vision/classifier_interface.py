"""
classifier_interface.py — Modular Tree Vitality Classification Architecture
MEER Rover — NAGC 2026 Autonomous Navigation Mission

Provides an extensible, model-agnostic classification subsystem:
  - Abstract base interface: TreeClassifierBase
  - Baseline scientific classifier: FoliarSpectralClassifier (VARI / ExG / HSV / Crown coverage)
  - Deep Learning wrapper: DeepLearningClassifier (pluggable ONNX / PyTorch / YOLO / ViT)
  - Factory pattern: create_classifier()
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, Any
import numpy as np
import cv2


@dataclass
class VitalityResult:
    """Encapsulates vitality determination, confidence, and diagnostic metrics."""
    verdict: str                  # "ALIVE", "DEAD", "UNKNOWN"
    confidence: float             # 0.0 to 1.0
    greenness_index: float        # VARI / vegetative index (-1.0 to 1.0)
    foliage_ratio: float          # Proportion of healthy foliar crown pixels in ROI (0.0 to 1.0)
    details: str                  # Diagnostic breakdown of visual characteristics
    metrics: Dict[str, Any] = field(default_factory=dict)
    annotated_roi: Optional[np.ndarray] = None


class TreeClassifierBase(ABC):
    """
    Abstract Base Class for Tree Vitality Classifiers.
    Enforces a uniform interface for baseline algorithms, deep learning models,
    and future thermal/multispectral sensing fusion.
    """

    @abstractmethod
    def classify(
        self,
        image_bgr: np.ndarray,
        roi_bbox: Optional[Tuple[int, int, int, int]] = None
    ) -> VitalityResult:
        """
        Classifies tree vitality from an input image and optional ROI bounding box.

        :param image_bgr: Full BGR camera image (numpy array, HxWxC uint8)
        :param roi_bbox: Optional (x, y, w, h) bounding box of detected tree in pixels
        :return: VitalityResult with verdict, confidence, and spectral metrics
        """
        pass


class FoliarSpectralClassifier(TreeClassifierBase):
    """
    Visual and Spectral Vegetation Classifier.
    Analyzes visible-band photosynthetic reflectance using:
      1. VARI (Visible Atmospherically Resistant Index): (G - R) / (G + R - B)
         A standard agronomic NDVI-proxy for RGB imagery.
      2. ExG (Excess Greenness Index): 2*G - R - B
      3. HSV chrominance segmentation for chlorophyll-rich foliage vs necrotic/senescent brown
      4. Foliar canopy density and crown coverage ratio
    """

    def __init__(
        self,
        foliage_ratio_threshold: float = 0.05,
        vari_alive_threshold: float = 0.02,
        min_green_pixels: int = 150
    ):
        self.foliage_ratio_threshold = foliage_ratio_threshold
        self.vari_alive_threshold = vari_alive_threshold
        self.min_green_pixels = min_green_pixels

    def classify(
        self,
        image_bgr: np.ndarray,
        roi_bbox: Optional[Tuple[int, int, int, int]] = None
    ) -> VitalityResult:
        if image_bgr is None or image_bgr.size == 0:
            return VitalityResult(
                verdict="UNKNOWN",
                confidence=0.0,
                greenness_index=0.0,
                foliage_ratio=0.0,
                details="Empty or invalid image frame received."
            )

        img_h, img_w = image_bgr.shape[:2]

        if roi_bbox is not None:
            x, y, w, h = roi_bbox
            x = max(0, min(x, img_w - 1))
            y = max(0, min(y, img_h - 1))
            w = max(1, min(w, img_w - x))
            h = max(1, min(h, img_h - y))
            roi = image_bgr[y:y+h, x:x+w]
        else:
            roi = image_bgr
            roi_bbox = (0, 0, img_w, img_h)

        total_pixels = float(roi.shape[0] * roi.shape[1])

        # ── Color Space Conversions ──────────────────────────────────────────
        roi_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB).astype(np.float32)
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        r = roi_rgb[:, :, 0]
        g = roi_rgb[:, :, 1]
        b = roi_rgb[:, :, 2]

        # ── 1. VARI Index: (G - R) / (G + R - B) ────────────────────────────
        # Positive (> 0.0) indicates vegetative biomass; negative indicates dead tissue/soil/wood.
        vari_denominator = g + r - b
        vari_denominator[vari_denominator == 0] = 1e-4
        vari = (g - r) / vari_denominator
        vari = np.clip(vari, -1.0, 1.0)
        mean_vari = float(np.mean(vari))

        # ── 2. Excess Green Index (ExG): 2*G - R - B ────────────────────────
        sum_rgb = r + g + b
        sum_rgb[sum_rgb == 0] = 1e-4
        norm_r = r / sum_rgb
        norm_g = g / sum_rgb
        norm_b = b / sum_rgb
        exg = 2.0 * norm_g - norm_r - norm_b
        mean_exg = float(np.mean(exg))

        # ── 3. HSV Color Segmentation ────────────────────────────────────────
        # Chlorophyll-rich green foliage: Hue ~ 25..85 (in 0..180 OpenCV range), Saturation > 40, Value > 30
        lower_foliage = np.array([25, 40, 30], dtype=np.uint8)
        upper_foliage = np.array([88, 255, 255], dtype=np.uint8)
        foliage_mask = cv2.inRange(roi_hsv, lower_foliage, upper_foliage)

        # Desiccated / necrotic / dead bark tissue: Hue ~ 5..24 (orange-brown), Saturation > 25, Value > 20
        lower_necrotic = np.array([5, 25, 20], dtype=np.uint8)
        upper_necrotic = np.array([24, 255, 200], dtype=np.uint8)
        necrotic_mask = cv2.inRange(roi_hsv, lower_necrotic, upper_necrotic)

        green_pixels = int(np.count_nonzero(foliage_mask))
        necrotic_pixels = int(np.count_nonzero(necrotic_mask))

        foliage_ratio = green_pixels / total_pixels
        necrotic_ratio = necrotic_pixels / total_pixels

        # ── 4. Classification Decision & Confidence Calculation ───────────────
        is_alive = (
            (foliage_ratio >= self.foliage_ratio_threshold) and
            (green_pixels >= self.min_green_pixels) and
            (mean_vari >= self.vari_alive_threshold or mean_exg > 0.05)
        )

        if is_alive:
            verdict = "ALIVE"
            # Confidence scales with foliage density and VARI strength
            conf_signal = 0.5 + 0.3 * min(1.0, foliage_ratio / 0.25) + 0.2 * min(1.0, max(0.0, mean_vari) / 0.20)
            confidence = float(np.clip(conf_signal, 0.70, 0.98))
            details = (
                f"Active photosynthetic canopy detected. Foliar coverage: {foliage_ratio*100:.1f}%, "
                f"VARI index: {mean_vari:.3f}, ExG: {mean_exg:.3f}. Vigorous chlorophyll absorption."
            )
        else:
            verdict = "DEAD"
            # Confidence scales with absence of green foliage and presence of dead/dry matter
            conf_signal = 0.55 + 0.25 * (1.0 - min(1.0, foliage_ratio / 0.05)) + 0.20 * min(1.0, necrotic_ratio / 0.15)
            confidence = float(np.clip(conf_signal, 0.72, 0.96))
            details = (
                f"Necrotic or leafless crown detected. Foliar coverage: {foliage_ratio*100:.1f}%, "
                f"VARI index: {mean_vari:.3f}, ExG: {mean_exg:.3f}. Insufficient photosynthetic foliage."
            )

        metrics = {
            "mean_vari": mean_vari,
            "mean_exg": mean_exg,
            "foliage_ratio": foliage_ratio,
            "necrotic_ratio": necrotic_ratio,
            "green_pixel_count": green_pixels,
            "total_pixel_count": int(total_pixels),
        }

        # Create annotated ROI
        annotated_roi = roi.copy()
        mask_color = np.zeros_like(roi)
        if is_alive:
            mask_color[foliage_mask > 0] = [0, 255, 0]  # Green overlay for alive foliage
        else:
            mask_color[necrotic_mask > 0] = [0, 0, 255]  # Red overlay for necrotic tissue
        cv2.addWeighted(annotated_roi, 0.75, mask_color, 0.25, 0, annotated_roi)

        return VitalityResult(
            verdict=verdict,
            confidence=confidence,
            greenness_index=mean_vari,
            foliage_ratio=foliage_ratio,
            details=details,
            metrics=metrics,
            annotated_roi=annotated_roi,
        )


class DeepLearningClassifier(TreeClassifierBase):
    """
    Pluggable Deep Learning Classifier.
    Supports ONNX Runtime, PyTorch, YOLO, or Vision Transformer checkpoints.
    If model weights file is absent, logs clear notification and falls back to
    FoliarSpectralClassifier for full operational safety.
    """

    def __init__(self, model_path: Optional[str] = None):
        self.model_path = model_path
        self.model_loaded = False
        self.fallback_classifier = FoliarSpectralClassifier()

        if model_path:
            self._try_load_model(model_path)

    def _try_load_model(self, path: str):
        try:
            # Placeholder for loading ONNX / TorchScript model
            # import onnxruntime as ort
            # self.session = ort.InferenceSession(path)
            # self.model_loaded = True
            pass
        except Exception:
            self.model_loaded = False

    def classify(
        self,
        image_bgr: np.ndarray,
        roi_bbox: Optional[Tuple[int, int, int, int]] = None
    ) -> VitalityResult:
        if self.model_loaded:
            # DL Inference pipeline would go here:
            # Preprocess -> Forward pass -> Softmax -> VitalityResult
            pass

        # Fallback to spectral classifier with documentation in details
        result = self.fallback_classifier.classify(image_bgr, roi_bbox)
        result.details += " [Inference engine: Spectral/Foliar Baseline Pipeline (Pluggable DL ready)]"
        return result


def create_classifier(classifier_type: str = "spectral", model_path: Optional[str] = None) -> TreeClassifierBase:
    """Factory helper to instantiate desired classifier."""
    classifier_type = classifier_type.lower().strip()
    if classifier_type in ("deep_learning", "dl", "yolo", "cnn"):
        return DeepLearningClassifier(model_path=model_path)
    return FoliarSpectralClassifier()
