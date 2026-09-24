from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import cv2
import numpy as np

from app_config import DEFAULT_CONFIG


def _odd_kernel(value: int, minimum: int = 1) -> int:
    value = max(int(value), minimum)
    if value % 2 == 0:
        value += 1
    return value


def _optional_float(config: dict[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = config.get(key)
        if value is None:
            continue
        try:
            return float(value)
        except (TypeError, ValueError):
            return None
    return None


@dataclass
class DetectionResult:
    found: bool
    targets: list[dict[str, Any]]


class GreenLightDetector:
    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config: dict[str, Any] = {}
        self.debug_mask: np.ndarray | None = None
        self.debug_result: np.ndarray | None = None
        self.last_metrics: dict[str, Any] = {}
        self.set_config(config or DEFAULT_CONFIG.get("detection", {}))

    def set_config(self, config: dict[str, Any]) -> None:
        self.config = config or {}

    def current_hsv_range(self) -> tuple[np.ndarray, np.ndarray]:
        hsv_config = self.config.get("hsv", {})
        lower = np.array(hsv_config.get("lower", [24, 31, 123]), dtype=np.uint8)
        upper = np.array(hsv_config.get("upper", [83, 253, 255]), dtype=np.uint8)
        return lower, upper

    def detect_green_light(self, frame_rgb: np.ndarray) -> DetectionResult:
        lower_hsv, upper_hsv = self.current_hsv_range()
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
        value_channel = hsv[:, :, 2]
        stage_reports = []
        for stage_name, relaxed in (("strict", False), ("relaxed", True)):
            mask = self._create_mask(hsv, value_channel, lower_hsv, upper_hsv, relaxed=relaxed)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidates = []
            for contour in contours:
                candidate = self._evaluate_candidate(
                    contour,
                    hsv,
                    value_channel,
                    frame_rgb.shape[:2],
                    relaxed=relaxed,
                )
                if candidate is not None:
                    candidates.append(candidate)

            targets = self._build_targets(candidates, frame_rgb.shape[:2], relaxed=relaxed)
            stage_reports.append(
                {
                    "stage": stage_name,
                    "raw_contours": len(contours),
                    "candidate_count": len(candidates),
                    "target_count": len(targets),
                }
            )

            if targets:
                self.debug_mask = mask
                self.last_metrics = {
                    "stage": stage_name,
                    "target_count": len(targets),
                    "targets": [target["metrics"] for target in targets],
                }
                self.debug_result = self._build_debug_result(frame_rgb, targets, candidates)
                return DetectionResult(True, targets)

            self.debug_mask = mask
            self.debug_result = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

        self.last_metrics = {
            "reason": "no_valid_candidate",
            "lower_hsv": lower_hsv.tolist(),
            "upper_hsv": upper_hsv.tolist(),
            "stages": stage_reports,
        }
        return DetectionResult(False, [])

    def _create_mask(
        self,
        hsv: np.ndarray,
        value_channel: np.ndarray,
        lower_hsv: np.ndarray,
        upper_hsv: np.ndarray,
        relaxed: bool = False,
    ) -> np.ndarray:
        if relaxed:
            lower = np.array(
                [max(int(lower_hsv[0]) - 8, 0), max(int(lower_hsv[1]) - 40, 0), max(int(lower_hsv[2]) - 60, 0)],
                dtype=np.uint8,
            )
            upper = np.array(
                [min(int(upper_hsv[0]) + 8, 179), min(int(upper_hsv[1]) + 35, 255), 255],
                dtype=np.uint8,
            )
        else:
            lower = lower_hsv
            upper = upper_hsv

        green_mask = cv2.inRange(hsv, lower, upper)
        min_v = int(self.config.get("brightness", {}).get("min_v", 180))
        if relaxed:
            min_v = max(int(min_v * 0.6), 80)
        bright_mask = cv2.threshold(value_channel, min_v, 255, cv2.THRESH_BINARY)[1]
        mask = cv2.bitwise_and(green_mask, bright_mask)
        return self._refine_mask(mask)

    def _build_targets(
        self,
        candidates: list[dict[str, Any]],
        frame_shape: tuple[int, int],
        relaxed: bool = False,
    ) -> list[dict[str, Any]]:
        if not candidates:
            return []

        contour_config = self.config.get("contour", {})
        min_area = float(contour_config.get("min_area", 60))
        max_area = float(contour_config.get("max_area", 20000))
        far_min_area = float(contour_config.get("far_min_area", contour_config.get("min_area", 60)))
        near_min_area = float(contour_config.get("near_min_area", 300))

        near_candidates = []
        far_candidates = []
        for candidate in sorted(candidates, key=lambda item: item["score"], reverse=True):
            area = float(candidate["area"])
            if area < min_area or area > max_area:
                continue
            # Area classification is strict: valid circle first, then far/near split.
            if area > near_min_area:
                near_candidates.append(candidate)
            elif far_min_area < area < near_min_area:
                far_candidates.append(candidate)

        selected_candidates: list[tuple[str, dict[str, Any]]] = []
        if near_candidates:
            selected_candidates.append(("near", near_candidates[0]))
        if far_candidates:
            selected_candidates.append(("far", far_candidates[0]))

        # 宽松阶段兜底：只要有明显候选，就至少输出一个目标。
        _frame_h, frame_w = frame_shape
        targets: list[dict[str, Any]] = []
        for index, (role, candidate) in enumerate(selected_candidates):
            x_pixel = int(candidate["center"][0])
            x_offset_px, angle_deg = self._calculate_horizontal_angle(x_pixel, frame_w)
            target = {
                "contour": candidate["contour"],
                "center": candidate["center"],
                "x_pixel": x_pixel,
                "x_offset": x_offset_px,
                "angle_deg": angle_deg,
                "area": candidate["area"],
                "bbox": candidate["bbox"],
                "circle": candidate["circle"],
                "score": candidate["score"],
                "metrics": candidate["metrics"],
                # Keep the legacy key for callers that still read offset; y is no longer used.
                "offset": (
                    x_offset_px,
                    0.0,
                ),
                "role": role,
                "index": index,
            }
            target["metrics"] = {
                **target["metrics"],
                "x_pixel": x_pixel,
                "x_offset_px": round(x_offset_px, 2),
                "angle_deg": round(angle_deg, 4),
            }
            targets.append(target)
        return targets

    def _calculate_horizontal_angle(self, x_pixel: int, frame_w: int) -> tuple[float, float]:
        angle_config = self.config.get("angle", {})
        calibration = self.config.get("calibration", {})

        fx_px = _optional_float(calibration, "fx", "fx_px")
        cx_px = _optional_float(calibration, "cx", "cx_px")
        calibration_width_px = _optional_float(calibration, "image_width_px", "width_px")
        if fx_px is not None and fx_px > 0.0:
            scale_x = frame_w / calibration_width_px if calibration_width_px and calibration_width_px > 0.0 else 1.0
            fx_px *= scale_x
            center_x = cx_px * scale_x if cx_px is not None else frame_w / 2.0
            x_offset_px = float(x_pixel) - center_x
            angle_deg = math.degrees(math.atan2(x_offset_px, fx_px))
            if bool(angle_config.get("invert_x", False)):
                angle_deg = -angle_deg
            return x_offset_px, angle_deg

        center_x_config = angle_config.get("center_x_px")
        center_x = float(center_x_config) if center_x_config is not None else frame_w / 2.0
        x_offset_px = float(x_pixel) - center_x
        focal_length_mm = max(float(angle_config.get("focal_length_mm", 50.0)), 1e-6)
        pixel_size_um = float(angle_config.get("pixel_size_um", 3.45) or 0.0)
        if pixel_size_um <= 0.0:
            sensor_width_mm = float(angle_config.get("sensor_width_mm", 0.0) or 0.0)
            pixel_size_um = (sensor_width_mm * 1000.0 / max(frame_w, 1)) if sensor_width_mm > 0.0 else 3.45

        x_on_sensor_mm = x_offset_px * pixel_size_um / 1000.0
        angle_deg = math.degrees(math.atan2(x_on_sensor_mm, focal_length_mm))
        if bool(angle_config.get("invert_x", False)):
            angle_deg = -angle_deg
        return x_offset_px, angle_deg

    def _refine_mask(self, mask: np.ndarray) -> np.ndarray:
        morphology = self.config.get("morphology", {})
        blur_kernel = _odd_kernel(morphology.get("blur_kernel", 5))
        open_kernel = max(int(morphology.get("open_kernel", 3)), 1)
        close_kernel = max(int(morphology.get("close_kernel", 5)), 1)
        erode_iterations = max(int(morphology.get("erode_iterations", 0)), 0)
        dilate_iterations = max(int(morphology.get("dilate_iterations", 1)), 0)

        refined = cv2.GaussianBlur(mask, (blur_kernel, blur_kernel), 0)
        open_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_kernel, open_kernel))
        close_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_kernel, close_kernel))
        refined = cv2.morphologyEx(refined, cv2.MORPH_OPEN, open_element)
        refined = cv2.morphologyEx(refined, cv2.MORPH_CLOSE, close_element)
        if erode_iterations:
            refined = cv2.erode(refined, close_element, iterations=erode_iterations)
        if dilate_iterations:
            refined = cv2.dilate(refined, close_element, iterations=dilate_iterations)
        return refined

    @staticmethod
    def _contour_geometry(contour: np.ndarray) -> dict[str, Any] | None:
        area = float(cv2.contourArea(contour))
        if area <= 0.0:
            return None

        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return None

        circularity = float(4.0 * np.pi * area / (perimeter * perimeter))
        aspect_ratio = float(min(w, h) / max(w, h))
        (circle_x, circle_y), radius = cv2.minEnclosingCircle(contour)
        enclosing_area = float(np.pi * radius * radius)
        fill_ratio = float(area / enclosing_area) if enclosing_area > 0.0 else 0.0

        moments = cv2.moments(contour)
        if abs(float(moments["m00"])) > 1e-6:
            center_x = float(moments["m10"] / moments["m00"])
            center_y = float(moments["m01"] / moments["m00"])
        else:
            center_x = float(circle_x)
            center_y = float(circle_y)

        return {
            "area": area,
            "perimeter": perimeter,
            "bbox": (x, y, w, h),
            "circularity": circularity,
            "aspect_ratio": aspect_ratio,
            "circle_center": (float(circle_x), float(circle_y)),
            "center": (center_x, center_y),
            "radius": float(radius),
            "fill_ratio": fill_ratio,
        }

    def _select_core_contour(
        self,
        contour: np.ndarray,
        value_channel: np.ndarray,
        min_v_threshold: float,
        min_area: float,
        relaxed: bool = False,
    ) -> np.ndarray:
        geometry = self._contour_geometry(contour)
        if geometry is None:
            return contour

        x, y, w, h = geometry["bbox"]
        contour_shift = np.array([[[x, y]]], dtype=np.int32)
        local_contour = contour.astype(np.int32, copy=False) - contour_shift

        contour_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(contour_mask, [local_contour], -1, 255, thickness=-1)
        contour_values = value_channel[y : y + h, x : x + w][contour_mask == 255]
        if contour_values.size < 24:
            return contour

        value_roi = value_channel[y : y + h, x : x + w]
        morphology = self.config.get("morphology", {})
        core_kernel = _odd_kernel(morphology.get("core_kernel", 3))
        core_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (core_kernel, core_kernel))

        contour_values_f = contour_values.astype(np.float32, copy=False)
        mean_value = float(np.mean(contour_values_f))
        max_value = float(np.max(contour_values_f))
        threshold_candidates = {
            max(float(min_v_threshold), mean_value + (max_value - mean_value) * (0.28 if relaxed else 0.38)),
            max(float(min_v_threshold), mean_value + (max_value - mean_value) * (0.45 if relaxed else 0.58)),
            max(float(min_v_threshold), float(np.percentile(contour_values_f, 60.0 if relaxed else 72.0))),
            max(float(min_v_threshold), float(np.percentile(contour_values_f, 74.0 if relaxed else 86.0))),
        }

        parent_center = np.array(geometry["center"], dtype=np.float32)
        parent_area = max(float(geometry["area"]), 1.0)
        parent_radius = max(float(geometry["radius"]), 1.0)
        min_core_area = max(float(min_area) * 0.8, parent_area * (0.18 if not relaxed else 0.10))
        min_shape_gain = 0.16 if not relaxed else 0.10
        parent_shape_score = geometry["circularity"] * 2.0 + geometry["fill_ratio"] * 1.6

        best_contour = contour
        best_geometry = geometry
        best_score = -1e9
        for local_threshold in sorted(threshold_candidates):
            core_mask = np.zeros((h, w), dtype=np.uint8)
            core_mask[(contour_mask == 255) & (value_roi >= local_threshold)] = 255
            if core_kernel > 1:
                core_mask = cv2.morphologyEx(core_mask, cv2.MORPH_OPEN, core_element)
                core_mask = cv2.morphologyEx(core_mask, cv2.MORPH_CLOSE, core_element)

            core_contours, _ = cv2.findContours(core_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for core_contour in core_contours:
                full_contour = core_contour.astype(np.int32, copy=False) + contour_shift
                core_geometry = self._contour_geometry(full_contour)
                if core_geometry is None:
                    continue
                if core_geometry["area"] < min_core_area:
                    continue

                center_gap = float(np.linalg.norm(np.array(core_geometry["center"], dtype=np.float32) - parent_center))
                center_gap_ratio = center_gap / parent_radius
                area_ratio = min(float(core_geometry["area"]) / parent_area, 1.0)
                score = (
                    core_geometry["circularity"] * 2.8
                    + core_geometry["fill_ratio"] * 1.8
                    + area_ratio * 0.2
                    - center_gap_ratio * 0.5
                )
                if score > best_score:
                    best_score = score
                    best_contour = full_contour
                    best_geometry = core_geometry

        if best_contour is contour:
            return contour
        best_shape_score = best_geometry["circularity"] * 2.0 + best_geometry["fill_ratio"] * 1.6
        if best_shape_score < parent_shape_score + min_shape_gain:
            return contour
        if best_geometry["circularity"] + 0.01 < geometry["circularity"]:
            return contour
        return best_contour

    def _evaluate_candidate(
        self,
        contour: np.ndarray,
        hsv: np.ndarray,
        value_channel: np.ndarray,
        frame_shape: tuple[int, int],
        relaxed: bool = False,
    ) -> dict[str, Any] | None:
        contour_config = self.config.get("contour", {})
        brightness_config = self.config.get("brightness", {})
        circle_config = self.config.get("circle", {})
        scoring = self.config.get("scoring", {})

        min_area = float(contour_config.get("min_area", 60))
        max_area = float(contour_config.get("max_area", 20000))
        min_circularity = float(contour_config.get("min_circularity", 0.55))
        min_aspect_ratio = float(contour_config.get("min_aspect_ratio", 0.55))
        min_fill_ratio = float(contour_config.get("min_fill_ratio", 0.55))
        min_radius = float(circle_config.get("min_radius", 4))
        max_radius = float(circle_config.get("max_radius", 120))
        min_v_threshold = float(brightness_config.get("min_v", 180))
        max_std_threshold = float(brightness_config.get("max_std", 90))

        if relaxed:
            min_circularity = max(0.35, min_circularity - 0.15)
            min_aspect_ratio = max(0.35, min_aspect_ratio - 0.15)
            min_fill_ratio = max(0.35, min_fill_ratio - 0.15)
            min_radius = max(2.0, min_radius - 2.0)
            max_radius = max_radius * 1.5
            min_v_threshold = max(80.0, min_v_threshold * 0.6)
            max_std_threshold = max_std_threshold * 1.8

        contour = self._select_core_contour(
            contour,
            value_channel,
            min_v_threshold=min_v_threshold,
            min_area=min_area,
            relaxed=relaxed,
        )

        geometry = self._contour_geometry(contour)
        if geometry is None:
            return None

        area = float(geometry["area"])
        if area < min_area:
            return None
        if area > max_area:
            return None

        circularity = float(geometry["circularity"])
        if circularity < min_circularity:
            return None

        x, y, w, h = geometry["bbox"]
        aspect_ratio = float(geometry["aspect_ratio"])
        if aspect_ratio < min_aspect_ratio:
            return None

        circle_x, circle_y = geometry["circle_center"]
        radius = float(geometry["radius"])
        if radius < min_radius or radius > max_radius:
            return None

        fill_ratio = float(geometry["fill_ratio"])
        if fill_ratio < min_fill_ratio:
            return None

        contour_mask = np.zeros(value_channel.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
        contour_values = value_channel[contour_mask == 255]
        contour_hsv = hsv[contour_mask == 255]
        if contour_values.size == 0 or contour_hsv.size == 0:
            return None

        mean_brightness = float(np.mean(contour_values))
        brightness_std = float(np.std(contour_values))
        if mean_brightness < min_v_threshold:
            return None
        if brightness_std > max_std_threshold:
            return None

        mean_h = float(np.mean(contour_hsv[:, 0]))
        mean_s = float(np.mean(contour_hsv[:, 1]))

        _frame_h, frame_w = frame_shape
        center_distance = float(abs(circle_x - frame_w / 2.0))
        max_distance = max(frame_w / 2.0, 1.0)
        center_score = max(0.0, 1.0 - center_distance / max_distance)

        score = (
            circularity * float(scoring.get("circularity_weight", 1.2))
            + fill_ratio * float(scoring.get("fill_ratio_weight", 1.0))
            + (mean_brightness / 255.0) * float(scoring.get("brightness_weight", 1.4))
            + center_score * float(scoring.get("center_weight", 0.35))
        )

        metrics = {
            "area": round(area, 2),
            "circularity": round(circularity, 3),
            "aspect_ratio": round(aspect_ratio, 3),
            "fill_ratio": round(fill_ratio, 3),
            "radius": round(float(radius), 2),
            "mean_brightness": round(mean_brightness, 2),
            "brightness_std": round(brightness_std, 2),
            "mean_h": round(mean_h, 2),
            "mean_s": round(mean_s, 2),
            "center_score": round(center_score, 3),
            "score": round(score, 3),
        }

        return {
            "contour": contour,
            "bbox": (x, y, w, h),
            "center": (int(circle_x), int(circle_y)),
            "circle": (int(circle_x), int(circle_y), int(radius)),
            "area": area,
            "score": score,
            "metrics": metrics,
        }

    def _build_debug_result(
        self,
        frame_rgb: np.ndarray,
        selected_targets: list[dict[str, Any]],
        candidates: list[dict[str, Any]],
    ) -> np.ndarray:
        result = frame_rgb.copy()
        draw_rejected = bool(self.config.get("debug", {}).get("draw_rejected", False))

        if draw_rejected:
            for candidate in candidates:
                x, y, w, h = candidate["bbox"]
                cv2.rectangle(result, (x, y), (x + w, y + h), (255, 165, 0), 1)

        colors = [(0, 255, 0), (255, 165, 0)]
        for target in selected_targets:
            x, y, w, h = target["bbox"]
            center_x, center_y, radius = target["circle"]
            color = colors[min(target["index"], len(colors) - 1)]
            cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
            cv2.drawContours(result, [target["contour"]], -1, color, 2)
            cv2.circle(result, (center_x, center_y), radius, (255, 255, 0), 2)
            cv2.circle(result, (center_x, center_y), 4, (255, 0, 0), -1)
            cv2.putText(
                result,
                "NEAR" if target["role"] == "near" else "FAR",
                (x, max(y - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )
        return result


detector = GreenLightDetector()


def set_detection_config(config: dict[str, Any]) -> None:
    detector.set_config(config)


def get_trackbar_values() -> tuple[np.ndarray, np.ndarray]:
    return detector.current_hsv_range()


def create_trackbars() -> None:
    return None


def detect_green_light_and_offset(
    frame: np.ndarray,
    lower_hsv: np.ndarray | None = None,
    upper_hsv: np.ndarray | None = None,
) -> tuple[bool, tuple[float, float], dict[str, Any] | None]:
    if lower_hsv is not None and upper_hsv is not None:
        detector.set_config(
            {
                **detector.config,
                "hsv": {
                    "lower": lower_hsv.tolist(),
                    "upper": upper_hsv.tolist(),
                },
            }
        )

    result = detector.detect_green_light(frame)
    primary_target = result.targets[0] if result.targets else None
    primary_offset = (float(primary_target.get("x_offset", 0.0)), 0.0) if primary_target is not None else (0.0, 0.0)
    return result.found, primary_offset, primary_target


def detect_green_targets(
    frame: np.ndarray,
    lower_hsv: np.ndarray | None = None,
    upper_hsv: np.ndarray | None = None,
) -> list[dict[str, Any]]:
    if lower_hsv is not None and upper_hsv is not None:
        detector.set_config(
            {
                **detector.config,
                "hsv": {
                    "lower": lower_hsv.tolist(),
                    "upper": upper_hsv.tolist(),
                },
            }
        )
    return detector.detect_green_light(frame).targets


def get_debug_images() -> tuple[np.ndarray | None, np.ndarray | None]:
    return detector.debug_mask, detector.debug_result
