"""OpenCV helpers for robust contour centroids and RGB-D depth sampling."""

import cv2
import numpy as np


def contour_centroid(contour):
    """Return the contour centroid, falling back to the bounding-box center."""
    moments = cv2.moments(contour)
    if abs(moments["m00"]) > 1e-9:
        return (
            int(round(moments["m10"] / moments["m00"])),
            int(round(moments["m01"] / moments["m00"])),
        )
    x, y, width, height = cv2.boundingRect(contour)
    return x + width // 2, y + height // 2


def centroid_in_normalized_roi(cx, cy, image_width, image_height, roi):
    """Return whether a centroid lies inside a normalized image ROI."""
    if image_width <= 0 or image_height <= 0:
        return False
    x_min, y_min, x_max, y_max = (float(value) for value in roi)
    if not (0.0 <= x_min < x_max <= 1.0 and 0.0 <= y_min < y_max <= 1.0):
        raise ValueError("normalized ROI must satisfy 0 <= min < max <= 1")
    x_ratio = float(cx) / float(image_width)
    y_ratio = float(cy) / float(image_height)
    return x_min <= x_ratio <= x_max and y_min <= y_ratio <= y_max


def contour_depth_median(depth, contour, erosion_pixels=1):
    """Return depth in metres from valid pixels inside an eroded contour ROI."""
    x, y, width, height = cv2.boundingRect(contour)
    if width <= 0 or height <= 0:
        return None, 0

    contour_mask = np.zeros((height, width), dtype=np.uint8)
    shifted_contour = contour - np.array([[[x, y]]], dtype=contour.dtype)
    cv2.drawContours(contour_mask, [shifted_contour], -1, 255, thickness=-1)
    if erosion_pixels > 0:
        kernel_size = erosion_pixels * 2 + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        eroded = cv2.erode(contour_mask, kernel, iterations=1)
        if cv2.countNonZero(eroded) >= 5:
            contour_mask = eroded

    depth_roi = np.asarray(depth[y : y + height, x : x + width])
    valid = depth_roi[(contour_mask > 0) & (depth_roi > 0) & np.isfinite(depth_roi)]
    if valid.size == 0:
        return None, 0

    depth_m = float(np.median(valid))
    if depth_m > 100.0:
        depth_m /= 1000.0
    if depth_m <= 0.0 or depth_m > 10.0:
        return None, int(valid.size)
    return depth_m, int(valid.size)
