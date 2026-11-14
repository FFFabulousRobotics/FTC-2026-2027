"""Limelight 3A Python SnapScript for red, yellow and blue objects."""

import cv2
import numpy as np


# Fraction of the image to ignore at the top. Try 0.36 for the field screenshot.
TOP_CROP = 0.0

# OpenCV HSV uses hue 0..179 and saturation/value 0..255.
# Area thresholds are in pixels at 640x480 and scale with frame size.
COLORS = (
    ("RED", (70, 85, 255), 85, 0.38, (((0, 135, 35), (9, 255, 255)),
                               ((170, 135, 35), (179, 255, 255)))),
    ("YELLOW", (0, 225, 255), 85, 0.38, (((21, 185, 48), (33, 255, 255)),)),
    ("BLUE", (255, 170, 65), 95, 0.28, (((98, 130, 27), (132, 255, 255)),)),
)
NO_TARGET = np.array([[]])
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


def runPipeline(image, llrobot):
    """Draw all accepted color regions and return the largest as the target."""
    height, width = image.shape[:2]
    top = int(height * TOP_CROP)
    area_scale = width * height / (640 * 480)
    hsv = cv2.cvtColor(image[top:], cv2.COLOR_BGR2HSV)
    counts = [0, 0, 0]
    target = NO_TARGET
    best_area = 0.0
    primary = [0, 0, 0, 0, 0]

    for color_id, (name, color, minimum_area, minimum_fill, ranges) in enumerate(COLORS, 1):
        low, high = ranges[0]
        mask = cv2.inRange(hsv, low, high)
        for low, high in ranges[1:]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))
        if min(width, height) >= 400:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < minimum_area * area_scale:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if max(w, h) > 3 * min(w, h) or area < minimum_fill * w * h:
                continue

            y += top
            counts[color_id - 1] += 1
            cv2.rectangle(image, (x, y), (x + w - 1, y + h - 1), color, 2)
            cv2.putText(image, name, (x, max(14, y - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            if area > best_area:
                best_area = area
                target = contour + np.array([[[0, top]]], dtype=contour.dtype)
                primary = [color_id, x + w / 2, y + h / 2, w, h]

    # getPythonOutputs(): red/yellow/blue counts, primary ID (1/2/3),
    # followed by primary center X/Y and box width/height in image pixels.
    return target, image, counts + primary
