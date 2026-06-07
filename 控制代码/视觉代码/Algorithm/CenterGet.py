import math

import cv2
import numpy as np

from config import CANNY_HIGH_THRESHOLD, CANNY_LOW_THRESHOLD, CENTER_THRESHOLD, scaled_min_rect_area


def preprocess_image(img):
    if img is None:
        return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, CENTER_THRESHOLD, 255, cv2.THRESH_BINARY)
    return thresh


def calculate_equidistant_center(pts):
    pts = np.array(pts, dtype=np.float32)
    if len(pts) != 4:
        return None

    diag1_start, diag1_end = pts[0], pts[2]
    diag2_start, diag2_end = pts[1], pts[3]

    a1 = diag1_end[1] - diag1_start[1]
    b1 = diag1_start[0] - diag1_end[0]
    c1 = diag1_end[0] * diag1_start[1] - diag1_start[0] * diag1_end[1]

    a2 = diag2_end[1] - diag2_start[1]
    b2 = diag2_start[0] - diag2_end[0]
    c2 = diag2_end[0] * diag2_start[1] - diag2_start[0] * diag2_end[1]

    denom = a1 * b2 - a2 * b1
    if denom != 0:
        x = (b1 * c2 - b2 * c1) / denom
        y = (a2 * c1 - a1 * c2) / denom
    else:
        x = np.mean(pts[:, 0])
        y = np.mean(pts[:, 1])

    return (int(round(x)), int(round(y)))


def CenterGet(img, return_pts=False):
    frame = preprocess_image(img)
    if frame is None:
        return None

    edges = cv2.Canny(frame, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    best_score = -1
    best_center = None
    best_approx = None
    h, w = frame.shape[:2]
    min_area = scaled_min_rect_area(w, h)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        perimeter = cv2.arcLength(contour, True)
        epsilon = 0.01 * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) != 4:
            continue

        pts = approx.reshape(4, 2).astype(int)
        border_threshold = 5
        if not all(
            border_threshold < pt[0] < w - border_threshold
            and border_threshold < pt[1] < h - border_threshold
            for pt in pts
        ):
            continue

        angles = []
        for i in range(4):
            p_prev = pts[(i - 1) % 4]
            p_curr = pts[i]
            p_next = pts[(i + 1) % 4]
            vec1 = p_prev - p_curr
            vec2 = p_next - p_curr
            angle = math.degrees(math.atan2(vec2[1], vec2[0]) - math.atan2(vec1[1], vec1[0]))
            angle = abs(angle)
            if angle > 180:
                angle = 360 - angle
            angles.append(angle)

        if not all(70 < angle < 110 for angle in angles):
            continue

        lengths = []
        for i in range(4):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % 4]
            lengths.append(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))

        if min(lengths) <= 0 or max(lengths) / min(lengths) > 5:
            continue

        angle_deviation = sum(abs(angle - 90) for angle in angles) / 4
        angle_score = 100 - angle_deviation
        max_possible_area = (w * h) / 2
        area_score = min(100, (area / max_possible_area) * 100)
        total_score = 0.6 * angle_score + 0.4 * area_score

        if total_score > best_score:
            best_score = total_score
            best_contour = contour
            best_approx = approx
            moments = cv2.moments(contour)
            best_center = calculate_equidistant_center(pts) if moments["m00"] != 0 else None

    if best_contour is not None and best_center is not None:
        if return_pts:
            return best_center, best_approx
        return best_center
    return None
