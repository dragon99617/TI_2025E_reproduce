import cv2
import numpy as np


class CircleGet:
    def __init__(self):
        self.a4_width = 265
        self.a4_height = 203
        self.circle_radius = 60
        self.M = None
        self.physical_circle_points = self._generate_physical_circle_points()

    def _generate_physical_circle_points(self, divisions=64):
        points = []
        center_x = self.a4_width / 2
        center_y = self.a4_height / 2
        for i in range(divisions):
            angle = 2 * np.pi * i / divisions
            x = center_x + self.circle_radius * np.cos(angle)
            y = center_y + self.circle_radius * np.sin(angle)
            points.append([x, y])
        return np.float32(points)

    def calculate_perspective_matrix(self, detected_corners):
        physical_corners = np.float32(
            [
                [0, 0],
                [self.a4_width, 0],
                [self.a4_width, self.a4_height],
                [0, self.a4_height],
            ]
        )
        image_corners = np.float32(detected_corners)
        self.M = cv2.getPerspectiveTransform(physical_corners, image_corners)
        return self.M

    def transform_circle_points(self):
        if self.M is None:
            raise ValueError("calculate_perspective_matrix must be called first")

        points_homogeneous = np.hstack(
            [
                self.physical_circle_points,
                np.ones((len(self.physical_circle_points), 1), dtype=np.float32),
            ]
        )
        transformed_points = np.dot(self.M, points_homogeneous.T).T
        transformed_points = transformed_points[:, :2] / transformed_points[:, 2:]
        return transformed_points.astype(np.int32)

    def pts_ordered(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        sums = pts.sum(axis=1)
        rect[3] = pts[np.argmax(sums)]
        rect[1] = pts[np.argmin(sums)]

        diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(diff)]
        rect[2] = pts[np.argmax(diff)]

        if rect[0][1] > rect[1][1] and rect[2][1] > rect[3][1]:
            rect[0][1], rect[1][1] = rect[1][1], rect[0][1]
            rect[2][1], rect[3][1] = rect[3][1], rect[2][1]
        return rect

    def forward(self, center, pts):
        pts = pts.reshape(4, 2).astype(int)
        pts = self.pts_ordered(pts)
        self.calculate_perspective_matrix(pts)
        circle_points = self.transform_circle_points()
        center_circle = np.mean(circle_points, axis=0).astype(int)
        offset = np.array(center) - center_circle
        return circle_points + offset
