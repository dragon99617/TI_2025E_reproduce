import cv2 as cv

from maix import camera as maix_camera
from maix import image as maix_image

from config import CAMERA_FORMAT, CAMERA_RESOLUTIONS, ROTATE_180, SKIP_FRAMES


class Camera:
    def __init__(self):
        self.cam = None
        self.is_opened = False
        self.width = 0
        self.height = 0
        self.format = getattr(maix_image.Format, CAMERA_FORMAT)

    def open(self, main_size=None):
        if self.is_opened:
            return True

        candidates = []
        if main_size:
            candidates.append(main_size)
        candidates.extend(CAMERA_RESOLUTIONS)

        seen = set()
        for width, height in candidates:
            if (width, height) in seen:
                continue
            seen.add((width, height))
            try:
                self.cam = maix_camera.Camera(width, height, self.format)
                if SKIP_FRAMES and hasattr(self.cam, "skip_frames"):
                    self.cam.skip_frames(SKIP_FRAMES)
                self.width = width
                self.height = height
                self.is_opened = True
                print("Camera opened: {}x{}".format(width, height))
                return True
            except Exception as exc:
                print("Camera open failed at {}x{}: {}".format(width, height, exc))

        self.is_opened = False
        return False

    def read_maix(self):
        if not self.is_opened:
            return None
        return self.cam.read()

    def capture(self, resize=None, return_maix=False):
        img = self.read_maix()
        if img is None:
            return (None, None) if return_maix else None

        frame = maix_image.image2cv(img, ensure_bgr=False, copy=False)
        if ROTATE_180:
            frame = cv.rotate(frame, cv.ROTATE_180)
        if resize and isinstance(resize, tuple) and len(resize) == 2:
            frame = cv.resize(frame, resize)

        if return_maix:
            return frame, img
        return frame

    def close(self):
        self.cam = None
        self.is_opened = False

    def __del__(self):
        self.close()
