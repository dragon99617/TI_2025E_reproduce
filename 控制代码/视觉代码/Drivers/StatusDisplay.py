from maix import display, image

from config import TASK_NAMES


class StatusDisplay:
    def __init__(self):
        self.disp = display.Display()
        self.white = image.Color.from_rgb(255, 255, 255)
        self.red = image.Color.from_rgb(255, 0, 0)
        self.green = image.Color.from_rgb(0, 255, 0)
        self.black = image.Color.from_rgb(0, 0, 0)

    def _draw_text(self, img, x, y, text, color, scale=1):
        try:
            img.draw_string(x, y, text, color=color, scale=scale)
        except TypeError:
            img.draw_string(x, y, text, color=color)

    def _image_width(self, img):
        width = getattr(img, "width", None)
        if callable(width):
            return width()
        return width

    def _overlay_scale(self, img):
        width = self._image_width(img)
        if not width:
            return 3
        return max(3, min(8, int(round(width / 320))))

    def show_status(self, task_id, message="READY"):
        w = self.disp.width()
        h = self.disp.height()
        img = image.Image(w, h, image.Format.FMT_RGB888)
        img.draw_rect(0, 0, w, h, color=self.black, thickness=-1)
        self._draw_text(img, 12, 12, "MAXICAM VISION", self.green, scale=2)
        self._draw_text(img, 12, 58, "TASK {} {}".format(task_id, TASK_NAMES.get(task_id, "")), self.white, scale=2)
        self._draw_text(img, 12, 104, message, self.red, scale=2)
        self._draw_text(img, 12, 160, "Short: next", self.white, scale=2)
        self._draw_text(img, 12, 204, "Long: run/stop", self.white, scale=2)
        self._draw_text(img, 12, 252, "UART: 1 2 3l 3r", self.white, scale=2)
        self._draw_text(img, 12, 296, "4 5 6", self.white, scale=2)
        self.disp.show(img)

    def show_frame(self, maix_img, task_id, message=None, center=None):
        if maix_img is None:
            self.show_status(task_id, message or "NO IMAGE")
            return
        scale = self._overlay_scale(maix_img)
        line_gap = 18 * scale
        self._draw_text(maix_img, 8, 8, "TASK {}".format(task_id), self.red, scale=scale)
        if message:
            self._draw_text(maix_img, 8, 8 + line_gap, message, self.red, scale=scale)
        if center is not None:
            x, y = int(center[0]), int(center[1])
            marker = max(16, 6 * scale)
            maix_img.draw_rect(
                x - marker // 2,
                y - marker // 2,
                marker,
                marker,
                color=self.green,
                thickness=max(2, scale // 2),
            )
        self.disp.show(maix_img)
