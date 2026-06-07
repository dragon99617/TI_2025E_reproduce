import time

from Algorithm.CenterGet import CenterGet


def get_center_frame(camera, return_pts=False):
    frame, maix_img = camera.capture(return_maix=True)
    if frame is None:
        return None, None, None
    center = CenterGet(frame, return_pts=return_pts)
    return frame, maix_img, center


def send_tracking(car, base_point, center):
    dx = base_point[0] - center[0]
    dy = base_point[1] - center[1]
    car.send_angle(dx, dy)
    print(dx, dy)
    return dx, dy


def fire_once(laser, seconds):
    laser.on()
    time.sleep(seconds)
    laser.off()
