import time

from Algorithm.CenterGet import CenterGet
from Algorithm.CircleGet import CircleGet
from config import BASE_TIME


def task(camera, car, laser, ctx):
    start_time = time.time()
    laser.on()
    circle = CircleGet()

    while not ctx.should_stop():
        frame, maix_img = camera.capture(return_maix=True)
        if frame is None:
            continue

        results = CenterGet(frame, return_pts=True)
        if results is None:
            ctx.show_frame(maix_img, "SEARCH CIRCLE")
            continue

        center, pts = results
        circle_points = circle.forward(center, pts)
        if len(circle_points) == 0:
            continue

        current_step = int((time.time() - start_time) / BASE_TIME * len(circle_points))
        target = circle_points[current_step % len(circle_points)]
        dx = ctx.base_point[0] - target[0]
        dy = ctx.base_point[1] - target[1]
        car.send_angle(dx, dy)
        print(dx, dy)
        ctx.show_frame(maix_img, "CIRCLE {},{}".format(int(dx), int(dy)), target)

    laser.off()
    car.send_angle(0, 0)
