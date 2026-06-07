import time

from Tasks.common import fire_once, get_center_frame, send_tracking


def task(camera, car, laser, ctx):
    start_time = time.time()
    laser.off()
    stable_count = 0
    laser_flag = False

    while not ctx.should_stop():
        _, maix_img, center = get_center_frame(camera)
        if center is not None:
            dx, dy = send_tracking(car, ctx.base_point, center)
            ctx.show_frame(maix_img, "LOCK {},{}".format(dx, dy), center)
            if abs(dx) < ctx.tolerance and abs(dy) < ctx.tolerance:
                stable_count += 1
                if stable_count > 11 and not laser_flag:
                    fire_once(laser, 0.001)
                    laser_flag = True
                    print("Task3R hit: {:.4f}".format(time.time() - start_time))
            else:
                stable_count = 0
        else:
            car.send_speed(-900, 0)
            ctx.show_frame(maix_img, "SEARCH RIGHT")

        elapsed = time.time() - start_time
        if not laser_flag and elapsed > 3.8:
            fire_once(laser, 0.01)
            laser_flag = True
        if elapsed > 5.0:
            laser.off()
            break
