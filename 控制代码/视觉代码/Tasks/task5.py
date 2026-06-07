from Tasks.common import get_center_frame, send_tracking


def task(camera, car, laser, ctx):
    laser.off()
    while not ctx.should_stop():
        _, maix_img, center = get_center_frame(camera)
        if center is not None:
            dx, dy = send_tracking(car, ctx.base_point, center)
            ctx.show_frame(maix_img, "FOLLOW {},{}".format(dx, dy), center)
        else:
            ctx.show_frame(maix_img, "SEARCH")
