import time


def task(camera, car, laser, ctx):
    laser.off()
    start = time.time()
    while not ctx.should_stop():
        _, maix_img = camera.capture(return_maix=True)
        ctx.show_frame(maix_img, "TASK1 PREVIEW")
        if time.time() - start > 3:
            break
