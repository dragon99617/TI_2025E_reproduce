import time

from maix import app

from config import TASK_NAMES, scaled_base_point, scaled_center_tolerance
from Drivers.Button import Button
from Drivers.CarControl import CarControl, open_serial
from Drivers.Laser import Laser
from Drivers.SerialTaskControl import SerialTaskControl
from Drivers.StatusDisplay import StatusDisplay
from Drivers.camera import Camera
from Tasks.task1 import task as task1
from Tasks.task2 import task as task2
from Tasks.task3l import task as task3l
from Tasks.task3r import task as task3r
from Tasks.task4 import task as task4
from Tasks.task5 import task as task5
from Tasks.task6 import task as task6


IDLE_PREVIEW_INTERVAL = 0.08


TASK_RUNNERS = {
    1: task1,
    2: task2,
    "3l": task3l,
    "3r": task3r,
    4: task4,
    5: task5,
    6: task6,
}
TASK_ORDER = (0, 1, 2, "3l", "3r", 4, 5, 6)


class RuntimeContext:
    def __init__(self, ui, button, serial_control, task_id, base_point, tolerance):
        self.ui = ui
        self.button = button
        self.serial_control = serial_control
        self.task_id = task_id
        self.base_point = base_point
        self.tolerance = tolerance
        self.request = None
        self._last_show = 0

    def _poll_command(self):
        button_event = self.button.poll()
        if button_event == "longPress":
            return ("stop", None)

        serial_event = self.serial_control.poll()
        if serial_event:
            return serial_event
        return None

    def should_stop(self):
        if app.need_exit():
            return True

        command = self._poll_command()
        if command is None:
            return False

        self.request = command
        return True

    def show_frame(self, maix_img, message="", center=None):
        now = time.time()
        if now - self._last_show < 0.05:
            return
        self._last_show = now
        self.ui.show_frame(maix_img, self.task_id, message, center)


def normalize_task_id(task_id):
    if task_id is None:
        return None
    if task_id in TASK_RUNNERS:
        return task_id
    return None


def apply_command(command, current_task):
    if command is None:
        return current_task, False

    action, value = command
    if action == "next":
        try:
            index = TASK_ORDER.index(current_task)
        except ValueError:
            index = 0
        return TASK_ORDER[(index + 1) % len(TASK_ORDER)], False
    if action == "select":
        task_id = normalize_task_id(value)
        return (task_id if task_id is not None else current_task), False
    if action == "run":
        task_id = normalize_task_id(value)
        if task_id is not None:
            current_task = task_id
        return current_task, True
    if action == "stop":
        return current_task, False
    return current_task, False


def run_task(task_id, camera, car, laser, ui, button, serial_control):
    runner = TASK_RUNNERS.get(task_id)
    if runner is None:
        ui.show_status(task_id, "NO TASK SELECTED")
        return None

    base_point = scaled_base_point(camera.width, camera.height)
    tolerance = scaled_center_tolerance(camera.width, camera.height)
    ctx = RuntimeContext(ui, button, serial_control, task_id, base_point, tolerance)

    ui.show_status(task_id, "RUNNING {}".format(TASK_NAMES.get(task_id, "")))
    try:
        runner(camera, car, laser, ctx)
    finally:
        laser.off()

    return ctx.request


def main():
    ui = StatusDisplay()
    ui.show_status(0, "BOOTING")

    serial_dev = open_serial()
    car = CarControl(serial_dev)
    serial_control = SerialTaskControl(serial_dev)
    laser = Laser()
    button = Button()

    camera = Camera()
    if not camera.open():
        ui.show_status(0, "CAMERA OPEN FAILED")
        while not app.need_exit():
            time.sleep(0.2)
        return

    current_task = 0
    last_preview_time = 0
    ui.show_frame(camera.read_maix(), current_task, "READY {}x{}".format(camera.width, camera.height))

    while not app.need_exit():
        command = None

        button_event = button.poll()
        if button_event == "shortPress":
            command = ("next", None)
        elif button_event == "longPress":
            command = ("run", None)

        serial_event = serial_control.poll()
        if serial_event:
            command = serial_event

        current_task, should_run = apply_command(command, current_task)
        if command:
            ui.show_frame(camera.read_maix(), current_task, "READY {}x{}".format(camera.width, camera.height))

        if should_run and current_task != 0:
            next_command = run_task(current_task, camera, car, laser, ui, button, serial_control)
            current_task, should_run_again = apply_command(next_command, current_task)
            ui.show_frame(camera.read_maix(), current_task, "READY {}x{}".format(camera.width, camera.height))
            if should_run_again and current_task != 0:
                next_command = run_task(current_task, camera, car, laser, ui, button, serial_control)
                current_task, _ = apply_command(next_command, current_task)
                ui.show_frame(camera.read_maix(), current_task, "READY {}x{}".format(camera.width, camera.height))

        now = time.time()
        if not should_run and now - last_preview_time > IDLE_PREVIEW_INTERVAL:
            last_preview_time = now
            ui.show_frame(camera.read_maix(), current_task, "READY {}x{}".format(camera.width, camera.height))

        time.sleep(0.05)

    laser.off()
    car.send_off()
    camera.close()


if __name__ == "__main__":
    main()
