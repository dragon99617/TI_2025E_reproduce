from maix import gpio

from config import LASER_ACTIVE_HIGH, LASER_GPIO, LASER_PIN
from Drivers.board_gpio import setup_gpio


class Laser:
    def __init__(self, pin_name=LASER_PIN, gpio_name=LASER_GPIO, active_high=LASER_ACTIVE_HIGH):
        self.active_high = active_high
        self.io = setup_gpio(pin_name, gpio_name, gpio.Mode.OUT)
        self.off()

    def on(self):
        self.io.value(1 if self.active_high else 0)

    def off(self):
        self.io.value(0 if self.active_high else 1)

    def pulse(self, seconds):
        import time

        self.on()
        time.sleep(seconds)
        self.off()
