import time

from maix import gpio

from config import BUTTON_ACTIVE_LOW, BUTTON_GPIO, BUTTON_PIN
from Drivers.board_gpio import setup_gpio


class Button:
    def __init__(
        self,
        pin_name=BUTTON_PIN,
        gpio_name=BUTTON_GPIO,
        active_low=BUTTON_ACTIVE_LOW,
        short_ticks=2,
        long_ticks=10,
    ):
        self.active_low = active_low
        self.io = setup_gpio(pin_name, gpio_name, gpio.Mode.IN)
        self.short_ticks = short_ticks
        self.long_ticks = long_ticks
        self.down_ticks = 0

    def _is_pressed(self):
        value = self.io.value()
        return value == 0 if self.active_low else value != 0

    def poll(self):
        if self._is_pressed():
            self.down_ticks += 1
            return None

        if self.down_ticks == 0:
            return None

        ticks = self.down_ticks
        self.down_ticks = 0
        if ticks >= self.long_ticks:
            return "longPress"
        if ticks >= self.short_ticks:
            return "shortPress"
        return None

    def wait_tick(self, seconds=0.05):
        time.sleep(seconds)
        return self.poll()
