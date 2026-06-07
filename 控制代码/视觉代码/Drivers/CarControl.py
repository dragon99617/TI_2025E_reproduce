from maix import err, pinmap, uart

from config import UART_BAUD, UART_DEVICE, UART_RX_FUNC, UART_RX_PIN, UART_TX_FUNC, UART_TX_PIN

SERIAL_ID = UART_DEVICE


def open_serial():
    err.check_raise(
        pinmap.set_pin_function(UART_TX_PIN, UART_TX_FUNC),
        "set UART TX pin failed",
    )
    err.check_raise(
        pinmap.set_pin_function(UART_RX_PIN, UART_RX_FUNC),
        "set UART RX pin failed",
    )
    return uart.UART(UART_DEVICE, UART_BAUD)


class CarControl:
    def __init__(self, serial_dev=None):
        self.ser = serial_dev or open_serial()

    def _send(self, text):
        self.ser.write_str(text)

    def send_angle(self, int1, int2):
        self._send("{},{},{},{}\n".format(int(int1), int(int2), 0, 0))

    def send_none(self):
        self._send("{},{},{},{}\n".format(404, 404, 0, 0))

    def send_speed(self, int1, int2):
        self._send("{},{},{},{}\n".format(333, 333, int(int1), int(int2)))

    def send_off(self):
        self._send("{},{},{},{}\n".format(666, 666, 0, 0))
