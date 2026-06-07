from maix import err, gpio, pinmap


def setup_gpio(pin_name, gpio_name, mode):
    err.check_raise(
        pinmap.set_pin_function(pin_name, gpio_name),
        "set pin {} to {} failed".format(pin_name, gpio_name),
    )
    return gpio.GPIO(gpio_name, mode)
