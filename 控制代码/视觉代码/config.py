UART_DEVICE = "/dev/ttyS0"
UART_BAUD = 115200
UART_TX_PIN = "A16"
UART_TX_FUNC = "UART0_TX"
UART_RX_PIN = "A17"
UART_RX_FUNC = "UART0_RX"

# Default external pins. Change only these four values if your wiring changes.
LASER_PIN = "A18"
LASER_GPIO = "GPIOA18"
LASER_ACTIVE_HIGH = True

BUTTON_PIN = "A19"
BUTTON_GPIO = "GPIOA19"
BUTTON_ACTIVE_LOW = True

# Highest practical default for MaixVision + traditional OpenCV processing.
# The camera driver tries these in order and falls back if the board rejects one.
CAMERA_RESOLUTIONS = (
    (1920, 1080),
    (1280, 720),
    (640, 360),
    (640, 480),
)
CAMERA_FORMAT = "FMT_BGR888"
ROTATE_180 = False
SKIP_FRAMES = 10

# Original project calibration was measured at 640x360.
BASE_POINT_REF = (285, 192)
BASE_POINT_REF_SIZE = (640, 360)
BASE_TIME = 18.0

CENTER_THRESHOLD = 144
CANNY_LOW_THRESHOLD = 50
CANNY_HIGH_THRESHOLD = 150
MIN_RECT_AREA_AT_640 = 500

TASK_NAMES = {
    0: "IDLE",
    1: "TASK1",
    2: "TASK2",
    "3l": "TASK3L",
    "3r": "TASK3R",
    4: "TASK4",
    5: "TASK5",
    6: "TASK6",
}


def scaled_base_point(width, height):
    ref_w, ref_h = BASE_POINT_REF_SIZE
    return (
        int(round(BASE_POINT_REF[0] * width / ref_w)),
        int(round(BASE_POINT_REF[1] * height / ref_h)),
    )


def scaled_center_tolerance(width, height):
    ref_w, ref_h = BASE_POINT_REF_SIZE
    scale = min(width / ref_w, height / ref_h)
    return max(5, int(round(5 * scale)))


def scaled_min_rect_area(width, height):
    ref_w, ref_h = BASE_POINT_REF_SIZE
    scale = (width * height) / (ref_w * ref_h)
    return max(MIN_RECT_AREA_AT_640, int(round(MIN_RECT_AREA_AT_640 * scale)))
