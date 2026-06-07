# Maxicam Vision for MaixCAM

This folder is a standalone MaixVision project. Open this folder or `main.py`
in MaixVision, upload it to MaixCAM, and run `main.py`.

## Defaults

- UART: `/dev/ttyS0`, `115200`
- UART pins: `A16` TX, `A17` RX
- Laser pin: `A18` as `GPIOA18`, active high
- Button pin: `A19` as `GPIOA19`, active low
- Camera resolution: tries `1920x1080`, then `1280x720`, `640x360`, `640x480`
- Camera rotation: disabled
- Algorithm: original traditional OpenCV rectangle/circle tracking

All hardware choices are in `config.py`.

## Button

- Short press: select next task
- Long press while idle: run selected task
- Long press while running: stop current task

## UART task commands

The same UART is reserved for task control input. Send ASCII text ending with
`\n` or `\r`.

- `1`: run task 1
- `2`: run task 2
- `3l`: run task 3 left
- `3r`: run task 3 right
- `4`: run task 4
- `5`: run task 5
- `6`: run task 6

Tracking output keeps the original lower-board protocol:

- angle: `dx,dy,0,0\n`
- speed search: `333,333,vx,vy\n`
- off: `666,666,0,0\n`

## Calibration

The original laser base point was `(285, 192)` at `640x360`. This project
scales it automatically for the active camera resolution. After mounting the
camera and laser, update `BASE_POINT_REF` in `config.py` if the hit point is
offset.

If MaixVision preview or OpenCV becomes too slow at `1920x1080`, move
`1280x720` to the first item in `CAMERA_RESOLUTIONS`.

## Wiring notes

MaixCAM IO is 3.3 V. Use a transistor, optocoupler, relay, or level circuit
for the laser driver and any 5 V lower board signal.
