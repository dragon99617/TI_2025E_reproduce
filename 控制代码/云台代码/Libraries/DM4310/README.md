# DM4310 FDCAN Driver

This driver adapts the DAMIAO DM4310 official CAN protocol to STM32H7 FDCAN.
It uses classic CAN frames through the H7 FDCAN peripheral.

## Basic Use

```c
#include "dm4310_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

static DM4310_Motor yaw;
static DM4310_Motor pitch;
static DM4310_Motor *motors[] = { &yaw, &pitch };

void gimbal_motor_init(void)
{
    DM4310_InitMotor(&yaw, 1, DM4310_MODE_MIT);
    DM4310_InitMotor(&pitch, 2, DM4310_MODE_MIT);

    DM4310_FdcanConfigAllPass(&hfdcan1);
    DM4310_FdcanStartWithRxFifo0(&hfdcan1);

    DM4310_Enable(&hfdcan1, &yaw);
    DM4310_Enable(&hfdcan1, &pitch);
}

void gimbal_send_speed(float yaw_rpm, float pitch_rpm)
{
    DM4310_SetCommand(&yaw, 0.0f, DM4310_RpmToRadPerSec(yaw_rpm), 0.0f, 2.0f, 0.0f);
    DM4310_SetCommand(&pitch, 0.0f, DM4310_RpmToRadPerSec(pitch_rpm), 0.0f, 2.0f, 0.0f);

    DM4310_CopyCommandToControl(&yaw);
    DM4310_CopyCommandToControl(&pitch);

    DM4310_SendControl(&hfdcan1, &yaw);
    DM4310_SendControl(&hfdcan1, &pitch);
}
```

## RX Callback Sketch

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t it)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((it & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
            DM4310_DispatchFeedbackPtr(motors, 2, data, 8);
        }
    }
}
```

Keep the motor array global/static so parsed feedback writes back to the
objects used by the controller.
