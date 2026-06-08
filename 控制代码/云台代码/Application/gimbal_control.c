#include "gimbal_control.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "dm4310_fdcan.h"
#include "fdcan.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#define YAW_MOTOR_ID                 1u
#define PITCH_MOTOR_ID               2u

#define VISION_RX_BUFFER_SIZE        64u
#define VISION_TIMEOUT_MS            500u

#define YAW_CENTER_RAD               (-0.456f)
#define PITCH_CENTER_RAD             2.31f
#define PITCH_MIN_RAD                2.15f
#define PITCH_MAX_RAD                2.35f

#define VISION_MAX_X                 320
#define VISION_MAX_Y                 240
#define VISION_OFF_CODE              666
#define VISION_NONE_CODE             404
#define VISION_SEARCH_CODE           333

#define VISION_SEARCH_SPEED_SCALE    100.0f
#define VISION_PID_OUTPUT_LIMIT_RPM  20.0f
#define MOTOR_HOME_MS                1000u
#define MOTOR_HOME_PERIOD_MS         5u

#define MIT_HOME_KP                  30.0f
#define MIT_HOME_KD                  1.0f
#define MIT_SPEED_KD                 0.8f

typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float sum_error;
    float last_error;
    float output_min;
    float output_max;
} PID_State;

typedef struct {
    int32_t value[4];
    uint32_t tick_ms;
    bool received;
} Vision_State;

__attribute__((section(".ram_d2"), aligned(32)))
static uint8_t vision_rx_buffer[VISION_RX_BUFFER_SIZE];

static DM4310_Motor yaw_motor;
static DM4310_Motor pitch_motor;
static DM4310_Motor *motor_list[] = { &yaw_motor, &pitch_motor };

static volatile bool pitch_feedback_ready;
static volatile float yaw_speed_rpm;
static volatile float pitch_speed_rpm;
static volatile bool pitch_hold_center;
static volatile bool motor_limp;
static volatile Vision_State vision_state = {
    .value = { VISION_OFF_CODE, VISION_OFF_CODE, 0, 0 },
    .tick_ms = 0,
    .received = false,
};

static PID_State yaw_pid = {
    .kp = -0.4f,
    .ki = 0.0f,
    .kd = 0.0f,
    .target = 0.0f,
    .output_min = -VISION_PID_OUTPUT_LIMIT_RPM,
    .output_max = VISION_PID_OUTPUT_LIMIT_RPM,
};

static PID_State pitch_pid = {
    .kp = -0.4f,
    .ki = 0.0f,
    .kd = 0.0f,
    .target = 0.0f,
    .output_min = -VISION_PID_OUTPUT_LIMIT_RPM,
    .output_max = VISION_PID_OUTPUT_LIMIT_RPM,
};

static float clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static float pid_calc(PID_State *pid, float input)
{
    const float error = pid->target - input;
    float output;

    pid->sum_error += error;
    output = pid->kp * error + pid->ki * pid->sum_error + pid->kd * (error - pid->last_error);
    pid->last_error = error;

    return clampf(output, pid->output_min, pid->output_max);
}

static bool is_pair(int32_t a, int32_t b, int32_t code)
{
    return a == code && b == code;
}

static void vision_set_values(const int32_t values[4])
{
    vision_state.value[0] = values[0];
    vision_state.value[1] = values[1];
    vision_state.value[2] = values[2];
    vision_state.value[3] = values[3];
    vision_state.tick_ms = HAL_GetTick();
    vision_state.received = true;
}

static void vision_parse_line(const uint8_t *data, uint16_t size)
{
    char text[VISION_RX_BUFFER_SIZE + 1u];
    char *cursor;
    int32_t values[4] = { 0, 0, 0, 0 };

    if (data == NULL || size == 0u) {
        return;
    }

    if (size > VISION_RX_BUFFER_SIZE) {
        size = VISION_RX_BUFFER_SIZE;
    }
    memcpy(text, data, size);
    text[size] = '\0';

    cursor = text;
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') {
        cursor++;
    }

    if ((*cursor < '0' || *cursor > '9') && *cursor != '-' && *cursor != '+') {
        return;
    }

    for (uint8_t i = 0; i < 4u; ++i) {
        char *end;
        long parsed = strtol(cursor, &end, 10);

        if (end == cursor) {
            return;
        }

        values[i] = (int32_t)parsed;
        cursor = end;

        while (*cursor == ' ' || *cursor == '\t') {
            cursor++;
        }
        if (*cursor == ',') {
            cursor++;
        } else {
            break;
        }
    }

    vision_set_values(values);
}

static void vision_receive_restart(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart7, vision_rx_buffer, VISION_RX_BUFFER_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}

static void motors_send_limp(void)
{
    (void)DM4310_SendMit(&hfdcan1, yaw_motor.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    (void)DM4310_SendMit(&hfdcan1, pitch_motor.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

static void motors_send_home_once(void)
{
    (void)DM4310_SendMit(&hfdcan1, yaw_motor.id, YAW_CENTER_RAD, 0.0f, MIT_HOME_KP, MIT_HOME_KD, 0.0f);
    (void)DM4310_SendMit(&hfdcan1, pitch_motor.id, PITCH_CENTER_RAD, 0.0f, MIT_HOME_KP, MIT_HOME_KD, 0.0f);
}

static void motors_home_blocking(void)
{
    const uint32_t count = MOTOR_HOME_MS / MOTOR_HOME_PERIOD_MS;

    for (uint32_t i = 0; i < count; ++i) {
        motors_send_home_once();
        HAL_Delay(MOTOR_HOME_PERIOD_MS);
    }
}

static void visual_loop_100hz(void)
{
    int32_t v0;
    int32_t v1;
    int32_t v2;
    bool timeout;

    v0 = vision_state.value[0];
    v1 = vision_state.value[1];
    v2 = vision_state.value[2];
    timeout = !vision_state.received || ((HAL_GetTick() - vision_state.tick_ms) > VISION_TIMEOUT_MS);

    if (timeout || is_pair(v0, v1, VISION_OFF_CODE)) {
        yaw_speed_rpm = 0.0f;
        pitch_speed_rpm = 0.0f;
        pitch_hold_center = false;
        motor_limp = true;
        return;
    }

    if (is_pair(v0, v1, VISION_NONE_CODE)) {
        yaw_speed_rpm = 0.0f;
        pitch_speed_rpm = 0.0f;
        pitch_hold_center = false;
        motor_limp = true;
        return;
    }

    motor_limp = false;

    if (is_pair(v0, v1, VISION_SEARCH_CODE)) {
        yaw_speed_rpm = (float)v2 / VISION_SEARCH_SPEED_SCALE;
        pitch_speed_rpm = 0.0f;
        pitch_hold_center = true;
        return;
    }

    if (v0 < -VISION_MAX_X || v0 > VISION_MAX_X || v1 < -VISION_MAX_Y || v1 > VISION_MAX_Y) {
        yaw_speed_rpm = 0.0f;
        pitch_speed_rpm = 0.0f;
        pitch_hold_center = false;
        return;
    }

    yaw_speed_rpm = pid_calc(&yaw_pid, (float)v0);
    pitch_speed_rpm = pid_calc(&pitch_pid, (float)v1);
    pitch_hold_center = false;
}

static void motor_loop_200hz(void)
{
    float pitch_speed = pitch_speed_rpm;

    if (motor_limp) {
        motors_send_limp();
        return;
    }

    if (pitch_hold_center) {
        (void)DM4310_SendMit(&hfdcan1, yaw_motor.id,
                             0.0f,
                             DM4310_RpmToRadPerSec(yaw_speed_rpm),
                             0.0f,
                             MIT_SPEED_KD,
                             0.0f);
        (void)DM4310_SendMit(&hfdcan1, pitch_motor.id,
                             PITCH_CENTER_RAD,
                             0.0f,
                             MIT_HOME_KP,
                             MIT_HOME_KD,
                             0.0f);
        return;
    }

    if (pitch_feedback_ready) {
        if (pitch_speed > 0.0f && pitch_motor.fb.pos > PITCH_MAX_RAD) {
            pitch_speed = 0.0f;
        } else if (pitch_speed < 0.0f && pitch_motor.fb.pos < PITCH_MIN_RAD) {
            pitch_speed = 0.0f;
        }
    }

    (void)DM4310_SendMit(&hfdcan1, yaw_motor.id,
                         0.0f,
                         DM4310_RpmToRadPerSec(yaw_speed_rpm),
                         0.0f,
                         MIT_SPEED_KD,
                         0.0f);
    (void)DM4310_SendMit(&hfdcan1, pitch_motor.id,
                         0.0f,
                         DM4310_RpmToRadPerSec(pitch_speed),
                         0.0f,
                         MIT_SPEED_KD,
                         0.0f);
}

void GimbalControl_Init(void)
{
    DM4310_InitMotor(&yaw_motor, YAW_MOTOR_ID, DM4310_MODE_MIT);
    DM4310_InitMotor(&pitch_motor, PITCH_MOTOR_ID, DM4310_MODE_MIT);

    if (DM4310_FdcanConfigAllPass(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    if (DM4310_FdcanStartWithRxFifo0(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    HAL_Delay(50);
    (void)DM4310_ClearError(&hfdcan1, &yaw_motor);
    (void)DM4310_ClearError(&hfdcan1, &pitch_motor);
    HAL_Delay(10);
    (void)DM4310_Enable(&hfdcan1, &yaw_motor);
    (void)DM4310_Enable(&hfdcan1, &pitch_motor);
    HAL_Delay(10);

    motors_home_blocking();
    motors_send_limp();

    vision_receive_restart();

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
        Error_Handler();
    }
}

void GimbalControl_Background(void)
{
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == UART7) {
        vision_parse_line(vision_rx_buffer, size);
        vision_receive_restart();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        vision_receive_restart();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (hfdcan->Instance != FDCAN1 || (rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            break;
        }

        if (header.IdType == FDCAN_STANDARD_ID && header.DataLength == FDCAN_DLC_BYTES_8) {
            const int index = DM4310_DispatchFeedbackPtr(motor_list, 2u, data, 8u);
            if (index == 1) {
                pitch_feedback_ready = true;
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        visual_loop_100hz();
    } else if (htim->Instance == TIM7) {
        motor_loop_200hz();
    }
}
