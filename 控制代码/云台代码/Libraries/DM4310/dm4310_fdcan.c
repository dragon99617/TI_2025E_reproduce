#include "dm4310_fdcan.h"

#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DM4310_ENABLE_CMD      0xFCu
#define DM4310_DISABLE_CMD     0xFDu
#define DM4310_SAVE_ZERO_CMD   0xFEu
#define DM4310_CLEAR_ERR_CMD   0xFBu

static float dm4310_clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static uint32_t dm4310_fdcan_dlc(uint8_t len)
{
    switch (len) {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
    }
}

static HAL_StatusTypeDef dm4310_send_mode_command(FDCAN_HandleTypeDef *hfdcan,
                                                  uint16_t motor_id,
                                                  DM4310_Mode mode,
                                                  uint8_t command)
{
    uint8_t data[8] = {
        0xFFu, 0xFFu, 0xFFu, 0xFFu,
        0xFFu, 0xFFu, 0xFFu, command,
    };

    return DM4310_FdcanSend(hfdcan, (uint32_t)(motor_id + (uint16_t)mode), data, 8);
}

void DM4310_InitMotor(DM4310_Motor *motor, uint16_t id, DM4310_Mode mode)
{
    if (motor == NULL) {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->cmd.mode = mode;
    motor->ctrl.mode = mode;
}

void DM4310_SetCommand(DM4310_Motor *motor, float pos, float vel, float kp, float kd, float tor)
{
    if (motor == NULL) {
        return;
    }

    motor->cmd.pos_set = pos;
    motor->cmd.vel_set = vel;
    motor->cmd.kp_set = kp;
    motor->cmd.kd_set = kd;
    motor->cmd.tor_set = tor;
}

void DM4310_CopyCommandToControl(DM4310_Motor *motor)
{
    if (motor == NULL) {
        return;
    }

    motor->ctrl = motor->cmd;
}

void DM4310_ClearCommand(DM4310_Motor *motor)
{
    if (motor == NULL) {
        return;
    }

    motor->cmd.pos_set = 0.0f;
    motor->cmd.vel_set = 0.0f;
    motor->cmd.kp_set = 0.0f;
    motor->cmd.kd_set = 0.0f;
    motor->cmd.tor_set = 0.0f;
    motor->ctrl.pos_set = 0.0f;
    motor->ctrl.vel_set = 0.0f;
    motor->ctrl.kp_set = 0.0f;
    motor->ctrl.kd_set = 0.0f;
    motor->ctrl.tor_set = 0.0f;
}

HAL_StatusTypeDef DM4310_FdcanConfigAllPass(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                        FDCAN_ACCEPT_IN_RX_FIFO0,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,
                                        FDCAN_REJECT_REMOTE);
}

HAL_StatusTypeDef DM4310_FdcanStartWithRxFifo0(FDCAN_HandleTypeDef *hfdcan)
{
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

HAL_StatusTypeDef DM4310_FdcanSend(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t standard_id,
                                   const uint8_t *data,
                                   uint8_t len)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t tx_data[8] = {0};

    if (hfdcan == NULL || data == NULL || len > 8u || standard_id > 0x7FFu) {
        return HAL_ERROR;
    }

    memcpy(tx_data, data, len);

    header.Identifier = standard_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dm4310_fdcan_dlc(len);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, tx_data);
}

HAL_StatusTypeDef DM4310_Enable(FDCAN_HandleTypeDef *hfdcan, const DM4310_Motor *motor)
{
    if (motor == NULL) {
        return HAL_ERROR;
    }

    return dm4310_send_mode_command(hfdcan, motor->id, motor->ctrl.mode, DM4310_ENABLE_CMD);
}

HAL_StatusTypeDef DM4310_Disable(FDCAN_HandleTypeDef *hfdcan, DM4310_Motor *motor)
{
    HAL_StatusTypeDef status;

    if (motor == NULL) {
        return HAL_ERROR;
    }

    status = dm4310_send_mode_command(hfdcan, motor->id, motor->ctrl.mode, DM4310_DISABLE_CMD);
    DM4310_ClearCommand(motor);
    return status;
}

HAL_StatusTypeDef DM4310_SaveZero(FDCAN_HandleTypeDef *hfdcan, const DM4310_Motor *motor)
{
    if (motor == NULL) {
        return HAL_ERROR;
    }

    return dm4310_send_mode_command(hfdcan, motor->id, motor->ctrl.mode, DM4310_SAVE_ZERO_CMD);
}

HAL_StatusTypeDef DM4310_ClearError(FDCAN_HandleTypeDef *hfdcan, const DM4310_Motor *motor)
{
    if (motor == NULL) {
        return HAL_ERROR;
    }

    return dm4310_send_mode_command(hfdcan, motor->id, motor->ctrl.mode, DM4310_CLEAR_ERR_CMD);
}

HAL_StatusTypeDef DM4310_SendControl(FDCAN_HandleTypeDef *hfdcan, const DM4310_Motor *motor)
{
    if (motor == NULL) {
        return HAL_ERROR;
    }

    switch (motor->ctrl.mode) {
    case DM4310_MODE_MIT:
        return DM4310_SendMit(hfdcan,
                              motor->id,
                              motor->ctrl.pos_set,
                              motor->ctrl.vel_set,
                              motor->ctrl.kp_set,
                              motor->ctrl.kd_set,
                              motor->ctrl.tor_set);
    case DM4310_MODE_POSITION_SPEED:
        return DM4310_SendPositionSpeed(hfdcan,
                                        motor->id,
                                        motor->ctrl.pos_set,
                                        motor->ctrl.vel_set);
    case DM4310_MODE_SPEED:
        return DM4310_SendSpeed(hfdcan, motor->id, motor->ctrl.vel_set);
    default:
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef DM4310_SendMit(FDCAN_HandleTypeDef *hfdcan,
                                 uint16_t motor_id,
                                 float pos,
                                 float vel,
                                 float kp,
                                 float kd,
                                 float tor)
{
    uint16_t pos_tmp;
    uint16_t vel_tmp;
    uint16_t kp_tmp;
    uint16_t kd_tmp;
    uint16_t tor_tmp;
    uint8_t data[8];

    pos_tmp = (uint16_t)DM4310_FloatToUint(pos, DM4310_P_MIN, DM4310_P_MAX, 16);
    vel_tmp = (uint16_t)DM4310_FloatToUint(vel, DM4310_V_MIN, DM4310_V_MAX, 12);
    kp_tmp = (uint16_t)DM4310_FloatToUint(kp, DM4310_KP_MIN, DM4310_KP_MAX, 12);
    kd_tmp = (uint16_t)DM4310_FloatToUint(kd, DM4310_KD_MIN, DM4310_KD_MAX, 12);
    tor_tmp = (uint16_t)DM4310_FloatToUint(tor, DM4310_T_MIN, DM4310_T_MAX, 12);

    data[0] = (uint8_t)(pos_tmp >> 8);
    data[1] = (uint8_t)pos_tmp;
    data[2] = (uint8_t)(vel_tmp >> 4);
    data[3] = (uint8_t)(((vel_tmp & 0xFu) << 4) | (kp_tmp >> 8));
    data[4] = (uint8_t)kp_tmp;
    data[5] = (uint8_t)(kd_tmp >> 4);
    data[6] = (uint8_t)(((kd_tmp & 0xFu) << 4) | (tor_tmp >> 8));
    data[7] = (uint8_t)tor_tmp;

    return DM4310_FdcanSend(hfdcan, (uint32_t)(motor_id + (uint16_t)DM4310_MODE_MIT), data, 8);
}

HAL_StatusTypeDef DM4310_SendPositionSpeed(FDCAN_HandleTypeDef *hfdcan,
                                           uint16_t motor_id,
                                           float pos,
                                           float vel)
{
    uint8_t data[8];

    memcpy(&data[0], &pos, sizeof(pos));
    memcpy(&data[4], &vel, sizeof(vel));

    return DM4310_FdcanSend(hfdcan,
                            (uint32_t)(motor_id + (uint16_t)DM4310_MODE_POSITION_SPEED),
                            data,
                            8);
}

HAL_StatusTypeDef DM4310_SendSpeed(FDCAN_HandleTypeDef *hfdcan,
                                   uint16_t motor_id,
                                   float vel)
{
    uint8_t data[4];

    memcpy(data, &vel, sizeof(vel));

    return DM4310_FdcanSend(hfdcan,
                            (uint32_t)(motor_id + (uint16_t)DM4310_MODE_SPEED),
                            data,
                            4);
}

void DM4310_ParseFeedback(DM4310_Motor *motor, const uint8_t data[8])
{
    if (motor == NULL || data == NULL) {
        return;
    }

    motor->fb.id = data[0] & 0x0Fu;
    motor->fb.state = data[0] >> 4;
    motor->fb.p_int = (uint16_t)((data[1] << 8) | data[2]);
    motor->fb.v_int = (uint16_t)((data[3] << 4) | (data[4] >> 4));
    motor->fb.t_int = (uint16_t)(((data[4] & 0xFu) << 8) | data[5]);
    motor->fb.pos = DM4310_UintToFloat(motor->fb.p_int, DM4310_P_MIN, DM4310_P_MAX, 16);
    motor->fb.vel = DM4310_UintToFloat(motor->fb.v_int, DM4310_V_MIN, DM4310_V_MAX, 12);
    motor->fb.tor = DM4310_UintToFloat(motor->fb.t_int, DM4310_T_MIN, DM4310_T_MAX, 12);
    motor->fb.t_mos = (float)data[6];
    motor->fb.t_coil = (float)data[7];
}

int DM4310_DispatchFeedback(DM4310_Motor *motors,
                            size_t motor_count,
                            const uint8_t data[8],
                            uint8_t len)
{
    uint8_t motor_id;

    if (motors == NULL || data == NULL || len < 8u) {
        return -1;
    }

    motor_id = data[0] & 0x0Fu;
    for (size_t i = 0; i < motor_count; ++i) {
        if ((uint8_t)motors[i].id == motor_id) {
            DM4310_ParseFeedback(&motors[i], data);
            return (int)i;
        }
    }

    return -1;
}

int DM4310_DispatchFeedbackPtr(DM4310_Motor *motors[],
                               size_t motor_count,
                               const uint8_t data[8],
                               uint8_t len)
{
    uint8_t motor_id;

    if (motors == NULL || data == NULL || len < 8u) {
        return -1;
    }

    motor_id = data[0] & 0x0Fu;
    for (size_t i = 0; i < motor_count; ++i) {
        if (motors[i] != NULL && (uint8_t)motors[i]->id == motor_id) {
            DM4310_ParseFeedback(motors[i], data);
            return (int)i;
        }
    }

    return -1;
}

int DM4310_FloatToUint(float value, float min, float max, int bits)
{
    float span;
    float scaled;

    value = dm4310_clampf(value, min, max);
    span = max - min;
    scaled = (value - min) * (float)((1u << bits) - 1u) / span;

    return (int)scaled;
}

float DM4310_UintToFloat(int value, float min, float max, int bits)
{
    const float span = max - min;
    return (float)value * span / (float)((1u << bits) - 1u) + min;
}

float DM4310_RpmToRadPerSec(float rpm)
{
    return rpm * (float)(2.0 * M_PI / 60.0);
}

float DM4310_RadPerSecToRpm(float rad_per_sec)
{
    return rad_per_sec * (float)(60.0 / (2.0 * M_PI));
}
