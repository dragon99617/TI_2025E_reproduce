#ifndef DM4310_FDCAN_H
#define DM4310_FDCAN_H

#include <stddef.h>
#include <stdint.h>

#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DAMIAO DM4310 official example ranges.
 * Position and velocity units are rad and rad/s.
 */
#define DM4310_P_MIN    (-12.5f)
#define DM4310_P_MAX    (12.5f)
#define DM4310_V_MIN    (-45.0f)
#define DM4310_V_MAX    (45.0f)
#define DM4310_T_MIN    (-18.0f)
#define DM4310_T_MAX    (18.0f)
#define DM4310_KP_MIN   (0.0f)
#define DM4310_KP_MAX   (500.0f)
#define DM4310_KD_MIN   (0.0f)
#define DM4310_KD_MAX   (5.0f)

typedef enum {
    DM4310_MODE_MIT = 0x000,
    DM4310_MODE_POSITION_SPEED = 0x100,
    DM4310_MODE_SPEED = 0x200,
} DM4310_Mode;

typedef enum {
    DM4310_STATE_DISABLE = 0,
    DM4310_STATE_ENABLE = 1,
    DM4310_STATE_OVERVOLTAGE = 8,
    DM4310_STATE_UNDERVOLTAGE = 9,
    DM4310_STATE_OVERCURRENT = 10,
    DM4310_STATE_MOS_OVERTEMP = 11,
    DM4310_STATE_COIL_OVERTEMP = 12,
    DM4310_STATE_COMM_LOST = 13,
    DM4310_STATE_OVERLOAD = 14,
} DM4310_State;

typedef struct {
    DM4310_Mode mode;
    float pos_set;
    float vel_set;
    float kp_set;
    float kd_set;
    float tor_set;
} DM4310_Command;

typedef struct {
    uint8_t id;
    uint8_t state;
    uint16_t p_int;
    uint16_t v_int;
    uint16_t t_int;
    float pos;
    float vel;
    float tor;
    float t_mos;
    float t_coil;
} DM4310_Feedback;

typedef struct {
    uint16_t id;
    DM4310_Command cmd;
    DM4310_Command ctrl;
    DM4310_Feedback fb;
} DM4310_Motor;

void DM4310_InitMotor(DM4310_Motor *motor, uint16_t id, DM4310_Mode mode);
void DM4310_SetCommand(DM4310_Motor *motor, float pos, float vel, float kp, float kd, float tor);
void DM4310_CopyCommandToControl(DM4310_Motor *motor);
void DM4310_ClearCommand(DM4310_Motor *motor);

HAL_StatusTypeDef DM4310_FdcanConfigAllPass(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef DM4310_FdcanStartWithRxFifo0(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef DM4310_FdcanSend(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t standard_id,
                                   const uint8_t *data,
                                   uint8_t len);

HAL_StatusTypeDef DM4310_Enable(FDCAN_HandleTypeDef *hfdcan, const DM4310_Motor *motor);
HAL_StatusTypeDef DM4310_Disable(FDCAN_HandleTypeDef *hfdcan, DM4310_Motor *motor);
HAL_StatusTypeDef DM4310_SaveZero(FDCAN_HandleTypeDef *hfdcan, const DM4310_Motor *motor);
HAL_StatusTypeDef DM4310_ClearError(FDCAN_HandleTypeDef *hfdcan, const DM4310_Motor *motor);
HAL_StatusTypeDef DM4310_SendControl(FDCAN_HandleTypeDef *hfdcan, const DM4310_Motor *motor);

HAL_StatusTypeDef DM4310_SendMit(FDCAN_HandleTypeDef *hfdcan,
                                 uint16_t motor_id,
                                 float pos,
                                 float vel,
                                 float kp,
                                 float kd,
                                 float tor);
HAL_StatusTypeDef DM4310_SendPositionSpeed(FDCAN_HandleTypeDef *hfdcan,
                                           uint16_t motor_id,
                                           float pos,
                                           float vel);
HAL_StatusTypeDef DM4310_SendSpeed(FDCAN_HandleTypeDef *hfdcan,
                                   uint16_t motor_id,
                                   float vel);

void DM4310_ParseFeedback(DM4310_Motor *motor, const uint8_t data[8]);
int DM4310_DispatchFeedback(DM4310_Motor *motors,
                            size_t motor_count,
                            const uint8_t data[8],
                            uint8_t len);
int DM4310_DispatchFeedbackPtr(DM4310_Motor *motors[],
                               size_t motor_count,
                               const uint8_t data[8],
                               uint8_t len);

int DM4310_FloatToUint(float value, float min, float max, int bits);
float DM4310_UintToFloat(int value, float min, float max, int bits);
float DM4310_RpmToRadPerSec(float rpm);
float DM4310_RadPerSecToRpm(float rad_per_sec);

#ifdef __cplusplus
}
#endif

#endif
