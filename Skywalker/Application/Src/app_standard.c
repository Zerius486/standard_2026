#include "app_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_protocol.h"
#include "app_referee.h"
#include "bsp_fdcan.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "app_unit_test.h"
#include "dev_bmi088.h"
#include "dev_dji_dt7.h"
#include "dev_dji_motor.h"
#include "dev_dm_motor.h"
#include "fdcan.h"
#include "spi.h"
#include "usart.h"

// 上赛季Standard机器人的Skywalker版重写。这个文件集中覆盖四个RTOS任务入口，
// 方便后续将整车行为作为模板或范例整体替换。

enum {
  // 1U: 接入裁判系统；0U: 不接入裁判系统，使用本文件内默认参数。
  kStandardRefereeEnabled = 0U,
  // 只接底盘时先关闭云台、发射和视觉链路，只跑DT7 + FDCAN3底盘。
  kStandardChassisEnabled = 1U,
  kStandardGimbalEnabled = 0U,
  kStandardShootEnabled = 0U,
  kStandardVisionEnabled = 0U,
  kStandardRemoteSwitchSafeModeEnabled = 0U,
  kStandardUnitTestMask = kUnitTestNone,
  kStandardManagerPeriodMs = 10,
  kStandardGimbalPeriodMs = 2,
  kStandardChassisPeriodMs = 2,
  kStandardShootPeriodMs = 2,
  kStandardMotorCount = 4,
  kStandardCanDataLength = 8,
  kStandardDt7OfflineTimeoutMs = 100,
  kStandardImuGyroZAxis = 2,
};

static const float kStandardChassisRadiusM = 0.15f;
static const float kStandardWheelRadiusM = 0.01f;
static const float kStandardMaxVxMps = 5.0f;
static const float kStandardMaxVyMps = 5.0f;
static const float kStandardMaxOmegaRadps = 10.0f;
static const float kStandardNoRefereeCurrentScale = 0.5f;
static const float kStandardChassisYawAngleKp = 6.0f;
static const float kStandardChassisYawRateFeedbackKp = 0.3f;
static const float kStandardChassisAutoSpinOmegaRadps = 10.0f;
static const float kStandardImuYawRateSign = 1.0f;
static const float kStandardPitchOffsetRad = -1.66286434f;
static const float kStandardYawManualScale = 0.5f;
static const float kStandardPitchMouseScale = 0.5f;
static const float kStandardPitchRockerScale = 5.0f;
static const float kStandardBulletFeedOmegaRadps = 500.0f;
static const float kStandardFrictionOmegaRadps = 800.0f;
static const float kStandardDefaultChassisPowerW = 100.0f;

// M3508底盘功率估计参数，保留旧车测试得到的基础模型。
static const float kStandardPowerK0 = 0.30f;
static const float kStandardPowerK1 = 0.37f;
static const float kStandardPowerK2 = 8.30f;
static const float kStandardPowerK3 = 0.0f;

typedef struct {
  float vx;
  float vy;
  float omega_z;
} StandardVelocity;

typedef struct {
  StandardVelocity desired_velocity;
  StandardVelocity actual_velocity;
  float estimated_power;
  float yaw_angle;
  float target_yaw_angle;
  float yaw_rate;
  float control_dt;
  Bmi088Status imu_status;
  bool imu_ready;
} StandardChassisState;

typedef struct {
  float yaw_angle;
  float pitch_angle;
  float yaw_rate;
  float pitch_rate;
} StandardGimbalState;

typedef struct {
  DjiMotor chassis_motor[kStandardMotorCount];
  DjiMotor friction_motor[2];
  DjiMotor bullet_feed_motor;
  DjiMotor pitch_motor;
  DmMotor yaw_motor;
  StandardChassisState chassis;
  StandardGimbalState gimbal;
  uint8_t fdcan1_tx_data[kStandardCanDataLength];
  uint8_t fdcan2_tx_data[kStandardCanDataLength];
  uint8_t fdcan3_tx_data[kStandardCanDataLength];
  uint16_t bullet_count;
  uint32_t chassis_imu_last_tick;
  volatile bool is_initialized;
  volatile bool is_initializing;
} StandardApp;

static StandardApp g_standard;

static bool StandardIsUnitTestMode(void) {
  return (uint32_t)kStandardUnitTestMask != (uint32_t)kUnitTestNone;
}

static float StandardClamp(float value, float min, float max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

static float StandardRcNormalize(int16_t value) {
  return StandardClamp((float)value / 660.0f, -1.0f, 1.0f);
}

static float StandardWrapPi(float angle) {
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

static void StandardPackDjiCurrents(const DjiMotor *motor_0,
                                    const DjiMotor *motor_1,
                                    const DjiMotor *motor_2,
                                    const DjiMotor *motor_3,
                                    uint8_t tx_data[8]) {
  const DjiMotor *motors[4] = {motor_0, motor_1, motor_2, motor_3};
  memset(tx_data, 0, 8);
  for (uint8_t i = 0; i < 4U; i++) {
    if (motors[i] == NULL) {
      continue;
    }
    int16_t current_units = DjiMotorCurrentAmperesToUnits(
        motors[i]->type, motors[i]->state.given_current);
    tx_data[i * 2U] = (uint8_t)(current_units >> 8);
    tx_data[i * 2U + 1U] = (uint8_t)(current_units & 0xFFU);
  }
}

static void StandardFdcan1RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  switch (std_id) {
    case 0x201U:
      DjiMotorUpdate(&g_standard.friction_motor[0], rx_buffer);
      break;
    case 0x202U:
      DjiMotorUpdate(&g_standard.friction_motor[1], rx_buffer);
      break;
    case 0x20BU:
      DjiMotorUpdate(&g_standard.pitch_motor, rx_buffer);
      break;
    default:
      break;
  }
}

static void StandardFdcan2RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  if (std_id == g_standard.yaw_motor.state.id ||
      std_id == g_standard.yaw_motor.state.master_id) {
    DmMotorUpdate(&g_standard.yaw_motor, rx_buffer);
  }
}

static void StandardFdcan3RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  if (std_id >= 0x201U && std_id <= 0x204U) {
    DjiMotorUpdate(&g_standard.chassis_motor[std_id - 0x201U], rx_buffer);
  } else if (std_id == 0x205U) {
    DjiMotorUpdate(&g_standard.bullet_feed_motor, rx_buffer);
  }
}

static void StandardMotorInit(void) {
  static PidConfig m2006_speed_pid = {
      .kp = 1.515502929687f,
      .ki = 1.165771484375f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 5.0f,
      .integral_max = 6.103515625f,
  };
  static PidConfig m3508_speed_pid = {
      .kp = 0.5515502929687f,
      .ki = 0.0f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 14.6484375f,
      .integral_max = 6.103515625f,
  };
  static PidConfig gm6020_speed_pid = {
      .kp = 0.35f,
      .ki = 6.75f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 0.54f,
      .integral_max = 0.54f,
  };
  static PidConfig gm6020_position_pid = {
      .kp = 5.50f,
      .ki = 0.0f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_feedforward_enabled = true,
      .kff = 0.1f,
      .is_antiwindup_enabled = true,
      .output_max = 10.0f,
      .integral_max = 10.0f,
  };

  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    DjiMotorInit(&g_standard.chassis_motor[i], kDjiMotorM3508,
                 kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  }
  DjiMotorInit(&g_standard.friction_motor[0], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_standard.friction_motor[1], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_standard.bullet_feed_motor, kDjiMotorM2006,
               kDjiMotorModeSpeed, &m2006_speed_pid, NULL);
  DjiMotorInit(&g_standard.pitch_motor, kDjiMotorGm6020,
               kDjiMotorModePosition, &gm6020_speed_pid,
               &gm6020_position_pid);
  DmMotorInit(&g_standard.yaw_motor, kDmMotorTypeJ4310, kDmMotorModeVelocity,
              0x01U, 0x11U);
}

static void StandardSharedInit(void) {
  if (g_standard.is_initialized) {
    return;
  }

  uint32_t lock = osKernelLock();
  if (g_standard.is_initialized || g_standard.is_initializing) {
    (void)osKernelRestoreLock(lock);
    while (!g_standard.is_initialized) {
      osDelay(1);
    }
    return;
  }
  memset(&g_standard, 0, sizeof(g_standard));
  g_standard.is_initializing = true;
  (void)osKernelRestoreLock(lock);

  StandardMotorInit();
  if (kStandardGimbalEnabled != 0U || kStandardShootEnabled != 0U) {
    FdcanInit(&hfdcan1, StandardFdcan1RxCallback);
  }
  if (kStandardGimbalEnabled != 0U) {
    FdcanInit(&hfdcan2, StandardFdcan2RxCallback);
  }
  if (kStandardChassisEnabled != 0U || kStandardShootEnabled != 0U) {
    FdcanInit(&hfdcan3, StandardFdcan3RxCallback);
  }
  UartInit(&huart5, Dt7RxCallback);
  if (kStandardVisionEnabled != 0U) {
    UartInit(&huart10, VisionProtocolRxCallback);
  }
  if (kStandardRefereeEnabled != 0U) {
    RefereeInit(&g_referee, &huart1, &huart1);
  }
  if (kStandardChassisEnabled != 0U) {
    g_standard.chassis.imu_status = Bmi088Init(&g_bmi088, &hspi2);
    if (g_standard.chassis.imu_status == kBmi088NoError) {
      g_standard.chassis.imu_status =
          Bmi088CalibrateGyroOffset(&g_bmi088, 300U, 1U);
      g_standard.chassis.imu_ready =
          g_standard.chassis.imu_status == kBmi088NoError;
    }
    g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
  }

  if (kStandardGimbalEnabled != 0U) {
    DmMotorPackEnable(&g_standard.yaw_motor, g_standard.fdcan2_tx_data);
    FdcanTransmit(&hfdcan2, g_standard.fdcan2_tx_data, kStandardCanDataLength,
                  DmMotorTxId(&g_standard.yaw_motor));
  }
  g_standard.chassis_imu_last_tick = osKernelGetTickCount();
  g_standard.is_initialized = true;
  g_standard.is_initializing = false;
}

static float StandardEstimateChassisPower(void) {
  float sum_power = 0.0f;
  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    float current = g_standard.chassis_motor[i].state.given_current;
    float omega = g_standard.chassis_motor[i].state.omega;
    float torque = kStandardPowerK0 * current;
    float wheel_power = torque * omega + kStandardPowerK1 * fabsf(omega) +
                        kStandardPowerK2 * torque * torque +
                        kStandardPowerK3 / (float)kStandardMotorCount;
    if (wheel_power > 0.0f) {
      sum_power += wheel_power;
    }
  }
  return sum_power;
}

static void StandardApplyChassisPowerLimit(float max_power) {
  float command_power = StandardEstimateChassisPower();
  if (max_power <= 0.0f) {
    for (uint8_t i = 0; i < kStandardMotorCount; i++) {
      g_standard.chassis_motor[i].state.given_current = 0.0f;
    }
    g_standard.chassis.estimated_power = 0.0f;
    return;
  }

  if (command_power > max_power) {
    float scale = max_power / command_power;
    for (uint8_t i = 0; i < kStandardMotorCount; i++) {
      g_standard.chassis_motor[i].state.given_current *= scale;
    }
    g_standard.chassis.estimated_power = max_power;
  } else {
    g_standard.chassis.estimated_power = command_power;
  }
}

static float StandardChassisPowerLimit(void) {
  if (kStandardRefereeEnabled != 0U &&
      g_referee.game_robot_state.chassis_power_limit > 0U) {
    return (float)g_referee.game_robot_state.chassis_power_limit;
  }
  return kStandardDefaultChassisPowerW;
}

static void StandardChassisImuUpdate(void) {
  uint32_t now = osKernelGetTickCount();
  uint32_t elapsed_ms = now - g_standard.chassis_imu_last_tick;
  g_standard.chassis_imu_last_tick = now;
  g_standard.chassis.control_dt = (float)elapsed_ms * 0.001f;

  if (!g_standard.chassis.imu_ready) {
    return;
  }

  Bmi088Status status = Bmi088Update(&g_bmi088);
  g_standard.chassis.imu_status = status;
  if (status != kBmi088NoError) {
    g_standard.chassis.imu_ready = false;
    g_standard.chassis.yaw_rate = 0.0f;
    g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
    return;
  }

  float yaw_rate =
      kStandardImuYawRateSign * g_bmi088.corrected_gyro[kStandardImuGyroZAxis];
  g_standard.chassis.yaw_rate = yaw_rate;
  g_standard.chassis.yaw_angle = StandardWrapPi(
      g_standard.chassis.yaw_angle + yaw_rate * (float)elapsed_ms * 0.001f);
}

static void StandardChassisKinematics(void) {
  float vx_input = -StandardRcNormalize(g_dt7_object.rocker.ch3) -
                   (float)g_dt7_object.key.w + (float)g_dt7_object.key.s;
  float vy_input = StandardRcNormalize(g_dt7_object.rocker.ch2) +
                   (float)g_dt7_object.key.a - (float)g_dt7_object.key.d;
  float manual_yaw_rate_input = -StandardRcNormalize(g_dt7_object.rocker.wheel) -
                                (float)g_dt7_object.key.q +
                                (float)g_dt7_object.key.e;
  float desired_yaw_rate = StandardClamp(manual_yaw_rate_input, -1.0f, 1.0f) *
                           kStandardMaxOmegaRadps;
  if (g_standard.chassis.imu_ready) {
    g_standard.chassis.target_yaw_angle = StandardWrapPi(
        g_standard.chassis.target_yaw_angle +
        desired_yaw_rate * g_standard.chassis.control_dt);
    float yaw_error = StandardWrapPi(g_standard.chassis.target_yaw_angle -
                                     g_standard.chassis.yaw_angle);
    desired_yaw_rate = StandardClamp(kStandardChassisYawAngleKp * yaw_error,
                                     -kStandardMaxOmegaRadps,
                                     kStandardMaxOmegaRadps);
  }
  if (SwitchIsUp(g_dt7_object.rocker.sw1)) {
    desired_yaw_rate += kStandardChassisAutoSpinOmegaRadps;
    if (g_standard.chassis.imu_ready) {
      g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
    }
  }

  g_standard.chassis.desired_velocity.vx =
      StandardClamp(vx_input, -1.0f, 1.0f) * kStandardMaxVxMps;
  g_standard.chassis.desired_velocity.vy =
      StandardClamp(vy_input, -1.0f, 1.0f) * kStandardMaxVyMps;
  g_standard.chassis.desired_velocity.omega_z = desired_yaw_rate;

  float field_vx = g_standard.chassis.desired_velocity.vx;
  float field_vy = g_standard.chassis.desired_velocity.vy;
  float yaw = g_standard.chassis.imu_ready ? g_standard.chassis.yaw_angle : 0.0f;
  float cos_yaw = cosf(yaw);
  float sin_yaw = sinf(yaw);
  float vx = cos_yaw * field_vx + sin_yaw * field_vy;
  float vy = -sin_yaw * field_vx + cos_yaw * field_vy;
  float wz = desired_yaw_rate;
  if (g_standard.chassis.imu_ready) {
    wz += kStandardChassisYawRateFeedbackKp *
          (desired_yaw_rate - g_standard.chassis.yaw_rate);
  }
  wz = StandardClamp(wz, -kStandardMaxOmegaRadps, kStandardMaxOmegaRadps);
  g_standard.chassis.actual_velocity.vx = vx;
  g_standard.chassis.actual_velocity.vy = vy;
  g_standard.chassis.actual_velocity.omega_z = g_standard.chassis.yaw_rate;
  float rotation_speed = wz * kStandardChassisRadiusM;

  g_standard.chassis_motor[0].state.given_omega =
      (-0.707f * vx + 0.707f * vy + rotation_speed) /
      kStandardWheelRadiusM;
  g_standard.chassis_motor[1].state.given_omega =
      (-0.707f * vx - 0.707f * vy + rotation_speed) /
      kStandardWheelRadiusM;
  g_standard.chassis_motor[2].state.given_omega =
      (0.707f * vx - 0.707f * vy + rotation_speed) /
      kStandardWheelRadiusM;
  g_standard.chassis_motor[3].state.given_omega =
      (0.707f * vx + 0.707f * vy + rotation_speed) / kStandardWheelRadiusM;
}

static void StandardChassisControl(void) {
  StandardChassisImuUpdate();
  StandardChassisKinematics();

  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    DjiMotorCurrentCalculate(&g_standard.chassis_motor[i]);
  }
  if (kStandardRefereeEnabled != 0U) {
    StandardApplyChassisPowerLimit(StandardChassisPowerLimit());
  } else {
    for (uint8_t i = 0; i < kStandardMotorCount; i++) {
      g_standard.chassis_motor[i].state.given_current *=
          kStandardNoRefereeCurrentScale;
    }
    g_standard.chassis.estimated_power = StandardEstimateChassisPower();
  }
  StandardPackDjiCurrents(&g_standard.chassis_motor[0],
                          &g_standard.chassis_motor[1],
                          &g_standard.chassis_motor[2],
                          &g_standard.chassis_motor[3],
                          g_standard.fdcan3_tx_data);
  FdcanTransmit(&hfdcan3, g_standard.fdcan3_tx_data, kStandardCanDataLength,
                0x200U);
}

static void StandardGimbalControl(void) {
  if (g_vision_to_gimbal.mode == 1U || g_vision_to_gimbal.mode == 2U) {
    g_standard.gimbal.yaw_rate = g_vision_to_gimbal.yaw_velocity;
    g_standard.gimbal.pitch_angle =
        g_vision_to_gimbal.pitch + kStandardPitchOffsetRad;
  } else {
    g_standard.gimbal.yaw_rate =
        kStandardYawManualScale * (float)g_dt7_object.mouse.x +
        kStandardYawManualScale * StandardRcNormalize(g_dt7_object.rocker.ch0);
    g_standard.gimbal.pitch_angle =
        kStandardPitchOffsetRad +
        kStandardPitchMouseScale * (float)g_dt7_object.mouse.y +
        kStandardPitchRockerScale *
            StandardRcNormalize(g_dt7_object.rocker.ch1);
  }

  g_standard.pitch_motor.state.given_angle = g_standard.gimbal.pitch_angle;
  DjiMotorCurrentCalculate(&g_standard.pitch_motor);

  // GM6020 ID 0x20B 对应高ID组发送帧0x2FE的第4路电流。
  StandardPackDjiCurrents(NULL, NULL, NULL, &g_standard.pitch_motor,
                          g_standard.fdcan1_tx_data);
  FdcanTransmit(&hfdcan1, g_standard.fdcan1_tx_data, kStandardCanDataLength,
                0x2FEU);

  g_standard.yaw_motor.state.given_omega = g_standard.gimbal.yaw_rate;
  DmMotorPackVelocity(&g_standard.yaw_motor, g_standard.fdcan2_tx_data);
  FdcanTransmit(&hfdcan2, g_standard.fdcan2_tx_data, kStandardCanDataLength,
                DmMotorTxId(&g_standard.yaw_motor));
  if (g_standard.yaw_motor.state.error_code != 0U) {
    DmMotorPackClearError(&g_standard.yaw_motor, g_standard.fdcan2_tx_data);
    FdcanTransmit(&hfdcan2, g_standard.fdcan2_tx_data, kStandardCanDataLength,
                  DmMotorTxId(&g_standard.yaw_motor));
  }
}

static void StandardShootControl(void) {
  bool should_feed =
      g_dt7_object.mouse.l != 0U || g_vision_to_gimbal.mode == 2U;
  bool should_spin = SwitchIsUp(g_dt7_object.rocker.sw2);

  g_standard.bullet_feed_motor.state.given_omega =
      should_feed ? kStandardBulletFeedOmegaRadps : 0.0f;
  g_standard.friction_motor[0].state.given_omega =
      should_spin ? -kStandardFrictionOmegaRadps : 0.0f;
  g_standard.friction_motor[1].state.given_omega =
      should_spin ? kStandardFrictionOmegaRadps : 0.0f;

  DjiMotorCurrentCalculate(&g_standard.bullet_feed_motor);
  StandardPackDjiCurrents(&g_standard.bullet_feed_motor, NULL, NULL, NULL,
                          g_standard.fdcan3_tx_data);
  FdcanTransmit(&hfdcan3, g_standard.fdcan3_tx_data, kStandardCanDataLength,
                0x1FFU);

  DjiMotorCurrentCalculate(&g_standard.friction_motor[0]);
  DjiMotorCurrentCalculate(&g_standard.friction_motor[1]);
  StandardPackDjiCurrents(&g_standard.friction_motor[0],
                          &g_standard.friction_motor[1], NULL, NULL,
                          g_standard.fdcan1_tx_data);
  FdcanTransmit(&hfdcan1, g_standard.fdcan1_tx_data, kStandardCanDataLength,
                0x200U);
}

static void StandardStopChassisMotors(void) {
  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    g_standard.chassis_motor[i].state.given_omega = 0.0f;
    g_standard.chassis_motor[i].state.given_current = 0.0f;
  }
  g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
  g_standard.chassis_imu_last_tick = osKernelGetTickCount();
}

static void StandardStopGimbalMotors(void) {
  g_standard.pitch_motor.state.given_angle = kStandardPitchOffsetRad;
  g_standard.pitch_motor.state.given_omega = 0.0f;
  g_standard.pitch_motor.state.given_current = 0.0f;
  g_standard.yaw_motor.state.given_omega = 0.0f;
}

static void StandardStopShootMotors(void) {
  g_standard.bullet_feed_motor.state.given_omega = 0.0f;
  g_standard.bullet_feed_motor.state.given_current = 0.0f;
  g_standard.friction_motor[0].state.given_omega = 0.0f;
  g_standard.friction_motor[0].state.given_current = 0.0f;
  g_standard.friction_motor[1].state.given_omega = 0.0f;
  g_standard.friction_motor[1].state.given_current = 0.0f;
}

static void StandardTransmitGimbalStopCurrents(void) {
  memset(g_standard.fdcan1_tx_data, 0, sizeof(g_standard.fdcan1_tx_data));
  DmMotorPackVelocity(&g_standard.yaw_motor, g_standard.fdcan2_tx_data);

  FdcanTransmit(&hfdcan1, g_standard.fdcan1_tx_data, kStandardCanDataLength,
                0x2FEU);
  FdcanTransmit(&hfdcan2, g_standard.fdcan2_tx_data, kStandardCanDataLength,
                DmMotorTxId(&g_standard.yaw_motor));
}

static void StandardTransmitChassisStopCurrents(void) {
  memset(g_standard.fdcan3_tx_data, 0, sizeof(g_standard.fdcan3_tx_data));

  FdcanTransmit(&hfdcan3, g_standard.fdcan3_tx_data, kStandardCanDataLength,
                0x200U);
}

static void StandardTransmitShootStopCurrents(void) {
  memset(g_standard.fdcan1_tx_data, 0, sizeof(g_standard.fdcan1_tx_data));
  memset(g_standard.fdcan3_tx_data, 0, sizeof(g_standard.fdcan3_tx_data));

  FdcanTransmit(&hfdcan1, g_standard.fdcan1_tx_data, kStandardCanDataLength,
                0x200U);
  FdcanTransmit(&hfdcan3, g_standard.fdcan3_tx_data, kStandardCanDataLength,
                0x1FFU);
}

static bool StandardDt7IsOnline(void) {
  if (Dt7FrameCount() == 0U) {
    return false;
  }
  return (uint32_t)(osKernelGetTickCount() - Dt7LastUpdateTick()) <=
         kStandardDt7OfflineTimeoutMs;
}

static bool StandardIsSafeMode(void) {
  if (!StandardDt7IsOnline()) {
    return true;
  }
  return kStandardRemoteSwitchSafeModeEnabled != 0U &&
         SwitchIsDown(g_dt7_object.rocker.sw1);
}

static void StandardEnterGimbalSafeMode(void) {
  StandardStopGimbalMotors();
  StandardTransmitGimbalStopCurrents();
}

static void StandardEnterChassisSafeMode(void) {
  StandardStopChassisMotors();
  StandardTransmitChassisStopCurrents();
}

static void StandardEnterShootSafeMode(void) {
  StandardStopShootMotors();
  StandardTransmitShootStopCurrents();
}

static void StandardUpdateVisionTx(void) {
  g_gimbal_to_vision.mode = 0U;
  g_gimbal_to_vision.q[0] = 1.0f;
  g_gimbal_to_vision.q[1] = 0.0f;
  g_gimbal_to_vision.q[2] = 0.0f;
  g_gimbal_to_vision.q[3] = 0.0f;
  g_gimbal_to_vision.yaw = g_standard.yaw_motor.state.angle;
  g_gimbal_to_vision.yaw_velocity = g_standard.yaw_motor.state.omega;
  g_gimbal_to_vision.pitch = g_standard.pitch_motor.state.angle;
  g_gimbal_to_vision.pitch_velocity = g_standard.pitch_motor.state.omega;
  g_gimbal_to_vision.bullet_speed =
      (kStandardRefereeEnabled != 0U) ? g_referee.shoot_data.bullet_speed
                                      : 0.0f;
  g_gimbal_to_vision.bullet_count = g_standard.bullet_count;
}

uint32_t ManagerTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return UnitTestInit(kStandardUnitTestMask);
  }
  StandardSharedInit();
  return kStandardManagerPeriodMs;
}

void ManagerTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    UnitTestLoop();
    return;
  }
  if (kStandardRefereeEnabled != 0U) {
    RefereeUpdate(&g_referee);
  }
  if (kStandardVisionEnabled != 0U) {
    StandardUpdateVisionTx();
    VisionProtocolTransmitGimbalState();
  }
}

uint32_t GimbalTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return kStandardGimbalPeriodMs;
  }
  if (kStandardGimbalEnabled == 0U) {
    return kStandardGimbalPeriodMs;
  }
  StandardSharedInit();
  return kStandardGimbalPeriodMs;
}

void GimbalTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    return;
  }
  if (kStandardGimbalEnabled == 0U) {
    return;
  }
  if (StandardIsSafeMode()) {
    StandardEnterGimbalSafeMode();
    return;
  }
  StandardGimbalControl();
}

uint32_t ChassisTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return kStandardChassisPeriodMs;
  }
  if (kStandardChassisEnabled == 0U) {
    return kStandardChassisPeriodMs;
  }
  StandardSharedInit();
  return kStandardChassisPeriodMs;
}

void ChassisTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    return;
  }
  if (kStandardChassisEnabled == 0U) {
    return;
  }
  if (StandardIsSafeMode()) {
    StandardEnterChassisSafeMode();
    return;
  }
  StandardChassisControl();
}

uint32_t ShootTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return kStandardShootPeriodMs;
  }
  if (kStandardShootEnabled == 0U) {
    return kStandardShootPeriodMs;
  }
  StandardSharedInit();
  return kStandardShootPeriodMs;
}

void ShootTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    return;
  }
  if (kStandardShootEnabled == 0U) {
    return;
  }
  if (StandardIsSafeMode()) {
    StandardEnterShootSafeMode();
    return;
  }
  StandardShootControl();
}
