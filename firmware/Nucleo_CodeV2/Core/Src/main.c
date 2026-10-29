/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#define FEEDBACK_MEMORY_BARRIER() __DMB()
#include "FeedbackPath.h"
#include "BusServoFeedback.h"
#include "TleSscGpio.h"
#include "CalibrationStorage.h"
#include "JointDiagnosticProtocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
#define RX_BUFFER_SIZE 256
#define ENABLE_DRIVE_DIAGNOSTICS 1 // Disable after the drive-path fault is resolved.
uint8_t rx_buffer[RX_BUFFER_SIZE];
char process_buffer[COMMAND_LINE_SIZE];
static CommandQueue command_queue;

enum { SENSOR_MOTOR_L, SENSOR_JOINT_L, SENSOR_MOTOR_R, SENSOR_JOINT_R, SENSOR_ELBOW,
       SENSOR_WRIST, SENSOR_COUNT };
static FeedbackSample feedback_samples[SENSOR_COUNT];
static uint16_t last_spi_data[5], last_spi_safety[5];
typedef struct {
    uint8_t configured, hard_error, attempted;
    uint16_t status;
    uint32_t last_config_attempt, crc_errors, io_errors, retries;
    uint32_t transfers, good_angles, safety_errors, shifted_frames;
    uint32_t last_transfer_us, max_transfer_us; // Timed on GPIO elbow transport.
} TleFeedbackState;
static TleFeedbackState tle_state[5];
typedef struct { SPI_HandleTypeDef *spi; uint16_t pin; } TleDevice;
static const TleDevice tle_devices[5] = {
    {&hspi1, CS_MotorAxle_DOF1_Pin}, {&hspi1, CS_JointAxle_DOF1_Pin},
    {&hspi2, CS_MotorAxle_DOF2_Pin}, {&hspi2, CS_JointAxle_DOF2_Pin},
    {&hspi3, CS_JointAxle_DOF3_Pin}
};
static uint8_t latest_spi_mask, motion_feedback_hold;
static volatile uint16_t portenta_rx_position;
static volatile uint32_t portenta_uart_error, portenta_rx_errors;
static volatile uint32_t portenta_noise_errors, portenta_frame_errors, portenta_overruns;
static volatile uint32_t portenta_parity_errors, portenta_dma_errors;
static volatile uint32_t portenta_queue_peak;
static volatile uint8_t portenta_rx_running;
static uint32_t portenta_last_restart_ms, portenta_rx_restarts, portenta_restart_failures;
static WristAddress wrist_address = {WRIST_FIRST_ID, 0, 0};
static float a1m = NAN, a1j = NAN, a2m = NAN, a2j = NAN, a3j = NAN, a4 = NAN, a5 = NAN;
static uint8_t feedback_fault_latched = 0;
static uint32_t last_wrist_read = 0;
typedef struct {
    int16_t requested;
    volatile uint32_t submitted, completed;
    volatile uint32_t uart_errors, last_uart_error;
    uint32_t busy, failed;
    HAL_StatusTypeDef last_status;
} MotorTxStats;
static MotorTxStats motor_tx_stats[4];
typedef struct {
    uint8_t id, locked, next_register;
    uint32_t successes, failures;
} DriveStatusReader;
static DriveStatusReader drive_status_reader[3];
typedef struct {
    uint8_t bytes[16], count, transmitted;
    uint32_t errors;
} ServoReadTrace;
static ServoReadTrace servo_read_trace[4];

// Variables de Velocidad y Control
int16_t overall_velocity = 500;
uint8_t last_l1_state = 0;
uint8_t last_r1_state = 0;
uint8_t is_manual_mode = 0;

// Posicion del Servo (GDL 5 y 6)
int pwm_pos_5 = 1500;
int pwm_pos_6 = 1500;

// =========================================================================
// POSITION CONTROL VARIABLES
// =========================================================================
uint8_t position_control_active = 0;  // 1 when moving to a target angle

// Calibration offsets: offset = calibrated - raw
// To get raw from calibrated: raw = calibrated - offset
static JointCalibration joint_calibration = {.active = {JC_DEFAULT_OFFSETS}};
#define OFFSET_L (joint_calibration.active.offset[0])
#define OFFSET_R (joint_calibration.active.offset[1])
#define OFFSET_BASE (joint_calibration.active.offset[2])
#define OFFSET_4 (joint_calibration.active.offset[3])
#define OFFSET_5 (joint_calibration.active.offset[4])

// Workspace Limits (Calibrated Angles)
const float LIMIT_ROLL_MIN = -60.0f;
const float LIMIT_ROLL_MAX = 120.0f;
const float LIMIT_PITCH_MIN = -50.0f;
const float LIMIT_PITCH_MAX = 60.0f;
const float LIMIT_DOF3_MIN = -70.0f;
const float LIMIT_DOF3_MAX = 90.0f;
const float LIMIT_DOF4_MIN = -200.0f;
const float LIMIT_DOF4_MAX = 30.0f;
const float LIMIT_DOF5_MIN = -95.0f;
const float LIMIT_DOF5_MAX = 84.0f;
const float LIMIT_GRIPPER_MIN = 0.0f;
const float LIMIT_GRIPPER_MAX = 180.0f;

static uint8_t limits_reply_pending;
static char limits_reply_buf[128];

static void ServiceLimitsReply(void)
{
    if (!limits_reply_pending ||
        huart6.gState != HAL_UART_STATE_READY)
        return;

    int length = snprintf(
        limits_reply_buf, sizeof(limits_reply_buf),
        "L:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
        LIMIT_ROLL_MIN, LIMIT_ROLL_MAX,
        LIMIT_PITCH_MIN, LIMIT_PITCH_MAX,
        LIMIT_DOF3_MIN, LIMIT_DOF3_MAX,
        LIMIT_DOF4_MIN, LIMIT_DOF4_MAX,
        LIMIT_DOF5_MIN, LIMIT_DOF5_MAX,
        LIMIT_GRIPPER_MIN, LIMIT_GRIPPER_MAX);

    if (length > 0 &&
        (size_t)length < sizeof(limits_reply_buf) &&
        HAL_UART_Transmit_IT(
            &huart6,
            (uint8_t *)limits_reply_buf,
            (uint16_t)length) == HAL_OK)
    {
        limits_reply_pending = 0;
    }
}

// Target angles (raw sensor frame, 0-360)
float target_L_raw = 0.0f;
float target_R_raw = 0.0f;
float target_3_raw = 0.0f;
float target_4_raw = 0.0f;
float target_5_raw = 0.0f;
float target_gripper_raw = 0.0f;

// Last commanded positions (for debug telemetry)
float cmd_roll = 0.0f;
float cmd_pitch = 0.0f;
float cmd_base = 0.0f;
float cmd_dof4 = 0.0f;
float cmd_dof5 = 0.0f;
float cmd_gripper = 0.0f;

// Calibrated gear targets (for debug telemetry)
float target_L_cal = 0.0f;
float target_R_cal = 0.0f;
float target_3_cal = 0.0f;
float target_4_cal = 0.0f;
float target_5_cal = 0.0f;
float target_gripper_cal = 0.0f;

// Slew rate internal targets
float slew_L_cal = 0.0f;
float slew_R_cal = 0.0f;
float slew_3_cal = 0.0f;

// PID controller gains for Left Bevel (DOF 1)
float Kp_pos_L = 33.0f;
float Ki_pos_L = 0.0f;
float Kd_pos_L = 1.5f;

// PID controller gains for Right Bevel (DOF 2)
float Kp_pos_R = 33.0f;
float Ki_pos_R = 0.0f;
float Kd_pos_R = 1.5f;

// PD controller gains for DOF 3 (Base)
float Kp_pos_3 = 40.0f;
float Ki_pos_3 = 0.0f;
float Kd_pos_3 = 4.0f;

// Integral accumulators and anti-windup limit
float integral_L = 0.0f;
float integral_R = 0.0f;
float integral_3 = 0.0f;
float integral_limit = 250.0f; // Maximum PWM contribution from the I-term

// Gravity and friction compensation
float Kg_pos_3 = 45.0f;
float Kg_pos_LR = 12.0f;      // Gravity compensation gain for Left/Right bevel gears
float P_weight = 1.5f;       // Ratio of link 1 to link 2 weight
float max_slew_rate_LR = 2.0f; // Max step per 10ms loop (1.0 = 100 deg/sec)
float max_slew_rate_3 = 2.0f;  // Max step per 10ms loop
float u_fric_val_L = 175.0f; // Minimum PWM to overcome gearbox friction (Left Bevel)
float u_fric_val_R = 190.0f; // Minimum PWM to overcome gearbox friction (Right Bevel)
float u_fric_val_3 = 180.0f; // Minimum PWM for DOF 3
float deadband_deg = 0.4f; // Acceptable error margin (degrees)

// Motor/Joint previous readings (for velocity estimation via derivative)
float prev_motor_angle_L = 0.0f;
float prev_motor_angle_R = 0.0f;
float prev_joint_angle_3 = 0.0f;

// Max output for position control (safety clamp)
#define POS_CTRL_MAX_OUTPUT 800

// FSM and Protocol Variables
typedef enum {
    STATE_ESTOP = 0,
    STATE_MODES,
    STATE_MANUAL,
    STATE_AUTO
} SystemState_t;

SystemState_t system_state = STATE_ESTOP;
int target_mode = 0; // 0 = Joint, 1 = Cartesian
int global_velocity = 500;
int move_dirs[6] = {0, 0, 0, 0, 0, 0};
float virtual_angle_5 = 90.0f; // raw angle 0-180 for MG90S velocity simulation
float virtual_gripper = 90.0f; // raw angle 0-180 for gripper velocity simulation

// Non-blocking UART buffers
static uint8_t tx_buf_1[10];
static uint8_t tx_buf_3[10];
static uint8_t tx_buf_4[10];
static uint8_t tx_buf_5[10];
static char telemetry_buf[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Hiwonder_SetMotorSpeed(UART_HandleTypeDef *huart, uint8_t id, int16_t speed);
void Hiwonder_SetMotorSpeed_IT(UART_HandleTypeDef *huart, uint8_t id, int16_t speed);
void Hiwonder_UnlockAngleLimits(UART_HandleTypeDef *huart, uint8_t id);
int Hiwonder_SetServoPosition(UART_HandleTypeDef *huart, uint8_t id, float angle_raw_deg, uint16_t time_ms);
float Read_TLE5012_Angle(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t *w1, uint16_t *w2);
float Read_LX225_Angle(UART_HandleTypeDef *huart, uint8_t id);
uint8_t Calc_TLE5012_CRC8(uint8_t *data, uint8_t len);
float angular_error(float target, float current);
float normalize_angle(float angle);
static void SampleFastFeedback(void);
static void ServiceWristFeedback(uint32_t now);
static int FeedbackReady(uint32_t now);
static int FeedbackWithinAge(uint32_t now);
static int PositionFeedbackReady(uint32_t now);
static void SuperviseMotionFeedback(uint32_t now);
static float RawFeedbackValue(int index, uint32_t now);
static void GetFeedbackAngles(uint32_t now, float q[6]);
static void StopArmOutputs(void);
static void LatchFeedbackFault(const char *reason);
static void ProcessPortentaCommand(char *line);
static void ProcessQueuedCommands(void);
static int ReadTleWord(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio, uint16_t pin,
                       uint16_t command, uint16_t *data, uint16_t *safety);
static int ReadTleRegisters(SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio, uint16_t pin,
                            uint16_t command, uint16_t *words, uint16_t count, uint16_t *safety);
static int ConfigureTleSensor(int index);
static int ElbowSscTransfer(const uint16_t *tx, unsigned tx_count, uint16_t *rx, unsigned rx_count);
static int StartPortentaReceiver(void);
static void ServicePortentaReceiver(uint32_t now);
static void LatchReceiveFault(uint8_t flags);
static void CalibrationAbort(unsigned reason);
static int JD_OwnsUart(void);
static int MotorBusIndex(UART_HandleTypeDef *huart);
static void ServiceDriveDiagnostics(uint32_t now);
static int ServoWritePacket(UART_HandleTypeDef *huart, const uint8_t *packet, uint16_t count);
static int ReadServoRegister(UART_HandleTypeDef *huart, uint8_t id, uint8_t command,
                             uint8_t count, uint8_t *data, uint8_t *source_id);
static void ServiceDriveStatus(uint32_t now);
static void ServoBusTransmit(UART_HandleTypeDef *huart);
static void ServoBusReceive(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redirigir printf a USART2 (Serial Monitor PC)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  if (JD_OwnsUart()) return ch; // USB UART belongs to the bounded binary recorder.
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
static void StopArmOutputs(void) {
    Hiwonder_SetMotorSpeed_IT(&huart1, 254, 0);
    Hiwonder_SetMotorSpeed_IT(&huart3, 254, 0);
    Hiwonder_SetMotorSpeed_IT(&huart4, 254, 0);
    Hiwonder_SetMotorSpeed_IT(&huart5, 254, 0);
}

static void HoldArmOutputs(void) {
    Hiwonder_SetMotorSpeed_IT(&huart1, 254, 0);
    Hiwonder_SetMotorSpeed_IT(&huart3, 254, 0);
    Hiwonder_SetMotorSpeed_IT(&huart4, 254, 0);
    // Do not switch a wrist position servo into motor mode for a brief SPI retry.
    if (!position_control_active) Hiwonder_SetMotorSpeed_IT(&huart5, 254, 0);
}

static float CalibratedLeftGear(void) {
    return Feedback_CalibrateSpiSector(a1j, OFFSET_L, LIMIT_ROLL_MIN - LIMIT_PITCH_MAX,
                                       LIMIT_ROLL_MAX - LIMIT_PITCH_MIN);
}
static float CalibratedRightGear(void) {
    return Feedback_CalibrateSpiSector(a2j, OFFSET_R, LIMIT_ROLL_MIN + LIMIT_PITCH_MIN,
                                       LIMIT_ROLL_MAX + LIMIT_PITCH_MAX);
}
static float CalibratedElbow(void) {
    return Feedback_CalibrateSpiSector(a3j, OFFSET_BASE, LIMIT_DOF3_MIN, LIMIT_DOF3_MAX);
}

static void LatchFeedbackFault(const char *reason) {
    system_state = STATE_ESTOP;
    position_control_active = 0;
    memset(move_dirs, 0, sizeof(move_dirs));
    StopArmOutputs();
    if (joint_calibration.open) CalibrationAbort(JC_BAD_FEEDBACK);
    if (!feedback_fault_latched)
        printf("[FEEDBACK FAULT] %s\r\n", reason);
    feedback_fault_latched = 1;
}

static float RawFeedbackValue(int index, uint32_t now) {
    if (index < SENSOR_WRIST &&
        (!tle_state[index].configured || !(latest_spi_mask & (1U << index)))) return NAN;
    uint32_t age = index == SENSOR_WRIST ? FEEDBACK_WRIST_MAX_AGE_MS : FEEDBACK_SPI_MAX_AGE_MS;
    return Feedback_Fresh(&feedback_samples[index], now, age) ? feedback_samples[index].value : NAN;
}

static int FeedbackReady(uint32_t now) {
    return latest_spi_mask == 0x1F && FeedbackWithinAge(now);
}

static int FeedbackWithinAge(uint32_t now) {
    if (feedback_fault_latched || !wrist_address.locked)
        return 0;
    for (int i = 0; i < SENSOR_COUNT; ++i)
        if ((i < SENSOR_WRIST && !tle_state[i].configured) ||
            !Feedback_Fresh(&feedback_samples[i], now,
                            i == SENSOR_WRIST ? FEEDBACK_WRIST_MAX_AGE_MS : FEEDBACK_SPI_MAX_AGE_MS))
            return 0;
    return 1;
}

static void GetFeedbackAngles(uint32_t now, float q[6]) {
    q[5] = ((float)pwm_pos_6 - 500.0f) * (180.0f / 2000.0f);
    if (!FeedbackReady(now)) {
        for (int i = 0; i < 5; ++i)
            q[i] = NAN;
        return;
    }
    float left = CalibratedLeftGear();
    float right = CalibratedRightGear();
    q[0] = (right - left) / 2.0f;
    q[1] = (right + left) / 2.0f;
    q[2] = CalibratedElbow();
    q[3] = a4 + OFFSET_4;
    q[4] = ((float)pwm_pos_5 - 500.0f) * (180.0f / 2000.0f) + OFFSET_5;
}

static int PositionFeedbackReady(uint32_t now) {
    float q[6];
    GetFeedbackAngles(now, q);
    const float minimum[5] = {LIMIT_PITCH_MIN, LIMIT_ROLL_MIN, LIMIT_DOF3_MIN, LIMIT_DOF4_MIN,
                              LIMIT_DOF5_MIN};
    const float maximum[5] = {LIMIT_PITCH_MAX, LIMIT_ROLL_MAX, LIMIT_DOF3_MAX, LIMIT_DOF4_MAX,
                              LIMIT_DOF5_MAX};
    for (int i = 0; i < 5; ++i)
        if (!isfinite(q[i]) || q[i] < minimum[i] - FEEDBACK_LIMIT_MARGIN_DEG ||
            q[i] > maximum[i] + FEEDBACK_LIMIT_MARGIN_DEG)
            return 0;
    return 1;
}

static void SuperviseMotionFeedback(uint32_t now) {
    // An ESTOP can arrive while sensor transactions are in progress.
    if (command_queue.emergency || command_queue.overflow) {
        if (command_queue.overflow) LatchReceiveFault(command_queue.overflow);
        else {
            system_state = STATE_ESTOP;
            position_control_active = 0;
            memset(move_dirs, 0, sizeof(move_dirs));
            StopArmOutputs();
        }
        return;
    }
    int moving = position_control_active;
    for (int i = 0; i < 6; ++i)
        moving |= move_dirs[i] != 0;
    if ((system_state == STATE_AUTO || system_state == STATE_MANUAL) && moving) {
        if (!FeedbackReady(now)) {
            motion_feedback_hold = 1;
            HoldArmOutputs();
            int hard_error = 0;
            for (int i = 0; i < 5; ++i) hard_error |= tle_state[i].hard_error;
            if (hard_error || !FeedbackWithinAge(now))
                LatchFeedbackFault("Motion feedback unavailable or stale");
            return;
        }
        if (position_control_active && !PositionFeedbackReady(now)) {
            LatchFeedbackFault("Measured joint outside configured limits");
            return;
        }
        if (motion_feedback_hold) {
            // Derivative terms must not include the pause/retry gap.
            prev_motor_angle_L = a1m;
            prev_motor_angle_R = a2m;
            prev_joint_angle_3 = a3j;
            integral_L = integral_R = integral_3 = 0;
        }
    }
    motion_feedback_hold = 0;
}

static void SampleFastFeedback(void) {
    float *raw[5] = {&a1m, &a1j, &a2m, &a2j, &a3j};
    latest_spi_mask = 0;
    int can_configure = !position_control_active && !JC_Moving(&joint_calibration);
    for (int i = 0; i < 6; ++i) can_configure &= move_dirs[i] == 0;
    for (int i = 0; i < 5; ++i) {
        uint32_t now = HAL_GetTick();
        if (!tle_state[i].configured && can_configure &&
            (!tle_state[i].attempted ||
             (uint32_t)(now - tle_state[i].last_config_attempt) >= SENSOR_CONFIG_RETRY_MS)) {
            tle_state[i].attempted = 1;
            tle_state[i].last_config_attempt = now;
            tle_state[i].configured = ConfigureTleSensor(i);
        }
        float value = tle_state[i].configured ?
            Read_TLE5012_Angle(tle_devices[i].spi, GPIOB, tle_devices[i].pin,
                              &last_spi_data[i], &last_spi_safety[i]) : NAN;
        if (!isfinite(value) && feedback_samples[i].successes &&
            (uint32_t)(HAL_GetTick() - feedback_samples[i].timestamp) > FEEDBACK_SPI_MAX_AGE_MS)
            tle_state[i].configured = 0;
        Feedback_Record(&feedback_samples[i], value, HAL_GetTick(), 2);
        if (isfinite(value)) {
            *raw[i] = value;
            latest_spi_mask |= (uint8_t)(1U << i);
        }
    }
}

static void ServiceWristFeedback(uint32_t now) {
    if ((uint32_t)(now - last_wrist_read) < 50U)
        return;
    last_wrist_read = now;
    float angle = Read_LX225_Angle(&huart5, wrist_address.id);
    uint8_t was_locked = wrist_address.locked;
    Feedback_Record(&feedback_samples[SENSOR_WRIST], angle, HAL_GetTick(), 1);
    if (isfinite(angle))
        a4 = angle;
    Wrist_RecordAddress(&wrist_address, isfinite(angle),
                        system_state == STATE_MODES || system_state == STATE_ESTOP);
    if (!was_locked && wrist_address.locked)
        printf("[FB] Wrist ID confirmed: %u\r\n", wrist_address.id);
}

static void ServiceFeedbackDiagnostics(uint32_t now) {
    static uint8_t last_mask = 0xFF;
    static uint8_t last_fault = 0xFF;
    static uint32_t last_print = 0;
    static uint32_t last_ssc_print = 0;
    uint8_t mask = 0;
    for (int i = 0; i < SENSOR_COUNT; ++i)
        if (isfinite(RawFeedbackValue(i, now)))
            mask |= (uint8_t)(1U << i);
    // Mask chatter must not block control or starve the incoming command FIFO.
    if (last_mask == 0xFF || feedback_fault_latched != last_fault ||
        (mask != last_mask && (uint32_t)(now - last_print) >= 100U) ||
        (mask != 0x3F && (uint32_t)(now - last_print) >= 1000U)) {
        printf("[FB] validMask=0x%02X wristId=%u locked=%u wristAge=%lums fault=%u hold=%u\r\n", mask,
               wrist_address.id, wrist_address.locked,
               (unsigned long)(now - feedback_samples[SENSOR_WRIST].timestamp),
               feedback_fault_latched, motion_feedback_hold);
        if ((mask & 0x1F) != 0x1F)
            printf("[FB SPI] data=%04X,%04X,%04X,%04X,%04X safety=%04X,%04X,%04X,%04X,%04X\r\n",
                   last_spi_data[0], last_spi_data[1], last_spi_data[2], last_spi_data[3],
                   last_spi_data[4], last_spi_safety[0], last_spi_safety[1], last_spi_safety[2],
                   last_spi_safety[3], last_spi_safety[4]);
        if ((mask & 0x1F) != 0x1F)
            printf("[FB STAT cached] %04X,%04X,%04X,%04X,%04X\r\n", tle_state[0].status,
                   tle_state[1].status, tle_state[2].status, tle_state[3].status, tle_state[4].status);
        last_mask = mask;
        last_fault = feedback_fault_latched;
        last_print = now;
    }
    // One compact line, at most once per second on trouble; five seconds when
    // healthy. Avoid printing within the clocked transaction/control decision.
    if ((uint32_t)(now - last_ssc_print) >= (mask == 0x3F ? 5000U : 1000U)) {
        int index = SENSOR_ELBOW;
        for (int i = 0; i < 5; ++i)
            if (!(mask & (1U << i))) { index = i; break; }
        const TleFeedbackState *s = &tle_state[index];
        printf("[SSC] sensor=%d transport=%s transfers=%lu good=%lu crc=%lu io=%lu "
               "safety=%lu shift=%lu retries=%lu goodAge=%lums workUs=%lu/%lu\r\n", index,
               index == SENSOR_ELBOW ? "GPIO" : "HAL",
               (unsigned long)s->transfers, (unsigned long)s->good_angles,
               (unsigned long)s->crc_errors, (unsigned long)s->io_errors,
               (unsigned long)s->safety_errors, (unsigned long)s->shifted_frames,
               (unsigned long)s->retries,
               (unsigned long)(now - feedback_samples[index].timestamp),
               (unsigned long)s->last_transfer_us, (unsigned long)s->max_transfer_us);
        last_ssc_print = now;
    }
}

static void LatchReceiveFault(uint8_t flags) {
    int report = !feedback_fault_latched;
    // Close and roll back the session with its real cause, before the generic
    // feedback latch can turn it into a misleading IDLE/OK or feedback error.
    if (joint_calibration.open) CalibrationAbort(JC_LINK_ERROR);
    // Halt outputs before potentially blocking diagnostic transmission.
    LatchFeedbackFault(flags & RX_FAULT_UART ? "Portenta UART receive error" :
                       flags & RX_FAULT_QUEUE_FULL ? "Portenta command FIFO full" :
                       flags & RX_FAULT_RESTART ? "Portenta receiver restart failed" :
                       "Portenta command framing/DMA event error");
    if (report) {
        printf("[RX FAULT] flags=0x%02X uartError=0x%lX rxErrors=%lu queuePeak=%lu\r\n",
               flags, (unsigned long)portenta_uart_error, (unsigned long)portenta_rx_errors,
               (unsigned long)portenta_queue_peak);
        printf("[RX COUNTS] noise=%lu frame=%lu overrun=%lu parity=%lu dma=%lu restarts=%lu restartFail=%lu\r\n",
               (unsigned long)portenta_noise_errors, (unsigned long)portenta_frame_errors,
               (unsigned long)portenta_overruns, (unsigned long)portenta_parity_errors,
               (unsigned long)portenta_dma_errors, (unsigned long)portenta_rx_restarts,
               (unsigned long)portenta_restart_failures);
    }
}

static int StartPortentaReceiver(void) {
    portenta_rx_position = 0;
    // Publish before enabling DMA/interrupts: an immediate error callback must
    // not be overwritten with "running" when the HAL start call returns.
    portenta_rx_running = 1;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, RX_BUFFER_SIZE) != HAL_OK) {
        portenta_rx_running = 0;
        ++portenta_restart_failures;
        command_queue.overflow |= RX_FAULT_RESTART;
    }
    return portenta_rx_running;
}

static void ServicePortentaReceiver(uint32_t now) {
    if (!portenta_rx_running && (uint32_t)(now - portenta_last_restart_ms) >= 100U) {
        portenta_last_restart_ms = now;
        // Main context only, after HAL's UART/DMA error interrupt has unwound.
        // Stop RX before SR/DR reads, which would otherwise race DMA for a byte.
        if (HAL_UART_AbortReceive(&huart6) == HAL_OK) {
            __HAL_UART_CLEAR_OREFLAG(&huart6);
            command_queue.partial_length = 0;
            command_queue.discard = 1; // Resynchronise at the next newline.
            if (StartPortentaReceiver()) ++portenta_rx_restarts;
        } else {
            ++portenta_restart_failures;
            command_queue.overflow |= RX_FAULT_RESTART;
        }
    }
}

static void ProcessQueuedCommands(void) {
    uint32_t started = HAL_GetTick();
    for (int i = 0; i < COMMAND_PROCESS_MAX_LINES &&
         (uint32_t)(HAL_GetTick() - started) < COMMAND_PROCESS_BUDGET_MS &&
         CommandQueue_Pop(&command_queue, process_buffer); ++i)
        ProcessPortentaCommand(process_buffer);
}

static int MotorBusIndex(UART_HandleTypeDef *huart) {
    return huart == &huart1 ? 0 : huart == &huart3 ? 1 :
           huart == &huart4 ? 2 : huart == &huart5 ? 3 : -1;
}

static void ServoBusPinType(UART_HandleTypeDef *huart, int receive) {
    GPIO_TypeDef *ports[4] = {Motor_DOF1_GPIO_Port, Motor_DOF2_GPIO_Port,
                             Motor_DOF3_GPIO_Port, Motor_DOF4_GPIO_Port};
    const uint16_t pins[4] = {Motor_DOF1_Pin, Motor_DOF2_Pin, Motor_DOF3_Pin, Motor_DOF4_Pin};
    int bus = MotorBusIndex(huart);
    if (bus < 0) return;
    // Retain push-pull during TX for the installed level shifter. During RX,
    // alternate-function open-drain releases the shared line (RM0390 25.4.10).
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    if (receive) {
        ports[bus]->BSRR = pins[bus]; // Released/high fallback level, never a driven low.
        ports[bus]->OTYPER |= pins[bus];
    }
    else ports[bus]->OTYPER &= ~(uint32_t)pins[bus];
    __set_PRIMASK(irq_state);
}
static void ServoBusTransmit(UART_HandleTypeDef *huart) {
    huart->Instance->CR1 &= ~USART_CR1_RE;
    ServoBusPinType(huart, 0);
    huart->Instance->CR1 |= USART_CR1_TE;
}
static void ServoBusReceive(UART_HandleTypeDef *huart) {
    // Called only after TC/blocking TX completion or an idle failed submission.
    huart->Instance->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
    ServoBusPinType(huart, 1);
    // Clear old echo/errors before enabling RX; retain a fast reply's first byte.
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->Instance->CR1 |= USART_CR1_RE;
}

static void ServiceDriveDiagnostics(uint32_t now) {
    if (!ENABLE_DRIVE_DIAGNOSTICS) return;
    static uint32_t last_print, submitted[2], completed[2], busy[2], failed[2];
    int active = position_control_active;
    for (int i = 0; i < 6; ++i) active |= move_dirs[i] != 0;
    if (!active || (system_state != STATE_MANUAL && system_state != STATE_AUTO) ||
        (uint32_t)(now - last_print) < 1000U) return;
    last_print = now;
    UART_HandleTypeDef *buses[2] = {&huart1, &huart3};
    for (int i = 0; i < 2; ++i) {
        MotorTxStats *s = &motor_tx_stats[i];
        uint32_t sent = s->submitted, done = s->completed;
        float joint = RawFeedbackValue(i ? SENSOR_JOINT_R : SENSOR_JOINT_L, now);
        joint = Feedback_CalibrateSpi(joint, i ? OFFSET_R : OFFSET_L);
        printf("[DRIVE %c] pos=%u hold=%u goal=%.2f slew=%.2f joint=%.2f motorRaw=%.2f "
               "req=%d tx+%lu done+%lu busy+%lu fail+%lu g=0x%lX status=%d uartErr=0x%lX/%lu\r\n",
               i ? 'R' : 'L', position_control_active, motion_feedback_hold,
               i ? target_R_cal : target_L_cal, i ? slew_R_cal : slew_L_cal, joint,
               RawFeedbackValue(i ? SENSOR_MOTOR_R : SENSOR_MOTOR_L, now), s->requested,
               (unsigned long)(sent-submitted[i]), (unsigned long)(done-completed[i]),
               (unsigned long)(s->busy-busy[i]), (unsigned long)(s->failed-failed[i]),
               (unsigned long)buses[i]->gState, s->last_status,
               (unsigned long)s->last_uart_error, (unsigned long)s->uart_errors);
        submitted[i] = sent; completed[i] = done; busy[i] = s->busy; failed[i] = s->failed;
    }
}

static int ReadServoRegister(UART_HandleTypeDef *huart, uint8_t id, uint8_t command,
                             uint8_t count, uint8_t *data, uint8_t *source_id) {
    int bus = MotorBusIndex(huart);
    if (bus < 0) return 0;
    ServoReadTrace *trace = &servo_read_trace[bus];
    memset(trace, 0, sizeof(*trace));
    if (!count || count > 4U || (id == 254U && command != 14U)) return 0;
    for (int i = 0; i < 32 && __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE); ++i) {
        volatile uint32_t discarded = huart->Instance->DR;
        (void)discarded;
    }
    uint8_t request[6] = {0x55,0x55,id,3,command,0};
    request[5] = (uint8_t)~(uint8_t)(id+3U+command);
    if (!ServoWritePacket(huart, request, sizeof(request))) return 0;
    trace->transmitted = 1;
    uint32_t start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick()-start) < 8U) {
        uint32_t status = huart->Instance->SR;
        trace->errors |= status & (UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
        if (status & UART_FLAG_RXNE) {
            uint8_t byte = (uint8_t)huart->Instance->DR;
            if (trace->count == sizeof(trace->bytes)) {
                memmove(trace->bytes, trace->bytes+1, sizeof(trace->bytes)-1); --trace->count;
            }
            trace->bytes[trace->count++] = byte;
            if (!trace->errors && Servo_ParseReply(trace->bytes,trace->count,id,command,
                                                   count,data,source_id)) return 1;
        } else if (trace->errors) {
            __HAL_UART_CLEAR_OREFLAG(huart);
        }
    }
    return 0;
}

static void ServiceDriveStatus(uint32_t now) {
    static uint32_t last_poll;
    static uint8_t bus;
    // Read-only diagnostics while stopped: no motion timing changes and no
    // automatic torque-enable, protection reset or persistent register writes.
    if ((system_state != STATE_MODES && system_state != STATE_ESTOP) ||
        position_control_active || (uint32_t)(now-last_poll) < 100U) return;
    last_poll = now;
    UART_HandleTypeDef *buses[3] = {&huart1,&huart3,&huart4};
    const char *names[3] = {"L","R","J3"};
    const uint8_t commands[7] = {27,26,32,30,23,25,28}, sizes[7] = {2,1,1,4,4,1,2};
    DriveStatusReader *reader = &drive_status_reader[bus];
    uint8_t reg = reader->next_register;
    uint8_t command = reader->locked ? commands[reg] : 14;
    uint8_t size = reader->locked ? sizes[reg] : 1;
    uint8_t data[4] = {0}, source;
    if (!ReadServoRegister(buses[bus],reader->locked ? reader->id : 254,
                           command,size,data,&source)) {
        ++reader->failures;
        printf("[SERVO %s] no valid reply cmd=%u id=%u failures=%lu\r\n",
               names[bus],command,reader->locked ? reader->id : 254,
               (unsigned long)reader->failures);
        ServoReadTrace *trace = &servo_read_trace[bus];
        printf("[SERVO RX %s] tx=%u bytes=%u flags=0x%lX raw=", names[bus],
               trace->transmitted, trace->count, (unsigned long)trace->errors);
        for (int i = 0; i < trace->count; ++i) printf("%02X",trace->bytes[i]);
        printf("\r\n");
        if (reader->locked) reader->next_register = (reg+1U)%7U;
    } else {
        ++reader->successes;
        reader->id = source;
        if (!reader->locked) {
            reader->locked = 1;
            printf("[SERVO %s] ID=%u confirmed\r\n",names[bus],source);
        } else {
            uint16_t first = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
            if (command == 27) printf("[SERVO %s] id=%u voltage=%umV\r\n",names[bus],source,first);
            else if (command == 26) printf("[SERVO %s] id=%u temperature=%uC\r\n",names[bus],source,data[0]);
            else if (command == 32) printf("[SERVO %s] id=%u torqueEnable=%u\r\n",names[bus],source,data[0]);
            else if (command == 30) {
                int16_t speed = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
                printf("[SERVO %s] id=%u mode=%u storedSpeed=%d\r\n",names[bus],source,data[0],speed);
            } else if (command == 23) {
                uint16_t second = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
                printf("[SERVO %s] id=%u voltageLimits=%u..%umV\r\n",names[bus],source,first,second);
            } else if (command == 28)
                printf("[SERVO %s] id=%u internalPosRaw=%d\r\n",names[bus],source,(int16_t)first);
            else printf("[SERVO %s] id=%u temperatureLimit=%uC\r\n",names[bus],source,data[0]);
            reader->next_register = (reg+1U)%7U;
        }
    }
    bus = (bus+1U)%3U;
}

#include "JointCalibrationRuntime.h"
#include "JointDiagnosticRuntime.h"

static void ProcessPortentaCommand(char *line) {
    if (command_queue.emergency || command_queue.overflow)
        return;
    if (CalibrationHandleCommand(line)) return;
    if (JD_HandleCommand(line)) return;
    if (joint_calibration.open) {
        if (!strcmp(line, "S:ESTOP") || !strcmp(line, "S:MODES")) CalibrationAbort(JC_ABORTED);
        else if (strcmp(line, "S:REQ_LIMITS")) return;
    }
    if (calibration_source == JC_STORAGE_CORRUPT &&
        (!strcmp(line, "S:AUTO") || !strncmp(line, "P:", 2))) {
        calibration_preview = 1; CalibrationReply(0, JC_STORAGE_CORRUPT); return;
    }
    // 1. STATE COMMANDS (S:)
    if (strncmp(line, "S:", 2) == 0) {
        if (strstr(line, "ESTOP")) {
            system_state = STATE_ESTOP;
            position_control_active = 0;
            memset(move_dirs, 0, sizeof(move_dirs));
            // Halt everything
            Hiwonder_SetMotorSpeed_IT(&huart1, 254, 0);
            Hiwonder_SetMotorSpeed_IT(&huart3, 254, 0);
            Hiwonder_SetMotorSpeed_IT(&huart4, 254, 0);
            Hiwonder_SetMotorSpeed_IT(&huart5, 254, 0);
        } else if (strstr(line, "MODES")) {
            system_state = STATE_MODES;
            position_control_active = 0;
            feedback_fault_latched = 0;
            integral_L = integral_R = integral_3 = 0;
            memset(move_dirs, 0, sizeof(move_dirs));
            Hiwonder_SetMotorSpeed_IT(&huart1, 254, 0);
            Hiwonder_SetMotorSpeed_IT(&huart3, 254, 0);
            Hiwonder_SetMotorSpeed_IT(&huart4, 254, 0);
            Hiwonder_SetMotorSpeed_IT(&huart5, 254, 0);
        } else if (strstr(line, "MANUAL")) {
            if (feedback_fault_latched)
                return;
            system_state = STATE_MANUAL;
            position_control_active = 0;
            memset(move_dirs, 0, sizeof(move_dirs));
            StopArmOutputs();
        } else if (strstr(line, "AUTO")) {
            if (feedback_fault_latched)
                return;
            system_state = STATE_AUTO;
            position_control_active = 0;
            memset(move_dirs, 0, sizeof(move_dirs));
            StopArmOutputs();
        } else if (strstr(line, "REQ_LIMITS")) {
            // Send Limits packet
        	limits_reply_pending = 1;
        }
    }

    // 2. TARGET MODE (T:)
    else if (strncmp(line, "T:", 2) == 0) {
        int mode;
        if (sscanf(line, "T:%d", &mode) == 1 && (mode == 0 || mode == 1))
            target_mode = mode;
    }

    // 3. GLOBAL VELOCITY (V:)
    else if (strncmp(line, "V:", 2) == 0) {
        int velocity;
        if (sscanf(line, "V:%d", &velocity) == 1 && velocity >= 0 && velocity <= 1000)
            global_velocity = velocity;
    }

    // 4. MOVE COMMAND (M:)
    else if (strncmp(line, "M:", 2) == 0) {
        if (system_state == STATE_MANUAL) {
            int m1 = 0, m2 = 0, m3 = 0, m4 = 0, m5 = 0, m6 = 0;
            if (sscanf(line, "M:%d,%d,%d,%d,%d,%d", &m1, &m2, &m3, &m4, &m5, &m6) == 6) {
                if (m1 < -1 || m1 > 1 || m2 < -1 || m2 > 1 || m3 < -1 || m3 > 1 || m4 < -1 ||
                    m4 > 1 || m5 < -1 || m5 > 1 || m6 < -1 || m6 > 1)
                    return;
                if ((m1 || m2 || m3 || m4 || m5 || m6) && !FeedbackWithinAge(HAL_GetTick())) {
                    LatchFeedbackFault("Manual motion rejected: feedback not ready");
                    return;
                }
                move_dirs[0] = m1;
                move_dirs[1] = m2;
                move_dirs[2] = m3;
                move_dirs[3] = m4;
                move_dirs[4] = m5;
                move_dirs[5] = m6;

                // Cancel position control if joystick is moved
                if (m1 != 0 || m2 != 0 || m3 != 0 || m4 != 0 || m5 != 0 || m6 != 0) {
                    position_control_active = 0;
                }
            }
        }
    }

    // 5. POSITION COMMAND (P:)
    else if (strncmp(line, "P:", 2) == 0) {
        if (system_state == STATE_MANUAL || system_state == STATE_AUTO) {
            float p_roll = 0, p_pitch = 0, p_base = 0, p_dof4 = 0, p_dof5 = 0;
            int parsed =
                sscanf(line, "P:%f,%f,%f,%f,%f", &p_roll, &p_pitch, &p_base, &p_dof4, &p_dof5);
            if (parsed == 5) {
                // Reacquire a snapshot if a queued point arrives during a transient SPI failure.
                if (!FeedbackReady(HAL_GetTick()) && !feedback_fault_latched)
                    SampleFastFeedback();
                if (!isfinite(p_roll) || !isfinite(p_pitch) || !isfinite(p_base) ||
                    !isfinite(p_dof4) || !isfinite(p_dof5) ||
                    !PositionFeedbackReady(HAL_GetTick())) {
                    LatchFeedbackFault("Position rejected: feedback/angles not valid");
                    return;
                }
                // Apply strict hardware limits
                if (p_roll < LIMIT_ROLL_MIN)
                    p_roll = LIMIT_ROLL_MIN;
                if (p_roll > LIMIT_ROLL_MAX)
                    p_roll = LIMIT_ROLL_MAX;
                if (p_pitch < LIMIT_PITCH_MIN)
                    p_pitch = LIMIT_PITCH_MIN;
                if (p_pitch > LIMIT_PITCH_MAX)
                    p_pitch = LIMIT_PITCH_MAX;
                if (p_base < LIMIT_DOF3_MIN)
                    p_base = LIMIT_DOF3_MIN;
                if (p_base > LIMIT_DOF3_MAX)
                    p_base = LIMIT_DOF3_MAX;
                if (p_dof4 < LIMIT_DOF4_MIN)
                    p_dof4 = LIMIT_DOF4_MIN;
                if (p_dof4 > LIMIT_DOF4_MAX)
                    p_dof4 = LIMIT_DOF4_MAX;
                if (p_dof5 < LIMIT_DOF5_MIN)
                    p_dof5 = LIMIT_DOF5_MIN;
                if (p_dof5 > LIMIT_DOF5_MAX)
                    p_dof5 = LIMIT_DOF5_MAX;

                cmd_roll = p_roll;
                cmd_pitch = p_pitch;
                cmd_base = p_base;
                cmd_dof4 = p_dof4;
                cmd_dof5 = p_dof5;

                // Inverse kinematics
                target_L_cal = p_roll - p_pitch;
                target_R_cal = p_roll + p_pitch;
                target_3_cal = p_base;
                target_4_cal = p_dof4;
                target_5_cal = p_dof5;

                // Apply offsets
                target_4_raw = target_4_cal - OFFSET_4; // LX-225 is 0-240 deg, no wrap!
                target_5_raw = target_5_cal - OFFSET_5;

                // Set LX-225 (DOF4) - Move smoothly matching 50Hz IK updates (20ms)
                // Command the same confirmed device whose position we monitor.
                if (!Hiwonder_SetServoPosition(&huart5, wrist_address.id, target_4_raw, 20)) {
                    if (command_queue.emergency || command_queue.overflow) return;
                    LatchFeedbackFault("Wrist position transmission failed");
                    return;
                }
                if (command_queue.emergency || command_queue.overflow)
                    return;

                // Set MG90S (DOF5)
                if (target_5_raw < 0.0f)
                    target_5_raw = 0.0f;
                if (target_5_raw > 180.0f)
                    target_5_raw = 180.0f;
                virtual_angle_5 = target_5_raw; // Sync virtual angle for velocity tracking
                pwm_pos_5 = (uint32_t)(500.0f + (target_5_raw / 180.0f) * 2000.0f);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_pos_5);

                // Initialize angle history and slew limiters
                prev_motor_angle_L = a1m;
                prev_motor_angle_R = a2m;
                prev_joint_angle_3 = a3j;

                if (position_control_active == 0) {
                    integral_L = integral_R = integral_3 = 0;
                    slew_L_cal = CalibratedLeftGear();
                    slew_R_cal = CalibratedRightGear();
                    slew_3_cal = CalibratedElbow();
                }

                position_control_active = 1;
                if (joint_diagnostic.active) {
                    ++joint_diagnostic.command;
                    joint_diagnostic.command_ms = HAL_GetTick();
                }
            }
        }
    }

    // 6. GRIPPER COMMAND (G:)
    else if (strncmp(line, "G:", 2) == 0) {
        if (system_state == STATE_MANUAL || system_state == STATE_AUTO) {
            float p_gripper = 0.0f;
            if (sscanf(line, "G:%f", &p_gripper) == 1 && isfinite(p_gripper)) {
                if (p_gripper < LIMIT_GRIPPER_MIN)
                    p_gripper = LIMIT_GRIPPER_MIN;
                if (p_gripper > LIMIT_GRIPPER_MAX)
                    p_gripper = LIMIT_GRIPPER_MAX;

                cmd_gripper = p_gripper;
                target_gripper_cal = p_gripper;
                target_gripper_raw = target_gripper_cal;

                if (target_gripper_raw < 0.0f)
                    target_gripper_raw = 0.0f;
                if (target_gripper_raw > 180.0f)
                    target_gripper_raw = 180.0f;

                virtual_gripper = target_gripper_raw;
                pwm_pos_6 = (uint32_t)(500.0f + (target_gripper_raw / 180.0f) * 2000.0f);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_pos_6);
            }
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CalibrationBootLoad();
  ServoBusReceive(&huart1);
  ServoBusReceive(&huart3);
  ServoBusReceive(&huart4);
  ServoBusReceive(&huart5);
  printf("=========================================\r\n");
  printf("NUCLEO INICIANDO (TEST DE RECEPCION UART)\r\n");
  printf("=========================================\r\n");
  printf("[SYSTEM] Nucleo starts in ESTOP; re-enter a mode on Portenta after reset\r\n");
  printf("[SSC] Elbow uses GPIO clock on PC10/PB0; nominal maximum 500kHz; CRC required\r\n");

  // Iniciar la escucha por DMA en el USART6 (conectado a la Portenta)
  hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
  if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK) Error_Handler();
  StartPortentaReceiver();
  
  // Arrancar los canales PWM (TIM2 CH1 y CH2) para los servos tradicionales
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  
  // Desbloquear los límites de ángulo del LX-225 (0 a 1000) por si la EEPROM los tiene restringidos
  HAL_Delay(50);
  Hiwonder_UnlockAngleLimits(&huart5, 254);
  HAL_Delay(50);
  
  // Configuration/readback/CRC_PAR repair is serviced while stopped,
  // including when sensor power returns independently of MCU power.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_control_read = 0;
  uint32_t last_telemetry_read = 0;
  
  while (1)
  {
      // Urgent ESTOP/receive failures take priority over queued motion commands.
      if (command_queue.emergency || command_queue.overflow) {
          uint32_t irq_state = __get_PRIMASK();
          __disable_irq();
          uint8_t overflow = command_queue.overflow;
          memset(&command_queue, 0, sizeof(command_queue));
          command_queue.discard = overflow != 0; // Never parse a damaged line's suffix.
          __set_PRIMASK(irq_state);
          if (overflow) LatchReceiveFault(overflow);
          else {
              system_state = STATE_ESTOP;
              position_control_active = 0;
              memset(move_dirs, 0, sizeof(move_dirs));
              StopArmOutputs();
          }
      }
      ServicePortentaReceiver(HAL_GetTick());
      ServiceLimitsReply();
      if (joint_calibration.open && system_state != STATE_MANUAL) CalibrationAbort(JC_ABORTED);
      // Poll before starting motor UART transmissions in this loop.
      ServiceWristFeedback(HAL_GetTick());
      uint32_t current_tick = HAL_GetTick();

      // --- 100 Hz CONTROL LOOP (10ms) ---
      if (current_tick - last_control_read >= 10) 
      {
          joint_diagnostic.control_dt_ms = current_tick-last_control_read;
          last_control_read = current_tick;
          // Sampling continues in ESTOP/MODES; telemetry never starts from zero caches.
          SampleFastFeedback();
          current_tick = HAL_GetTick();
          SuperviseMotionFeedback(current_tick);
          
          if (joint_calibration.open && system_state == STATE_MANUAL) {
              CalibrationControlTick(current_tick);
          }
          else if (system_state == STATE_ESTOP || system_state == STATE_MODES) {
              // Safety Halt
              Hiwonder_SetMotorSpeed_IT(&huart1, 254, 0);
              Hiwonder_SetMotorSpeed_IT(&huart3, 254, 0);
              Hiwonder_SetMotorSpeed_IT(&huart4, 254, 0);
              Hiwonder_SetMotorSpeed_IT(&huart5, 254, 0);
          }
          else if ((system_state == STATE_MANUAL || system_state == STATE_AUTO) &&
                   !motion_feedback_hold) {
              // --- CLOSED-LOOP POSITION CONTROLLER ---
                if (position_control_active) {
                    // --- TRAJECTORY GENERATOR (SLEW RATE LIMITER) ---
                    // Slew L
                    float err_slew_L = target_L_cal - slew_L_cal;
                    if (err_slew_L > max_slew_rate_LR) slew_L_cal += max_slew_rate_LR;
                    else if (err_slew_L < -max_slew_rate_LR) slew_L_cal -= max_slew_rate_LR;
                    else slew_L_cal = target_L_cal;
                    
                    // Slew R
                    float err_slew_R = target_R_cal - slew_R_cal;
                    if (err_slew_R > max_slew_rate_LR) slew_R_cal += max_slew_rate_LR;
                    else if (err_slew_R < -max_slew_rate_LR) slew_R_cal -= max_slew_rate_LR;
                    else slew_R_cal = target_R_cal;
                    
                    // Slew 3
                    float err_slew_3 = target_3_cal - slew_3_cal;
                    if (err_slew_3 > max_slew_rate_3) slew_3_cal += max_slew_rate_3;
                    else if (err_slew_3 < -max_slew_rate_3) slew_3_cal -= max_slew_rate_3;
                    else slew_3_cal = target_3_cal;

                    // Compute raw targets from slewed calibrated targets
                    target_L_raw = normalize_angle(slew_L_cal - OFFSET_L);
                    target_R_raw = normalize_angle(slew_R_cal - OFFSET_R);
                    target_3_raw = normalize_angle(slew_3_cal - OFFSET_BASE);

                    float err_L = angular_error(target_L_raw, a1j);
                  float err_R = angular_error(target_R_raw, a2j);
                  float err_3 = angular_error(target_3_raw, a3j);
                  
                  float omega_L = -angular_error(a1m, prev_motor_angle_L);
                  float omega_R = -angular_error(a2m, prev_motor_angle_R);
                  float omega_3 = angular_error(a3j, prev_joint_angle_3);
                  
                  prev_motor_angle_L = a1m;
                  prev_motor_angle_R = a2m;
                  prev_joint_angle_3 = a3j;
                  
                  // Integral accumulation with anti-windup and deadband reset
                  if (err_L >= -deadband_deg && err_L <= deadband_deg) {
                      integral_L = 0.0f; // Reset if within deadband to prevent hunting
                  } else {
                      integral_L += err_L; 
                  }
                  float i_term_L = Ki_pos_L * integral_L;
                  if (i_term_L > integral_limit) { i_term_L = integral_limit; integral_L = integral_limit / Ki_pos_L; }
                  else if (i_term_L < -integral_limit) { i_term_L = -integral_limit; integral_L = -integral_limit / Ki_pos_L; }
                  
                  if (err_R >= -deadband_deg && err_R <= deadband_deg) {
                      integral_R = 0.0f;
                  } else {
                      integral_R += err_R;
                  }
                  float i_term_R = Ki_pos_R * integral_R;
                  if (i_term_R > integral_limit) { i_term_R = integral_limit; integral_R = integral_limit / Ki_pos_R; }
                  else if (i_term_R < -integral_limit) { i_term_R = -integral_limit; integral_R = -integral_limit / Ki_pos_R; }

                  if (err_3 >= -deadband_deg && err_3 <= deadband_deg) {
                      integral_3 = 0.0f;
                  } else {
                      integral_3 += err_3;
                  }
                  float i_term_3 = Ki_pos_3 * integral_3;
                  if (i_term_3 > integral_limit) { i_term_3 = integral_limit; integral_3 = integral_limit / Ki_pos_3; }
                  else if (i_term_3 < -integral_limit) { i_term_3 = -integral_limit; integral_3 = -integral_limit / Ki_pos_3; }

                  // --- Gravity Compensation Math for DOF 3 ---
                  float current_L_cal = CalibratedLeftGear();
                  float current_R_cal = CalibratedRightGear();
                  float current_3_cal = CalibratedElbow();

                  float theta_roll  = (current_R_cal + current_L_cal) / 2.0f;
                  float theta_pitch = (current_R_cal - current_L_cal) / 2.0f;
                  float theta_dof3  = current_3_cal;

                    // Applying user requested offsets: A = -theta_pitch - 90, C = theta_dof3 + 90
                    float A_deg = -theta_pitch - 90.0f;
                    float B_deg = theta_roll;
                    float C_deg = theta_dof3 + 90.0f;
                    
                    float A_rad = A_deg * (3.14159265f / 180.0f);
                    float B_rad = B_deg * (3.14159265f / 180.0f);
                    float C_rad = C_deg * (3.14159265f / 180.0f);
  
                    float cA = cosf(A_rad), sA = sinf(A_rad);
                    float cB = cosf(B_rad), sB = sinf(B_rad);
                    float cC = cosf(C_rad), sC = sinf(C_rad);
  
                    // Column 1 of roty(A)*rotx(B)*roty(C)
                    float r11 = cA * cC - sA * cB * sC;
                    float r21 = sB * sC;
                  
                  // Magnitude of horizontal projection of the arm
                  float u_gravity = Kg_pos_3 * sqrtf(r11*r11 + r21*r21);
                  // -------------------------------------------
                    
                    // --- BEVEL GEAR GRAVITY COMPENSATION ---
                    float A_term = cosf((-90.0f - theta_pitch) * (3.14159265f / 180.0f));
                    float B_term = cosf((90.0f + theta_dof3) * (3.14159265f / 180.0f));
                    float C_term = sinf((-90.0f - theta_pitch) * (3.14159265f / 180.0f)) * 
                                   cosf(theta_roll * (3.14159265f / 180.0f)) * 
                                   sinf((90.0f + theta_dof3) * (3.14159265f / 180.0f));
                    float u_gravity_LR = Kg_pos_LR * (A_term * (B_term + 2.0f + P_weight) - C_term);
                    // ---------------------------------------
  
                  float uf_L = (err_L > deadband_deg) ? u_fric_val_L : ((err_L < -deadband_deg) ? -u_fric_val_L : 0.0f);
                  float uf_R = (err_R > deadband_deg) ? u_fric_val_R : ((err_R < -deadband_deg) ? -u_fric_val_R : 0.0f);
                  float uf_3 = (err_3 > deadband_deg) ? u_fric_val_3 : ((err_3 < -deadband_deg) ? -u_fric_val_3 : 0.0f);
                  
                  float u_L = Kp_pos_L * err_L + i_term_L - Kd_pos_L * omega_L + uf_L - u_gravity_LR;
                  float u_R = Kp_pos_R * err_R + i_term_R - Kd_pos_R * omega_R + uf_R + u_gravity_LR;
                  // Add u_gravity as a feed-forward term
                  float u_3 = -(Kp_pos_3 * err_3 + i_term_3 - Kd_pos_3 * omega_3 + uf_3 + u_gravity);
                  
                  if (err_L >= -deadband_deg && err_L <= deadband_deg) u_L = 0.0f;
                  if (err_R >= -deadband_deg && err_R <= deadband_deg) u_R = 0.0f;
                  if (err_3 >= -deadband_deg && err_3 <= deadband_deg) u_3 = 0.0f;
                  
                  if (u_L > (float)POS_CTRL_MAX_OUTPUT) u_L = (float)POS_CTRL_MAX_OUTPUT;
                  if (u_L < -(float)POS_CTRL_MAX_OUTPUT) u_L = -(float)POS_CTRL_MAX_OUTPUT;
                  if (u_R > (float)POS_CTRL_MAX_OUTPUT) u_R = (float)POS_CTRL_MAX_OUTPUT;
                  if (u_R < -(float)POS_CTRL_MAX_OUTPUT) u_R = -(float)POS_CTRL_MAX_OUTPUT;
                  if (u_3 > (float)POS_CTRL_MAX_OUTPUT) u_3 = (float)POS_CTRL_MAX_OUTPUT;
                  if (u_3 < -(float)POS_CTRL_MAX_OUTPUT) u_3 = -(float)POS_CTRL_MAX_OUTPUT;
                  
                  Hiwonder_SetMotorSpeed_IT(&huart1, 254, (int16_t)u_L);
                  Hiwonder_SetMotorSpeed_IT(&huart3, 254, (int16_t)u_R);
                  Hiwonder_SetMotorSpeed_IT(&huart4, 254, (int16_t)u_3);
              } 
                // --- MANUAL VELOCITY CONTROL ---
                else if (system_state == STATE_MANUAL) {
                    if (target_mode == 0) { // Joint Space
                        // MANUAL_0 retains its limit bypass, but feedback must be valid/fresh.
                        
                        int speed1 = move_dirs[0] * global_velocity;
                        int speed2 = move_dirs[1] * global_velocity;
                        int speed3 = move_dirs[2] * global_velocity;
                        int speed4 = move_dirs[3] * (int)(global_velocity / 3.5f);
                        
                        Hiwonder_SetMotorSpeed_IT(&huart1, 254, speed1);
                        Hiwonder_SetMotorSpeed_IT(&huart3, 254, speed2);
                        Hiwonder_SetMotorSpeed_IT(&huart4, 254, speed3);
                        Hiwonder_SetMotorSpeed_IT(&huart5, 254, speed4);
                        
                        // Incremental Velocity Emulation for MG90S (Wrist - DOF5)
                        if (move_dirs[4] != 0) {
                            float raw_5_min = LIMIT_DOF5_MIN - OFFSET_5;
                            float raw_5_max = LIMIT_DOF5_MAX - OFFSET_5;
                            virtual_angle_5 += move_dirs[4] * (global_velocity * 0.002f); 
                            if (virtual_angle_5 < raw_5_min) virtual_angle_5 = raw_5_min;
                            if (virtual_angle_5 > raw_5_max) virtual_angle_5 = raw_5_max;
                            pwm_pos_5 = (uint32_t)(500.0f + (virtual_angle_5 / 180.0f) * 2000.0f);
                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_pos_5);
                        }
                        
                        // Incremental Velocity Emulation for MG90S (Gripper)
                        if (move_dirs[5] != 0) {
                            virtual_gripper += move_dirs[5] * (global_velocity * 0.002f); 
                            if (virtual_gripper < LIMIT_GRIPPER_MIN) virtual_gripper = LIMIT_GRIPPER_MIN;
                            if (virtual_gripper > LIMIT_GRIPPER_MAX) virtual_gripper = LIMIT_GRIPPER_MAX;
                            pwm_pos_6 = (uint32_t)(500.0f + (virtual_gripper / 180.0f) * 2000.0f);
                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_pos_6);
                        }
                    } else {
                      // Cartesian space placeholder
                  }
              }
          }
      }

      JD_Sample(HAL_GetTick(),0);
      JD_Service(HAL_GetTick());
      // --- 20 Hz TELEMETRY LOOP (50ms) ---
      if (current_tick - last_telemetry_read >= 50) 
      {
          last_telemetry_read = current_tick;
          a5 = ((float)pwm_pos_5 - 500.0f) * (180.0f / 2000.0f);
          
            if (huart6.gState == HAL_UART_STATE_READY) {
                float q[6];
                GetFeedbackAngles(HAL_GetTick(), q);
                // Pack standard telemetry (A:) - Always sent
                int len = snprintf(telemetry_buf, sizeof(telemetry_buf), 
                                   "A:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                                   q[0], q[1], q[2], q[3], q[4], q[5]);
                                   
                // Pack raw sensor telemetry (R:) - Sent only in MANUAL target 0
                if (system_state == STATE_MANUAL && target_mode == 0) {
                    len += snprintf(telemetry_buf + len, sizeof(telemetry_buf) - len, 
                                    "R:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                                    RawFeedbackValue(SENSOR_MOTOR_L, HAL_GetTick()), RawFeedbackValue(SENSOR_JOINT_L, HAL_GetTick()), RawFeedbackValue(SENSOR_MOTOR_R, HAL_GetTick()), RawFeedbackValue(SENSOR_JOINT_R, HAL_GetTick()), RawFeedbackValue(SENSOR_ELBOW, HAL_GetTick()), RawFeedbackValue(SENSOR_WRIST, HAL_GetTick()), a5);
                }
                
                // Transmision No-Bloqueante hacia Portenta (ahorra ~15ms)
                HAL_UART_Transmit_IT(&huart6, (uint8_t*)telemetry_buf, len);
            }
      }

      // Drain bursts with both a line and elapsed-time budget.
      ProcessQueuedCommands();
      CalibrationService();
      if (!joint_calibration.open) {
          ServiceFeedbackDiagnostics(HAL_GetTick());
          ServiceDriveDiagnostics(HAL_GetTick());
          ServiceDriveStatus(HAL_GetTick());
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_JointAxle_DOF3_Pin|CS_MotorAxle_DOF3_Pin|CS_JointAxle_DOF2_Pin|CS_MotorAxle_DOF2_Pin
                          |CS_JointAxle_DOF1_Pin|CS_MotorAxle_DOF1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_JointAxle_DOF3_Pin CS_MotorAxle_DOF3_Pin CS_JointAxle_DOF2_Pin CS_MotorAxle_DOF2_Pin
                           CS_JointAxle_DOF1_Pin CS_MotorAxle_DOF1_Pin */
  GPIO_InitStruct.Pin = CS_JointAxle_DOF3_Pin|CS_MotorAxle_DOF3_Pin|CS_JointAxle_DOF2_Pin|CS_MotorAxle_DOF2_Pin
                          |CS_JointAxle_DOF1_Pin|CS_MotorAxle_DOF1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// =========================================================================
// ANGULAR MATH HELPERS (for position control)
// =========================================================================

// Normalize angle to [0, 360)
float normalize_angle(float angle)
{
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// Returns shortest signed angular difference in degrees [-180, +180]
float angular_error(float target, float current)
{
    float err = target - current;
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

// =========================================================================
// SENSOR DE ÁNGULO TLE5012B (HALF-DUPLEX SPI)
// =========================================================================

uint8_t Calc_TLE5012_CRC8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t)((crc << 1) ^ 0x1D);
            } else {
                crc <<= 1;
            }
        }
    }
    return (~crc) & 0xFF;
}

enum { TLE_TRANSFER_OK, TLE_TRANSFER_IO, TLE_TRANSFER_CRC };
static uint8_t tle_transfer_result;

static int TleDeviceIndex(SPI_HandleTypeDef *spi, uint16_t pin) {
    for (int i = 0; i < 5; ++i)
        if (tle_devices[i].spi == spi && tle_devices[i].pin == pin) return i;
    return -1;
}

// SPI3's automatic 1-line receive clock is bypassed on the existing pins.
// Re-establish idle GPIO state with CS high for EVERY attempt, so a failed
// frame cannot leave a partially clocked peripheral transaction behind.
static uint32_t elbow_ssc_start, elbow_ssc_half_cycles;
static void ElbowSscSelect(int active) {
    HAL_GPIO_WritePin(CS_JointAxle_DOF3_GPIO_Port, CS_JointAxle_DOF3_Pin,
                      active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static void ElbowSscClock(int high) {
    HAL_GPIO_WritePin(DOF3_Sensor_SCK_GPIO_Port, DOF3_Sensor_SCK_Pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void ElbowSscDrive(int high) {
    HAL_GPIO_WritePin(DOF3_Sensor_MOSI_GPIO_Port, DOF3_Sensor_MOSI_Pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void ElbowSscOutput(int enabled) {
    GPIO_InitTypeDef init = {0};
    init.Pin = DOF3_Sensor_MOSI_Pin;
    init.Mode = enabled ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    // HAL updates shared GPIO registers by read/modify/write. Protect only
    // this pin-mode change from servo UART callbacks on other port-B pins.
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    HAL_GPIO_Init(DOF3_Sensor_MOSI_GPIO_Port, &init);
    __set_PRIMASK(irq_state);
}
static int ElbowSscRead(void) {
    return HAL_GPIO_ReadPin(DOF3_Sensor_MOSI_GPIO_Port, DOF3_Sensor_MOSI_Pin) == GPIO_PIN_SET;
}
static int ElbowSscDelay(void) {
    uint32_t start = DWT->CYCCNT;
    // Bounded even if the debug cycle counter is unavailable/stopped.
    for (uint32_t budget = elbow_ssc_half_cycles * 4U + 64U; budget; --budget)
        if ((uint32_t)(DWT->CYCCNT - start) >= elbow_ssc_half_cycles) return 1;
    return 0;
}
static int ElbowSscCancelled(void) {
    return command_queue.emergency || command_queue.overflow ||
        (uint32_t)(DWT->CYCCNT - elbow_ssc_start) >=
            (SystemCoreClock / 1000U) * SENSOR_SPI_TIMEOUT_MS;
}
static int ElbowSscTransfer(const uint16_t *tx, unsigned tx_count, uint16_t *rx, unsigned rx_count) {
    __HAL_SPI_DISABLE(&hspi3);
    // CS must be inactive before either pin is remuxed. The SPI peripheral
    // stays disabled; only software generates clocks on this bus thereafter.
    ElbowSscSelect(0);
    GPIO_InitTypeDef init = {0};
    init.Pin = DOF3_Sensor_SCK_Pin;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    ElbowSscClock(0);
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    HAL_GPIO_Init(DOF3_Sensor_SCK_GPIO_Port, &init);
    __set_PRIMASK(irq_state);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    elbow_ssc_half_cycles = (SystemCoreClock + 999999U) / 1000000U;
    if (!elbow_ssc_half_cycles) elbow_ssc_half_cycles = 1;
    elbow_ssc_start = DWT->CYCCNT;
    const TleSscGpioIo io = {ElbowSscSelect, ElbowSscClock, ElbowSscDrive,
        ElbowSscOutput, ElbowSscRead, ElbowSscDelay, ElbowSscCancelled};
    int ok = TleSscGpioTransfer(&io, tx, tx_count, rx, rx_count);
    uint32_t cycles_per_us = elbow_ssc_half_cycles;
    TleFeedbackState *state = &tle_state[SENSOR_ELBOW];
    state->last_transfer_us = (uint32_t)(DWT->CYCCNT - elbow_ssc_start) / cycles_per_us;
    if (state->last_transfer_us > state->max_transfer_us)
        state->max_transfer_us = state->last_transfer_us;
    return ok;
}

static void RecordTleTransfer(SPI_HandleTypeDef *spi, uint16_t pin, int transport_ok,
                               int crc_ok) {
    int index = TleDeviceIndex(spi, pin);
    tle_transfer_result = !transport_ok ? TLE_TRANSFER_IO : crc_ok ? TLE_TRANSFER_OK : TLE_TRANSFER_CRC;
    if (index < 0) return;
    ++tle_state[index].transfers;
    if (!transport_ok) ++tle_state[index].io_errors;
    else if (!crc_ok) ++tle_state[index].crc_errors;
}

static int ReadTleRegisters(SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio, uint16_t pin,
                            uint16_t command, uint16_t *words, uint16_t count, uint16_t *safety) {
    if (!count || count > 8U) return 0;
    uint16_t response[9] = {0};
    if (spi == &hspi3 && gpio == GPIOB && pin == CS_JointAxle_DOF3_Pin) {
        int ok = ElbowSscTransfer(&command, 1, response, count + 1U);
        memcpy(words, response, count * sizeof(uint16_t));
        if (safety) *safety = response[count];
        RecordTleTransfer(spi, pin, ok,
            Feedback_TleBlockCrcValid(command, response, count, response[count]));
        return tle_transfer_result == TLE_TRANSFER_OK;
    }
    HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
    __HAL_SPI_DISABLE(spi);
    __HAL_SPI_CLEAR_OVRFLAG(spi);
    spi->Instance->CR1 |= SPI_CR1_BIDIOE;
    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, (uint8_t *)&command, 1, SENSOR_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        __HAL_SPI_DISABLE(spi);
        spi->Instance->CR1 &= ~SPI_CR1_BIDIOE;
        for (int i = 0; i < 4 && __HAL_SPI_GET_FLAG(spi, SPI_FLAG_RXNE); ++i) {
            volatile uint16_t dummy = spi->Instance->DR;
            (void)dummy;
        }
        status = HAL_SPI_Receive(spi, (uint8_t *)response, count + 1U, SENSOR_SPI_TIMEOUT_MS);
    }
    __HAL_SPI_DISABLE(spi);
    HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
    memcpy(words, response, count * sizeof(uint16_t));
    if (safety) *safety = response[count];
    RecordTleTransfer(spi, pin, status == HAL_OK,
        Feedback_TleBlockCrcValid(command, response, count, response[count]));
    return tle_transfer_result == TLE_TRANSFER_OK;
}

static int ReadTleWord(SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio, uint16_t pin,
                       uint16_t command, uint16_t *data, uint16_t *safety) {
    return ReadTleRegisters(spi, gpio, pin, command, data, 1, safety);
}

static int WriteTleWord(SPI_HandleTypeDef *spi, uint16_t pin, uint16_t address, uint16_t value) {
    uint16_t words[2] = {(uint16_t)(0x5001U | (address << 4)), value}, safety = 0;
    if (spi == &hspi3 && pin == CS_JointAxle_DOF3_Pin) {
        int ok = ElbowSscTransfer(words, 2, &safety, 1);
        RecordTleTransfer(spi, pin, ok, Feedback_TleCrcValid(words[0], value, safety));
        return tle_transfer_result == TLE_TRANSFER_OK;
    }
    HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_RESET);
    __HAL_SPI_DISABLE(spi);
    __HAL_SPI_CLEAR_OVRFLAG(spi);
    spi->Instance->CR1 |= SPI_CR1_BIDIOE;
    HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, (uint8_t *)words, 2, SENSOR_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        __HAL_SPI_DISABLE(spi);
        spi->Instance->CR1 &= ~SPI_CR1_BIDIOE;
        for (int i = 0; i < 4 && __HAL_SPI_GET_FLAG(spi, SPI_FLAG_RXNE); ++i) {
            volatile uint16_t dummy = spi->Instance->DR;
            (void)dummy;
        }
        status = HAL_SPI_Receive(spi, (uint8_t *)&safety, 1, SENSOR_SPI_TIMEOUT_MS);
    }
    __HAL_SPI_DISABLE(spi);
    HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_SET);
    RecordTleTransfer(spi, pin, status == HAL_OK, Feedback_TleCrcValid(words[0], value, safety));
    return tle_transfer_result == TLE_TRANSFER_OK;
}

static int ConfigureTleSensor(int index) {
    TleFeedbackState *state = &tle_state[index];
    const TleDevice *device = &tle_devices[index];
    uint16_t bank[8], mode1, safety, status;
    // Read configuration even when system-error is latched, but require valid
    // transport CRC. Angle acceptance still requires all safety bits.
    if (!ReadTleWord(device->spi, GPIOB, device->pin, 0x8001, &status, &safety)) return 0;
    state->status = status;
    if (!ReadTleRegisters(device->spi, GPIOB, device->pin, 0x8088, bank, 8, &safety) ||
        !ReadTleWord(device->spi, GPIOB, device->pin, 0x8061, &mode1, &safety)) return 0;

    if (bank[0] & 3U) {
        // Disable autocal first while preserving prediction/direction/range.
        // Changing those fields with autocal active can corrupt its coefficients.
        bank[0] &= (uint16_t)~3U;
        if (!WriteTleWord(device->spi, device->pin, 0x0F, Feedback_TleParameterCrc(bank)) ||
            !WriteTleWord(device->spi, device->pin, 0x08, bank[0])) return 0;
        HAL_Delay(1);
        if (!ReadTleRegisters(device->spi, GPIOB, device->pin, 0x8088, bank, 8, &safety)) return 0;
    }
    if (bank[0] != 0x0804) {
        bank[0] = 0x0804; // Existing 360-degree/prediction/no-autocal configuration.
        // Maintain the configuration checksum around the remaining mode change.
        uint16_t crc_word = Feedback_TleParameterCrc(bank);
        if (!WriteTleWord(device->spi, device->pin, 0x0F, crc_word) ||
            !WriteTleWord(device->spi, device->pin, 0x08, bank[0])) return 0;
    }
    if (mode1 != 0xC001 && !WriteTleWord(device->spi, device->pin, 0x06, 0xC001)) return 0;

    // Re-read after changes: coefficient bytes may have changed while autocal
    // was still active. Preserve all bytes except CRC_PAR.
    if (!ReadTleRegisters(device->spi, GPIOB, device->pin, 0x8088, bank, 8, &safety)) return 0;
    uint16_t crc_word = Feedback_TleParameterCrc(bank);
    if (bank[7] != crc_word && !WriteTleWord(device->spi, device->pin, 0x0F, crc_word)) return 0;
    HAL_Delay(1);
    if (!ReadTleRegisters(device->spi, GPIOB, device->pin, 0x8088, bank, 8, &safety) ||
        !ReadTleWord(device->spi, GPIOB, device->pin, 0x8061, &mode1, &safety) ||
        bank[0] != 0x0804 || mode1 != 0xC001 || bank[7] != Feedback_TleParameterCrc(bank)) return 0;
    if (!ReadTleWord(device->spi, GPIOB, device->pin, 0x8001, &status, &safety)) return 0;
    printf("[FB CFG] sensor=%d verified; priorSTAT=%04X clearReadSTAT=%04X\r\n",
           index, state->status, status);
    state->status = status;
    state->hard_error = 0;
    return 1;
}

float Read_TLE5012_Angle(SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio, uint16_t pin,
                         uint16_t *w1, uint16_t *w2) {
    int index = TleDeviceIndex(spi, pin);
    TleFeedbackState *state = index >= 0 ? &tle_state[index] : NULL;
    if (state) state->hard_error = 0;
    uint16_t data = 0, safety = 0;
    for (unsigned attempt = 0; attempt < SENSOR_SPI_ATTEMPTS; ++attempt) {
        if (ReadTleWord(spi, gpio, pin, 0x8021, &data, &safety)) {
            float angle = Feedback_TleAngle(0x8021, data, safety);
            if (state) {
                if (isfinite(angle)) ++state->good_angles;
                else ++state->safety_errors;
            }
            if (!isfinite(angle)) {
                uint16_t status = 0, status_safety = 0;
                int status_valid = ReadTleWord(spi, gpio, pin, 0x8001, &status, &status_safety);
                if (state) {
                    state->hard_error = 1;
                    if (status_valid) state->status = status;
                    if (!(safety & 0x8000U) || (status_valid && (status & 0x0009U)))
                        state->configured = 0;
                }
            }
            if (w1) *w1 = data;
            if (w2) *w2 = safety;
            return angle;
        }
        if (state) {
            // Diagnostic only: NEVER accept a reconstructed/shifted reading.
            uint16_t shifted_data = data >> 1;
            uint16_t shifted_safety = (uint16_t)((data << 15) | (safety >> 1));
            if (tle_transfer_result == TLE_TRANSFER_CRC &&
                (shifted_safety & 0xF000U) == 0xF000U &&
                (Feedback_TleCrcValid(0x8021, shifted_data, shifted_safety) ||
                 Feedback_TleCrcValid(0x8021, shifted_data | 0x8000U, shifted_safety)))
                ++state->shifted_frames;
            if (attempt + 1U < SENSOR_SPI_ATTEMPTS) ++state->retries;
        }
    }
    if (w1) *w1 = data;
    if (w2) *w2 = safety;
    return NAN;
}

static int ServoWritePacket(UART_HandleTypeDef *huart, const uint8_t *packet, uint16_t count) {
    uint32_t start = HAL_GetTick();
    while (huart->gState != HAL_UART_STATE_READY)
        if ((uint32_t)(HAL_GetTick() - start) >= 2U)
            return 0;
    ServoBusTransmit(huart);
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t *)packet, count, 3U);
    // Clear echo/errors while RX is still disabled so a fast reply's first
    // byte cannot be consumed by this cleanup.
    ServoBusReceive(huart);
    return status == HAL_OK;
}

float Read_LX225_Angle(UART_HandleTypeDef *huart, uint8_t id) {
    if (id > 253U)
        return NAN;
    uint8_t request[6] = {0x55, 0x55, id, 3, 28, 0};
    request[5] = (uint8_t)~(uint8_t)(id + 3U + 28U);
    // Discard old traffic before this request; only a matching current reply is accepted.
    for (int i = 0; i < 32 && __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE); ++i) {
        volatile uint32_t dummy = huart->Instance->DR;
        (void)dummy;
    }
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (!ServoWritePacket(huart, request, sizeof(request)))
        return NAN;
    uint8_t bytes[32];
    size_t count = 0;
    uint32_t start = HAL_GetTick();
    while (count < sizeof(bytes) && (uint32_t)(HAL_GetTick() - start) < WRIST_RESPONSE_TIMEOUT_MS) {
        if (huart->Instance->SR & (UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE)) {
            __HAL_UART_CLEAR_OREFLAG(huart);
            return NAN;
        }
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
            bytes[count++] = (uint8_t)huart->Instance->DR;
            float angle = Wrist_ParsePosition(bytes, count, id);
            if (isfinite(angle))
                return angle;
        }
    }
    return NAN;
}
// Función para empaquetar y enviar el protocolo de HiWonder / Lobot
void Hiwonder_SetMotorSpeed(UART_HandleTypeDef *huart, uint8_t id, int16_t speed) {
    uint8_t buf[10];
    buf[0] = 0x55; // Header
    buf[1] = 0x55; // Header
    buf[2] = id;   // Servo ID (1)
    buf[3] = 7;    // Length = Parámetros(4) + 3 = 7
    buf[4] = 29;   // CMD: SERVO_OR_MOTOR_MODE_WRITE (0x1D)
    buf[5] = 1;    // Mode: 1 = Motor (Rotación Continua), 0 = Servo
    buf[6] = 0;    // Null (reservado)
    buf[7] = (uint8_t)(speed & 0xFF);         // Low Byte de la velocidad (-1000 a 1000)
    buf[8] = (uint8_t)((speed >> 8) & 0xFF);  // High Byte
    
    // Checksum = ~(ID + Length + Command + P1 + P2 + P3 + P4) & 0xFF
    uint16_t sum = buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8];
    buf[9] = ~(sum) & 0xFF;
    
    ServoWritePacket(huart, buf, sizeof(buf));
}

// Non-blocking Interrupt-based Motor Speed Command
void Hiwonder_SetMotorSpeed_IT(UART_HandleTypeDef *huart, uint8_t id, int16_t speed) {
    if (command_queue.emergency || command_queue.overflow || motion_feedback_hold) speed = 0;
    int bus = MotorBusIndex(huart);
    if (bus < 0) return;
    MotorTxStats *stats = &motor_tx_stats[bus];
    stats->requested = speed;
    if (huart->gState != HAL_UART_STATE_READY) {
        ++stats->busy;
        stats->last_status = HAL_BUSY;
        return;
    }
    
    uint8_t *buf = NULL;
    if (huart == &huart1) buf = tx_buf_1;
    else if (huart == &huart3) buf = tx_buf_3;
    else if (huart == &huart4) buf = tx_buf_4;
    else if (huart == &huart5) buf = tx_buf_5;
    else return;

    buf[0] = 0x55; 
    buf[1] = 0x55; 
    buf[2] = id;   
    buf[3] = 7;    
    buf[4] = 29;   
    buf[5] = 1;    
    buf[6] = 0;    
    buf[7] = (uint8_t)(speed & 0xFF);         
    buf[8] = (uint8_t)((speed >> 8) & 0xFF);  
    uint16_t sum = buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8];
    buf[9] = ~(sum) & 0xFF;
    
    ServoBusTransmit(huart);
    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(huart, buf, 10);
    stats->last_status = status;
    if (status == HAL_OK) ++stats->submitted;
    else if (status == HAL_BUSY) ++stats->busy;
    else ++stats->failed;
    if (status != HAL_OK && huart->gState == HAL_UART_STATE_READY) ServoBusReceive(huart);
}

// Re-enable RX after non-blocking IT transmission is complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) { JD_TransmitComplete(); return; }
    if (huart->Instance == USART1 || huart->Instance == USART3 || 
        huart->Instance == UART4  || huart->Instance == UART5) {
        ServoBusReceive(huart);
        int bus = MotorBusIndex(huart);
        if (bus >= 0) ++motor_tx_stats[bus].completed;
    }
}


// Función para eliminar los límites de ángulo de EEPROM del LX-225
void Hiwonder_UnlockAngleLimits(UART_HandleTypeDef *huart, uint8_t id) {
    uint8_t buf[10];
    buf[0] = 0x55; buf[1] = 0x55;
    buf[2] = id;
    buf[3] = 7;
    buf[4] = 20; // CMD: SERVO_ANGLE_LIMIT_WRITE
    buf[5] = 0;  // Min Angle L = 0
    buf[6] = 0;  // Min Angle H = 0
    buf[7] = 0xE8; // Max Angle L = 1000 (0x03E8)
    buf[8] = 0x03; // Max Angle H
    uint16_t sum = buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8];
    buf[9] = ~(sum) & 0xFF;
    
    ServoWritePacket(huart, buf, sizeof(buf));
}

// Función para establecer el modo a Servo y enviarlo a una posición específica
int Hiwonder_SetServoPosition(UART_HandleTypeDef *huart, uint8_t id, float angle_raw_deg,
                              uint16_t time_ms) {
    if (command_queue.emergency || command_queue.overflow) return 0;
    // 1. Asegurarse de que el LX-225 esté en modo Servo (0)
    uint8_t mode_buf[10];
    mode_buf[0] = 0x55;
    mode_buf[1] = 0x55;
    mode_buf[2] = id;
    mode_buf[3] = 7;
    mode_buf[4] = 29; // CMD: SERVO_OR_MOTOR_MODE_WRITE
    mode_buf[5] = 0;  // Mode: 0 = Servo
    mode_buf[6] = 0;
    mode_buf[7] = 0;
    mode_buf[8] = 0;
    uint16_t sum_mode = mode_buf[2] + mode_buf[3] + mode_buf[4] + mode_buf[5] + mode_buf[6] +
                        mode_buf[7] + mode_buf[8];
    mode_buf[9] = ~(sum_mode) & 0xFF;

    if (!ServoWritePacket(huart, mode_buf, sizeof(mode_buf)))
        return 0;

    HAL_Delay(5);
    if (command_queue.emergency || command_queue.overflow) return 0;

    // Convertir de grados (0-240 aprox) al valor crudo del servo (0-1000)
    if (angle_raw_deg < 0.0f)
        angle_raw_deg = 0.0f;
    if (angle_raw_deg > 240.0f)
        angle_raw_deg = 240.0f; // Asumiendo rango max
    uint16_t pos_val = (uint16_t)((angle_raw_deg / 240.0f) * 1000.0f);

    // 2. Enviar posición (CMD 1: SERVO_MOVE_TIME_WRITE)
    uint8_t pos_buf[10];
    pos_buf[0] = 0x55;
    pos_buf[1] = 0x55;
    pos_buf[2] = id;
    pos_buf[3] = 7;
    pos_buf[4] = 1;
    pos_buf[5] = (uint8_t)(pos_val & 0xFF);
    pos_buf[6] = (uint8_t)((pos_val >> 8) & 0xFF);
    pos_buf[7] = (uint8_t)(time_ms & 0xFF);
    pos_buf[8] = (uint8_t)((time_ms >> 8) & 0xFF);
    uint16_t sum_pos =
        pos_buf[2] + pos_buf[3] + pos_buf[4] + pos_buf[5] + pos_buf[6] + pos_buf[7] + pos_buf[8];
    pos_buf[9] = ~(sum_pos) & 0xFF;

    return ServoWritePacket(huart, pos_buf, sizeof(pos_buf));
}

// Callback disparado automáticamente cuando el DMA de UART termina de recibir un paquete de la
// Portenta
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART6) {
        if (!portenta_rx_running) return; // Ignore late events from the aborted DMA.
        // Circular DMA Size is a write position, not a fresh packet length.
        // Duplicate HT/TC/IDLE positions contain no new bytes.
        if (!Size || Size > RX_BUFFER_SIZE) {
            command_queue.overflow |= RX_FAULT_DMA_POSITION;
            return;
        }
        if (Size > portenta_rx_position)
            CommandQueue_Push(&command_queue, rx_buffer + portenta_rx_position, Size - portenta_rx_position);
        else if (Size < portenta_rx_position) {
            CommandQueue_Push(&command_queue, rx_buffer + portenta_rx_position, RX_BUFFER_SIZE - portenta_rx_position);
            CommandQueue_Push(&command_queue, rx_buffer, Size);
        }
        portenta_rx_position = Size;
        uint32_t used = (command_queue.head + COMMAND_QUEUE_SIZE - command_queue.tail) % COMMAND_QUEUE_SIZE;
        if (used > portenta_queue_peak) portenta_queue_peak = used;
    }
}

// Callback de error (Vital para recuperarse de un Overrun Error - ORE si la Portenta manda datos
// muy rápido)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    int bus = MotorBusIndex(huart);
    if (bus >= 0) {
        motor_tx_stats[bus].last_uart_error = huart->ErrorCode;
        ++motor_tx_stats[bus].uart_errors;
    }
    if (huart->Instance == USART6) {
        portenta_uart_error = huart->ErrorCode;
        ++portenta_rx_errors;
        if (huart->ErrorCode & HAL_UART_ERROR_NE) ++portenta_noise_errors;
        if (huart->ErrorCode & HAL_UART_ERROR_FE) ++portenta_frame_errors;
        if (huart->ErrorCode & HAL_UART_ERROR_ORE) ++portenta_overruns;
        if (huart->ErrorCode & HAL_UART_ERROR_PE) ++portenta_parity_errors;
        if (huart->ErrorCode & HAL_UART_ERROR_DMA) ++portenta_dma_errors;
        command_queue.overflow |= RX_FAULT_UART;
        command_queue.partial_length = 0;
        command_queue.discard = 1;
        portenta_rx_running = 0;
        // Do not restart DMA, read DR, transmit, or block inside a HAL error
        // callback. Recovery is serviced in main, while motion remains latched.
        __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
        __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
    } else __HAL_UART_CLEAR_OREFLAG(huart);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
