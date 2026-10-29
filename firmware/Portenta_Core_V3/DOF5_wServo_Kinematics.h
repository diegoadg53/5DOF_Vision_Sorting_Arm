#ifndef DOF5_WSERVO_KINEMATICS_H
#define DOF5_WSERVO_KINEMATICS_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

enum class IkState : uint8_t { Idle, Running, Success, Failed };
enum class IkTask : uint8_t { FullPose, PositionPitchYaw, PositionClosestOrientation, PositionOnly };
enum class IkFailure : uint8_t {
  None,
  InvalidInput,
  EulerSingularity,
  IterationLimit,
  LinearSolve
};
struct IkReport {
  IkState state = IkState::Idle;
  IkFailure failure = IkFailure::None;
  int iterations = 0;
  float positionErrorMm = NAN, controlledRotationErrorRad = NAN, fullRotationErrorRad = NAN;
  IkTask task = IkTask::FullPose;
};
const char *ik_failure_name(IkFailure failure);
const char *ik_task_name(IkTask task);

// ===================================
// Camera Calibration defines
// ===================================
//ALL Y + 77
#define APRILTAG_DICT "36h11"
#define TAG0_ID 0
#define TAG0_WORLD_X 114.0f // 39+75
#define TAG0_WORLD_Y 614.0f // 300+253 + 61
#define TAG1_ID 1
#define TAG1_WORLD_X 113.0f // 38+75
#define TAG1_WORLD_Y 417.0f // 553-197 + 61
#define TAG2_ID 2
#define TAG2_WORLD_X 550.5f // 505-29.5+75
#define TAG2_WORLD_Y 656.0f // 315+280 + 61
#define TAG3_ID 3
#define TAG3_WORLD_X 550.0f // 505-30+75
#define TAG3_WORLD_Y 376.0f // 300+15 + 61

// Estructura para almacenar la matriz de homografía 3x3
struct calibH {
  Matrix<3, 3> H;
  bool valid = false;
};

// Estructura auxiliar para retornar coordenadas 2D (X, Y)
struct Point2D {
  float x;
  float y;
};

// Definición de la estructura de parámetros DH
struct RobotDHParams {
  // Final DH local axes -> TCP, mm.
  float toolOffset[3] = {0, 0, -49}; //-49
  float theta_offset[5];
  float d[5];
  float a[5];
  float cos_alpha[5]; // Pre-cálculo del coseno de alpha
  float sin_alpha[5]; // Pre-cálculo del seno de alpha
  Matrix<4, 4> T_B0;

  // Límites articulares físicos (en radianes para cálculo interno)
  float q_min[5];
  float q_max[5];
};

// --- Prototipos de Funciones ---

// Generador de parámetros DH por defecto
RobotDHParams robot_get_dh_params();

// Cinemática Directa completa: salida TCP; T_acc conserva los marcos DH
void robot_fkine_all(const float q[5], const RobotDHParams &params, Matrix<4, 4> T_acc[5],
                     Matrix<4, 4> &T_base_E);

// Jacobiano geométrico rotado a la base
Matrix<6, 5> robot_jacobian(const Matrix<4, 4> T_acc[5], const RobotDHParams &params);

// Error de pose completo, sin proyección de eje ciego
Matrix<6, 1> error_vector(const Matrix<4, 4> &T_actual, const Matrix<4, 4> &T_deseada,
                          const Matrix<6, 5> &J);

// Cinemática Inversa con tolerancias separadas:
// tol_pos en mm (ej. 0.5f)
// tol_rot en radianes (ej. 0.02f rad ≈ 1.15°)
bool robot_ikine(const float q0[5], const float T_parametros[6], int maxiter, float tol_pos,
                 float tol_rot, const RobotDHParams &params, float q_out[5],
                 IkTask task = IkTask::FullPose, IkReport *report = nullptr);
// Commit an incremental target and its command seed only after a valid solution.
bool robot_ikine_increment(float seed[5], float pose[6], const float increment[6], int maxiter,
                           float tol_pos, float tol_rot, const RobotDHParams &params,
                           IkReport &report);

// Cinemática Directa del TCP: q en grados -> target [x, y, z, roll, pitch, yaw]
void robot_fkine(const float q[5], const RobotDHParams &params, float target[6]);

// Actualiza los límites físicos del robot en el objeto params (recibe grados)
void updateLimits(RobotDHParams &params, float roll_min, float roll_max, float pitch_min,
                  float pitch_max, float dof3_min, float dof3_max, float dof4_min, float dof4_max,
                  float dof5_min, float dof5_max);

// --- Prototipos de Homografía Automática ---
// Calcula la matriz H a partir de las coordenadas pixel (u, v) de los 4 tags
calibH AutoHomography(const float u[4], const float v[4]);

// Transforma coordenadas pixel (u, v) a coordenadas reales (x, y en mm)
Point2D pixelTransformation(float u, float v, const calibH &calib);
void pixelTransformation(float u, float v, const calibH &calib, float &x_out, float &y_out);

class IkStepper {
public:
  void begin(const float seed[5], const float pose[6], const RobotDHParams &p, bool ignoreRoll,
             int maxIterations = 150, float posTolerance = .5f, float rotTolerance = .035f);
  void begin(const float seed[5], const float pose[6], const RobotDHParams &p, IkTask task,
             int maxIterations = 150, float posTolerance = .5f, float rotTolerance = .035f,
             uint8_t fixedJointMask = 0);
  IkState step(uint8_t budget = 2);
  IkState state() const {
    return state_;
  }
  void result(float q[5]) const;
  IkReport report() const {
    return {state_,         failure_,           iteration_, positionError_,
            rotationError_, fullRotationError_, task_};
  }

private:
  const RobotDHParams *p_ = nullptr;
  float q_[5]{};
  BLA::Matrix<4, 4> desired_;
  float pitch_ = 0, yaw_ = 0, posTol_ = .5f, rotTol_ = .035f;
  int iteration_ = 0, maxIterations_ = 150;
  IkTask task_ = IkTask::PositionPitchYaw;
  // Bits 0..4 pin internal pitch, roll, j3, j4, j5 to their seed values.
  uint8_t fixedJointMask_ = 0;
  IkFailure failure_ = IkFailure::None;
  float positionError_ = NAN, rotationError_ = NAN, fullRotationError_ = NAN;
  IkState state_ = IkState::Idle;
};

#endif // DOF5_WSERVO_KINEMATICS_H
