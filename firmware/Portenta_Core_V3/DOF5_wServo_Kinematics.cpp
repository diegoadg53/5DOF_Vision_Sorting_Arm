#include "DOF5_wServo_Kinematics.h"

#include <BasicLinearAlgebra.h>
using namespace BLA;

RobotDHParams robot_get_dh_params() {
  RobotDHParams params;

  // 1. Offsets articulares (theta) en radianes para la configuración cero[cite: 5, 13]
  params.theta_offset[0] = PI / 2.0f; // 90°
  params.theta_offset[1] = 0.0f;
  params.theta_offset[2] = 0.0f;
  params.theta_offset[3] = 0.0f;
  params.theta_offset[4] = 0.0f;

  // 2. Distancias d en mm[cite: 5]
  params.d[0] = 0.0f;
  params.d[1] = 315.557f;
  params.d[2] = 0.0f;
  params.d[3] = 0.0f;
  params.d[4] = 115.14f;

  // 3. Longitudes de eslabón a en mm[cite: 5]
  params.a[0] = 0.0f;
  params.a[1] = 47.45f;
  params.a[2] = 307.0f;
  params.a[3] = 0.0f;
  params.a[4] = 0.0f;

  // 4. Valores trigonométricos pre-calculados de alpha[cite: 5]
  // Alpha 1: 90°
  params.cos_alpha[0] = 0.0f;
  params.sin_alpha[0] = 1.0f;
  // Alpha 2: 90°
  params.cos_alpha[1] = 0.0f;
  params.sin_alpha[1] = 1.0f;
  // Alpha 3: 180°
  params.cos_alpha[2] = -1.0f;
  params.sin_alpha[2] = 0.0f;
  // Alpha 4: -90°
  params.cos_alpha[3] = 0.0f;
  params.sin_alpha[3] = -1.0f;
  // Alpha 5: 180°
  params.cos_alpha[4] = -1.0f;
  params.sin_alpha[4] = 0.0f;

  // 5. Matriz de transformación fija de Base a Marco 0 (^B T_0)[cite: 5]
  params.T_B0 = {0.0f, 1.0f, 0.0f, -76.0f,   0.0f, 0.0f, 1.0f, 0.0f,
                 1.0f, 0.0f, 0.0f, 102.465f, 0.0f, 0.0f, 0.0f, 1.0f};

  // 6. Límites por defecto (en radianes) para evitar valores basura en memoria
  params.q_min[0] = -50.0f * PI / 180.0f;
  params.q_max[0] = 60.0f * PI / 180.0f;
  params.q_min[1] = -60.0f * PI / 180.0f;
  params.q_max[1] = 120.0f * PI / 180.0f;
  params.q_min[2] = -70.0f * PI / 180.0f;
  params.q_max[2] = 90.0f * PI / 180.0f;
  params.q_min[3] = -200.0f * PI / 180.0f;
  params.q_max[3] = 30.0f * PI / 180.0f;
  params.q_min[4] = -95.0f * PI / 180.0f;
  params.q_max[4] = 84.0f * PI / 180.0f;

  return params;
}

namespace {
template <int N> bool solveSystem(double a[N][N], double b[N], double x[N]) {
  for (int k = 0; k < N; ++k) {
    int pivot = k;
    for (int i = k + 1; i < N; ++i)
      if (fabs(a[i][k]) > fabs(a[pivot][k]))
        pivot = i;
    if (!isfinite(a[pivot][k]) || fabs(a[pivot][k]) < 1e-12)
      return false;
    if (pivot != k) {
      for (int j = k; j < N; ++j) {
        double t = a[k][j];
        a[k][j] = a[pivot][j];
        a[pivot][j] = t;
      }
      double t = b[k];
      b[k] = b[pivot];
      b[pivot] = t;
    }
    for (int i = k + 1; i < N; ++i) {
      double r = a[i][k] / a[k][k];
      for (int j = k; j < N; ++j)
        a[i][j] -= r * a[k][j];
      b[i] -= r * b[k];
    }
  }
  for (int i = N - 1; i >= 0; --i) {
    double s = b[i];
    for (int j = i + 1; j < N; ++j)
      s -= a[i][j] * x[j];
    x[i] = s / a[i][i];
    if (!isfinite(x[i]))
      return false;
  }
  return true;
}
float angleWrap(float a) {
  return atan2f(sinf(a), cosf(a));
}
// Build the attainable angular-velocity projector with TCP position held:
// range(J_rotation * null(J_position)). This is pose-dependent and includes
// the tool offset through the TCP Jacobian; an arbitrary cross product does not.
Matrix<3, 3> orientationProjector(const Matrix<6, 5> &j) {
  double rowBasis[3][5]{}, nullBasis[5][5]{}, angularBasis[3][3]{};
  int rank = 0, nullity = 0, angularRank = 0;
  for (int row = 0; row < 3; ++row) {
    double v[5], original = 0;
    for (int k = 0; k < 5; ++k) {
      v[k] = j(row, k);
      original += v[k] * v[k];
    }
    for (int pass = 0; pass < 2; ++pass)
      for (int r = 0; r < rank; ++r) {
        double dot = 0;
        for (int k = 0; k < 5; ++k)
          dot += v[k] * rowBasis[r][k];
        for (int k = 0; k < 5; ++k)
          v[k] -= dot * rowBasis[r][k];
      }
    double norm = 0;
    for (double x : v)
      norm += x * x;
    norm = sqrt(norm);
    if (norm > fmax(1e-6, sqrt(original) * 1e-6)) {
      for (int k = 0; k < 5; ++k)
        rowBasis[rank][k] = v[k] / norm;
      ++rank;
    }
  }
  for (int axis = 0; axis < 5; ++axis) {
    double v[5]{};
    v[axis] = 1;
    for (int pass = 0; pass < 2; ++pass) {
      for (int r = 0; r < rank; ++r) {
        double dot = 0;
        for (int k = 0; k < 5; ++k)
          dot += v[k] * rowBasis[r][k];
        for (int k = 0; k < 5; ++k)
          v[k] -= dot * rowBasis[r][k];
      }
      for (int r = 0; r < nullity; ++r) {
        double dot = 0;
        for (int k = 0; k < 5; ++k)
          dot += v[k] * nullBasis[r][k];
        for (int k = 0; k < 5; ++k)
          v[k] -= dot * nullBasis[r][k];
      }
    }
    double norm = 0;
    for (double x : v)
      norm += x * x;
    norm = sqrt(norm);
    if (norm > 1e-6) {
      for (int k = 0; k < 5; ++k)
        nullBasis[nullity][k] = v[k] / norm;
      ++nullity;
    }
  }
  for (int n = 0; n < nullity && angularRank < 3; ++n) {
    double v[3]{};
    for (int r = 0; r < 3; ++r)
      for (int k = 0; k < 5; ++k)
        v[r] += j(r + 3, k) * nullBasis[n][k];
    for (int pass = 0; pass < 2; ++pass)
      for (int r = 0; r < angularRank; ++r) {
        double dot = 0;
        for (int k = 0; k < 3; ++k)
          dot += v[k] * angularBasis[r][k];
        for (int k = 0; k < 3; ++k)
          v[k] -= dot * angularBasis[r][k];
      }
    double norm = 0;
    for (double x : v)
      norm += x * x;
    norm = sqrt(norm);
    if (norm > 1e-5) {
      for (int k = 0; k < 3; ++k)
        angularBasis[angularRank][k] = v[k] / norm;
      ++angularRank;
    }
  }
  Matrix<3, 3> projector;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) {
      projector(r, c) = 0;
      for (int n = 0; n < angularRank; ++n)
        projector(r, c) += float(angularBasis[n][r] * angularBasis[n][c]);
    }
  return projector;
}
float projectedRotationError(const Matrix<6, 1> &e, const Matrix<3, 3> &projector) {
  float v[3]{};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      v[r] += projector(r, c) * e(c + 3);
  return hypotf(hypotf(v[0], v[1]), v[2]);
}
void eulerAngles(const Matrix<4, 4> &t, float &r, float &p, float &y) {
  p = atan2f(-t(2, 0), hypotf(t(0, 0), t(1, 0)));
  if (fabsf(cosf(p)) > 1e-5f) {
    r = atan2f(t(2, 1), t(2, 2));
    y = atan2f(t(1, 0), t(0, 0));
  } else {
    y = 0;
    r = t(2, 0) < 0 ? atan2f(t(0, 1), t(0, 2)) : atan2f(-t(0, 1), -t(0, 2));
  }
}
Matrix<4, 4> poseMatrix(const float p[6]) {
  float r = p[3] * PI / 180, t = p[4] * PI / 180, y = p[5] * PI / 180;
  float cr = cosf(r), sr = sinf(r), cp = cosf(t), sp = sinf(t), cy = cosf(y), sy = sinf(y);
  return {cy * cp,
          cy * sp * sr - sy * cr,
          cy * sp * cr + sy * sr,
          p[0],
          sy * cp,
          sy * sp * sr + cy * cr,
          sy * sp * cr - cy * sr,
          p[1],
          -sp,
          cp * sr,
          cp * cr,
          p[2],
          0,
          0,
          0,
          1};
}
} // namespace
void robot_fkine_all(const float q[5], const RobotDHParams &p, Matrix<4, 4> a[5],
                     Matrix<4, 4> &tcp) {
  for (int i = 0; i < 5; ++i) {
    float t = q[i] + p.theta_offset[i], c = cosf(t), s = sinf(t);
    float ca = p.cos_alpha[i], sa = p.sin_alpha[i];
    Matrix<4, 4> v = {c, -s * ca, s * sa, p.a[i] * c, s, c * ca, -c * sa, p.a[i] * s,
                      0, sa,      ca,     p.d[i],     0, 0,      0,       1};
    if (i == 0)
      a[i] = v;
    else
      a[i] = a[i - 1] * v;
  }
  tcp = p.T_B0 * a[4];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      tcp(i, 3) += tcp(i, j) * p.toolOffset[j];
}
Matrix<6, 5> robot_jacobian(const Matrix<4, 4> a[5], const RobotDHParams &p) {
  Matrix<6, 5> j;
  float end[3];
  for (int i = 0; i < 3; ++i) {
    end[i] = a[4](i, 3);
    for (int k = 0; k < 3; ++k)
      end[i] += a[4](i, k) * p.toolOffset[k];
  }
  for (int n = 0; n < 5; ++n) {
    float z[3] = {0, 0, 1}, o[3] = {0, 0, 0}, v[3], d[3];
    if (n)
      for (int k = 0; k < 3; ++k) {
        z[k] = a[n - 1](k, 2);
        o[k] = a[n - 1](k, 3);
      }
    for (int k = 0; k < 3; ++k)
      d[k] = end[k] - o[k];
    v[0] = z[1] * d[2] - z[2] * d[1];
    v[1] = z[2] * d[0] - z[0] * d[2];
    v[2] = z[0] * d[1] - z[1] * d[0];
    for (int i = 0; i < 3; ++i) {
      j(i, n) = j(i + 3, n) = 0;
      for (int k = 0; k < 3; ++k) {
        j(i, n) += p.T_B0(i, k) * v[k];
        j(i + 3, n) += p.T_B0(i, k) * z[k];
      }
    }
  }
  return j;
}
Matrix<6, 1> error_vector(const Matrix<4, 4> &a, const Matrix<4, 4> &d, const Matrix<6, 5> &) {
  Matrix<6, 1> e;
  double r[3][3];
  for (int i = 0; i < 3; ++i) {
    e(i) = d(i, 3) - a(i, 3);
    for (int j = 0; j < 3; ++j) {
      r[i][j] = 0;
      for (int k = 0; k < 3; ++k)
        r[i][j] += double(d(i, k)) * a(j, k);
    }
  }
  double c = fmax(-1.0, fmin(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2));
  double theta = acos(c), axis[3] = {r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]};
  if (theta < 1e-5)
    for (int i = 0; i < 3; ++i)
      e(i + 3) = float(axis[i] / 2);
  else if (PI - theta < 1e-3) {
    int k = 0;
    if (r[1][1] > r[k][k])
      k = 1;
    if (r[2][2] > r[k][k])
      k = 2;
    axis[k] = sqrt(fmax(0.0, (r[k][k] + 1) / 2));
    for (int i = 0; i < 3; ++i)
      if (i != k)
        axis[i] = (r[i][k] + r[k][i]) / (4 * axis[k]);
    double norm = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    for (int i = 0; i < 3; ++i)
      e(i + 3) = float(theta * axis[i] / norm);
  } else
    for (int i = 0; i < 3; ++i)
      e(i + 3) = float(theta * axis[i] / (2 * sin(theta)));
  return e;
}
void IkStepper::begin(const float seed[5], const float pose[6], const RobotDHParams &p,
                      bool ignoreRoll, int maximum, float pt, float rt) {
  begin(seed, pose, p, ignoreRoll ? IkTask::PositionPitchYaw : IkTask::FullPose, maximum, pt, rt);
}
const char *ik_failure_name(IkFailure failure) {
  switch (failure) {
  case IkFailure::None:
    return "none";
  case IkFailure::InvalidInput:
    return "invalid input";
  case IkFailure::EulerSingularity:
    return "pitch/yaw singularity";
  case IkFailure::IterationLimit:
    return "iteration limit/stalled";
  case IkFailure::LinearSolve:
    return "linear solve failure";
  }
  return "unknown";
}
const char *ik_task_name(IkTask task) {
  switch (task) {
  case IkTask::FullPose:
    return "full pose";
  case IkTask::PositionPitchYaw:
    return "XYZ/pitch/yaw";
  case IkTask::PositionClosestOrientation:
    return "XYZ/attainable orientation";
  case IkTask::PositionOnly:
    return "XYZ only";
  }
  return "unknown";
}
void IkStepper::begin(const float seed[5], const float pose[6], const RobotDHParams &p, IkTask task,
                      int maximum, float pt, float rt, uint8_t fixedJointMask) {
  state_ = IkState::Failed;
  failure_ = IkFailure::InvalidInput;
  iteration_ = 0;
  positionError_ = rotationError_ = fullRotationError_ = NAN;
  task_ = task;
  fixedJointMask_ = fixedJointMask;
  for (float &q : q_)
    q = NAN;
  const bool ignoreRoll = task == IkTask::PositionPitchYaw;
  if (maximum <= 0 || !isfinite(pt) || !isfinite(rt) || !(pt > 0) || !(rt > 0) ||
      (fixedJointMask & ~uint8_t(0x1f)))
    return;
  for (int i = 0; i < 6; ++i)
    if (!isfinite(pose[i]))
      return;
  if (ignoreRoll && fabsf(pose[4]) >= 89.0f)
    return;
  for (int i = 0; i < 5; ++i) {
    if (!isfinite(seed[i]) || !isfinite(p.q_min[i]) || !isfinite(p.q_max[i]) ||
        p.q_min[i] >= p.q_max[i])
      return;
    const float radians = seed[i] * PI / 180;
    if ((fixedJointMask & (1U << i)) && (radians < p.q_min[i] || radians > p.q_max[i]))
      return;
    q_[i] = fmaxf(p.q_min[i], fminf(p.q_max[i], radians));
  }
  p_ = &p;
  desired_ = poseMatrix(pose);
  pitch_ = pose[4] * PI / 180;
  yaw_ = pose[5] * PI / 180;
  posTol_ = pt;
  rotTol_ = rt;
  task_ = task;
  maxIterations_ = maximum;
  iteration_ = 0;
  if (ignoreRoll && !(fixedJointMask & 1U)) {
    Matrix<4, 4> a[5], t;
    robot_fkine_all(q_, p, a, t);
    float r, pp, yy;
    eulerAngles(t, r, pp, yy);
    if (fabsf(cosf(pp)) < 1e-3f) {
      float delta = 2 * PI / 180;
      q_[0] += q_[0] + delta <= p.q_max[0] ? delta : -delta;
      q_[0] = fmaxf(p.q_min[0], fminf(p.q_max[0], q_[0]));
    }
  }
  state_ = IkState::Running;
  failure_ = IkFailure::None;
}
void IkStepper::result(float q[5]) const {
  for (int i = 0; i < 5; ++i) {
    float degrees = q_[i] * 180 / PI;
    // Success was already verified at 0.01-degree command precision. Preserve
    // that representation through the radian-to-degree round trip, especially
    // at a joint limit (e.g. 84.00 must not become 84.000008).
    q[i] = state_ == IkState::Success ? roundf(degrees * 100) / 100 : degrees;
  }
}
IkState IkStepper::step(uint8_t budget) {
  while (budget-- && state_ == IkState::Running) {
    Matrix<4, 4> a[5], t;
    robot_fkine_all(q_, *p_, a, t);
    Matrix<6, 5> j = robot_jacobian(a, *p_);
    for (int k = 0; k < 5; ++k)
      if (fixedJointMask_ & (1U << k))
        for (int row = 0; row < 6; ++row)
          j(row, k) = 0;
    Matrix<6, 1> e = error_vector(t, desired_, j);
    float roll, pitch, yaw;
    eulerAngles(t, roll, pitch, yaw);
    const bool ignoreRoll = task_ == IkTask::PositionPitchYaw;
    const bool closest = task_ == IkTask::PositionClosestOrientation;
    const bool positionOnly = task_ == IkTask::PositionOnly;
    if (ignoreRoll && fabsf(cosf(pitch)) < 1e-4f) {
      state_ = IkState::Failed;
      failure_ = IkFailure::EulerSingularity;
      break;
    }
    double jt[6][5]{}, et[6]{};
    int rows = positionOnly ? 3 : ignoreRoll ? 5 : 6;
    Matrix<3, 3> projector;
    if (closest)
      projector = orientationProjector(j);
    for (int i = 0; i < 3; ++i) {
      et[i] = e(i) / 50.0;
      for (int k = 0; k < 5; ++k)
        jt[i][k] = j(i, k) / 50.0;
    }
    float ep = angleWrap(pitch_ - pitch), ey = angleWrap(yaw_ - yaw);
    if (ignoreRoll) {
      et[3] = ep;
      et[4] = ey;
      for (int k = 0; k < 5; ++k) {
        jt[3][k] = -sinf(yaw) * j(3, k) + cosf(yaw) * j(4, k);
        jt[4][k] = tanf(pitch) * (cosf(yaw) * j(3, k) + sinf(yaw) * j(4, k)) + j(5, k);
      }
    } else if (closest) {
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
          et[r + 3] += projector(r, c) * e(c + 3);
          for (int k = 0; k < 5; ++k)
            jt[r + 3][k] += projector(r, c) * j(c + 3, k);
        }
    } else if (!positionOnly) {
      for (int i = 3; i < 6; ++i) {
        et[i] = e(i);
        for (int k = 0; k < 5; ++k)
          jt[i][k] = j(i, k);
      }
    }
    float pos = hypotf(hypotf(e(0), e(1)), e(2));
    float fullRot = hypotf(hypotf(e(3), e(4)), e(5));
    float rot = positionOnly ? 0 : ignoreRoll ? fmaxf(fabsf(ep), fabsf(ey))
                : closest  ? projectedRotationError(e, projector)
                           : fullRot;
    positionError_ = pos;
    rotationError_ = rot;
    fullRotationError_ = fullRot;
    if (!isfinite(pos) || !isfinite(rot)) {
      state_ = IkState::Failed;
      failure_ = IkFailure::InvalidInput;
      break;
    }
    if (pos <= posTol_ && rot <= rotTol_) {
      float rounded[5];
      for (int i = 0; i < 5; ++i)
        rounded[i] = roundf(q_[i] * 180 / PI * 100) / 100 * PI / 180;
      Matrix<4, 4> aa[5], tt;
      robot_fkine_all(rounded, *p_, aa, tt);
      Matrix<6, 5> jj = robot_jacobian(aa, *p_);
      for (int k = 0; k < 5; ++k)
        if (fixedJointMask_ & (1U << k))
          for (int row = 0; row < 6; ++row)
            jj(row, k) = 0;
      Matrix<6, 1> ee = error_vector(tt, desired_, jj);
      float rr, pp, yy;
      eulerAngles(tt, rr, pp, yy);
      float pe = hypotf(hypotf(ee(0), ee(1)), ee(2));
      float fullRe = hypotf(hypotf(ee(3), ee(4)), ee(5));
      float re = positionOnly ? 0 : ignoreRoll ? fmaxf(fabsf(angleWrap(pitch_ - pp)), fabsf(angleWrap(yaw_ - yy)))
                 : closest  ? projectedRotationError(ee, orientationProjector(jj))
                            : fullRe;
      bool good = isfinite(pe) && isfinite(re) && pe <= posTol_ && re <= rotTol_;
      for (int i = 0; i < 5; ++i)
        good = good && rounded[i] >= p_->q_min[i] && rounded[i] <= p_->q_max[i];
      if (good) {
        for (int i = 0; i < 5; ++i)
          q_[i] = rounded[i];
        state_ = IkState::Success;
        positionError_ = pe;
        rotationError_ = re;
        fullRotationError_ = fullRe;
        break;
      }
      // Rounding can push a barely acceptable solution outside tolerance.
      // Continue refining instead of failing before the iteration budget is used.
    }
    if (iteration_ >= maxIterations_) {
      state_ = IkState::Failed;
      failure_ = IkFailure::IterationLimit;
      break;
    }
    ++iteration_;
    double h[5][5]{}, b[5]{}, dq[5]{};
    for (int i = 0; i < 5; ++i) {
      for (int n = 0; n < rows; ++n)
        b[i] += jt[n][i] * et[n];
      for (int k = 0; k < 5; ++k) {
        for (int n = 0; n < rows; ++n)
          h[i][k] += jt[n][i] * jt[n][k];
      }
      h[i][i] += .0001;
    }
    if (!solveSystem<5>(h, b, dq)) {
      state_ = IkState::Failed;
      failure_ = IkFailure::LinearSolve;
      break;
    }
    double largest = 0;
    for (double v : dq)
      largest = fmax(largest, fabs(v));
    double gain = .7 * (largest > .12 ? .12 / largest : 1.0);
    for (int i = 0; i < 5; ++i)
      if (!(fixedJointMask_ & (1U << i)))
        q_[i] = fmaxf(p_->q_min[i], fminf(p_->q_max[i], q_[i] + float(gain * dq[i])));
  }
  return state_;
}
bool robot_ikine(const float q0[5], const float pose[6], int maxiter, float tp, float tr,
                 const RobotDHParams &p, float out[5], IkTask task, IkReport *report) {
  IkStepper solver;
  solver.begin(q0, pose, p, task, maxiter, tp, tr);
  while (solver.state() == IkState::Running)
    solver.step(1);
  if (report)
    *report = solver.report();
  if (solver.state() != IkState::Success)
    return false;
  solver.result(out);
  return true;
}
bool robot_ikine_increment(float seed[5], float pose[6], const float increment[6], int maxiter,
                           float tp, float tr, const RobotDHParams &p, IkReport &report) {
  float candidate[6], solution[5];
  for (int i = 0; i < 6; ++i)
    candidate[i] = pose[i] + increment[i];
  if (!robot_ikine(seed, candidate, maxiter, tp, tr, p, solution,
                   IkTask::PositionClosestOrientation, &report))
    return false;
  for (int i = 0; i < 6; ++i)
    pose[i] = candidate[i];
  for (int i = 0; i < 5; ++i)
    seed[i] = solution[i];
  return true;
}
void robot_fkine(const float q[5], const RobotDHParams &p, float out[6]) {
  float qr[5];
  for (int i = 0; i < 5; ++i)
    qr[i] = q[i] * PI / 180;
  Matrix<4, 4> a[5], t;
  robot_fkine_all(qr, p, a, t);
  for (int i = 0; i < 3; ++i)
    out[i] = t(i, 3);
  float r, pitch, y;
  eulerAngles(t, r, pitch, y);
  out[3] = r * 180 / PI;
  out[4] = pitch * 180 / PI;
  out[5] = y * 180 / PI;
}

void updateLimits(RobotDHParams &p, float rmin, float rmax, float pmin, float pmax, float a3,
                  float b3, float a4, float b4, float a5, float b5) {
  const float lo[5] = {pmin, rmin, a3, a4, a5}, hi[5] = {pmax, rmax, b3, b4, b5};
  for (int i = 0; i < 5; ++i) {
    p.q_min[i] = lo[i] * PI / 180;
    p.q_max[i] = hi[i] * PI / 180;
  }
}
calibH AutoHomography(const float u[4], const float v[4]) {
  const double x[4] = {TAG0_WORLD_X, TAG1_WORLD_X, TAG2_WORLD_X, TAG3_WORLD_X};
  const double y[4] = {TAG0_WORLD_Y, TAG1_WORLD_Y, TAG2_WORLD_Y, TAG3_WORLD_Y};
  calibH c;
  double a[8][8]{}, b[8]{}, h[8]{};
  for (int i = 0; i < 4; ++i) {
    if (!isfinite(u[i]) || !isfinite(v[i]))
      return c;
    int r = 2 * i;
    a[r][0] = u[i];
    a[r][1] = v[i];
    a[r][2] = 1;
    a[r][6] = -x[i] * u[i];
    a[r][7] = -x[i] * v[i];
    b[r] = x[i];
    a[r + 1][3] = u[i];
    a[r + 1][4] = v[i];
    a[r + 1][5] = 1;
    a[r + 1][6] = -y[i] * u[i];
    a[r + 1][7] = -y[i] * v[i];
    b[r + 1] = y[i];
  }
  if (!solveSystem<8>(a, b, h))
    return c;
  for (int i = 0; i < 8; ++i)
    c.H(i / 3, i % 3) = float(h[i]);
  c.H(2, 2) = 1;
  double det = c.H(0, 0) * (c.H(1, 1) - c.H(1, 2) * c.H(2, 1)) -
               c.H(0, 1) * (c.H(1, 0) - c.H(1, 2) * c.H(2, 0)) +
               c.H(0, 2) * (c.H(1, 0) * c.H(2, 1) - c.H(1, 1) * c.H(2, 0));
  if (!isfinite(det) || fabs(det) < 1e-10)
    return c;
  c.valid = true;
  for (int i = 0; i < 4; ++i) {
    Point2D p = pixelTransformation(u[i], v[i], c);
    if (!isfinite(p.x) || !isfinite(p.y) || hypot(p.x - x[i], p.y - y[i]) > 1)
      c.valid = false;
  }
  return c;
}
Point2D pixelTransformation(float u, float v, const calibH &c) {
  Point2D p{NAN, NAN};
  if (!c.valid || !isfinite(u) || !isfinite(v))
    return p;
  double w = double(c.H(2, 0)) * u + double(c.H(2, 1)) * v + c.H(2, 2);
  if (!isfinite(w) || fabs(w) < 1e-6)
    return p;
  p.x = float((double(c.H(0, 0)) * u + double(c.H(0, 1)) * v + c.H(0, 2)) / w);
  p.y = float((double(c.H(1, 0)) * u + double(c.H(1, 1)) * v + c.H(1, 2)) / w);
  if (!isfinite(p.x) || !isfinite(p.y))
    return {NAN, NAN};
  return p;
}
void pixelTransformation(float u, float v, const calibH &c, float &x, float &y) {
  Point2D p = pixelTransformation(u, v, c);
  x = p.x;
  y = p.y;
}
