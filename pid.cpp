#include "pid.h"
#include <math.h>
PIDAxis pid[3];

void pidReset(PIDAxis &pid) {
  pid.integral = 0;
  pid.dMinFilter = pid.dMinPercent;
}

static void rotateVector(float v[3], float rotation[3]) {
  for (int i = 0; i < 3; i++) {
    int i_1 = (i + 1) % 3;
    int i_2 = (i + 2) % 3;
    float newV = v[i_1] + v[i_2] * rotation[i];
    v[i_2] -= v[i_1] * rotation[i];
    v[i_1] = newV;
  }
}

void rotateIterm(float gyro[3], float dt) {
  float rotationRads[3];
  for (int i = 0; i < 3; i++) {
    rotationRads[i] = gyro[i] * dt;
  }
  float v[3];
  for (int i = 0; i < 3; i++) {
    v[i] = pid[i].integral;
  }
  rotateVector(v, rotationRads);
  for (int i = 0; i < 3; i++) {
    pid[i].integral = v[i];
  }
}

void pidController(double output[3], float setpoint[3], float gyro[3], float dt,
                   float throttle, float motorSaturation) {
  if (dt <= 0.0001)
    return;

  for (int i = 0; i < 3; i++) {
    float error = setpoint[i] - gyro[i];

    float tpaFactor = 1.0;

    if (throttle > 0.6)
      tpaFactor = 1.0 - (throttle - 0.6) * 0.7;

    float P = pid[i].kp * error * tpaFactor;

    float setpointDerivative = (setpoint[i] - pid[i].prevSetpoint) / dt;

    float relaxFactor =
        1.0f / (1.0f + pid[i].itermRelaxGain * fabs(setpointDerivative));

    if (relaxFactor < pid[i].itermRelaxMin) {
      relaxFactor = pid[i].itermRelaxMin;
    }

    float antiWindup = 1.0f - motorSaturation;

    pid[i].integral += error * pid[i].ki * dt * relaxFactor * antiWindup;

    if (pid[i].integral > pid[i].itermLimit) {
      pid[i].integral = pid[i].itermLimit;
    }

    if (pid[i].integral < -pid[i].itermLimit) {
      pid[i].integral = -pid[i].itermLimit;
    }

    float filteredGyro =
        pid[i].dtermState + pid[i].dtermAlpha * (gyro[i] - pid[i].dtermState);

    pid[i].dtermState = filteredGyro;

    float derivative = (filteredGyro - pid[i].prevGyro) / dt;

    pid[i].dtermFilter +=
        pid[i].dtermAlpha2 * (derivative - pid[i].dtermFilter);

    float dMinGyro = fabs(pid[i].dtermFilter) * pid[i].dMinGain;
    float dMinSetpoint = fabs(setpointDerivative) * pid[i].dMinSetpointGain;

    float dMinDrive = fmax(dMinGyro, dMinSetpoint);

    float dMinFactor =
        pid[i].dMinPercent + (1.0f - pid[i].dMinPercent) * dMinDrive;

    if (dMinFactor > 1.0f)
      dMinFactor = 1.0f;

    pid[i].dMinFilter += pid[i].dMinAlpha * (dMinFactor - pid[i].dMinFilter);
    pid[i].dMinFilter = fmax(pid[i].dMinPercent, fmin(pid[i].dMinFilter, 1.0f));

    float D = -pid[i].kd * pid[i].dtermFilter * tpaFactor * pid[i].dMinFilter;

    pid[i].ffState += pid[i].ffAlpha * (setpointDerivative - pid[i].ffState);
    float F = pid[i].kf * pid[i].ffState;

    pid[i].prevGyro = filteredGyro;
    pid[i].prevSetpoint = setpoint[i];

    output[i] = P + pid[i].integral + D + F;
  }
}