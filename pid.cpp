#include "pid.h"
#include <math.h>

void pidReset(PIDAxis &pid) {
  pid.integral = 0;
  pid.dMinFilter = pid.dMinPercent;
}

float pidController(PIDAxis &pid, float setpoint, float gyro, float dt,
                    float throttle, float motorSaturation) {
  if (dt <= 0.0001)
    return 0;
  float error = setpoint - gyro;

  float tpaFactor = 1.0;

  if (throttle > 0.6)
    tpaFactor = 1.0 - (throttle - 0.6) * 0.7;

  float P = pid.kp * error * tpaFactor;

  float setpointDerivative = (setpoint - pid.prevSetpoint) / dt;

  float relaxFactor =
      1.0f / (1.0f + pid.itermRelaxGain * fabs(setpointDerivative));

  if (relaxFactor < pid.itermRelaxMin) {
    relaxFactor = pid.itermRelaxMin;
  }

  float antiWindup = 1.0f - motorSaturation;

  pid.integral += error * pid.ki * dt * relaxFactor * antiWindup;

  if (pid.integral > pid.itermLimit) {
    pid.integral = pid.itermLimit;
  }

  if (pid.integral < -pid.itermLimit) {
    pid.integral = -pid.itermLimit;
  }

  float filteredGyro =
      pid.dtermState + pid.dtermAlpha * (gyro - pid.dtermState);

  pid.dtermState = filteredGyro;

  float derivative = (filteredGyro - pid.prevGyro) / dt;

  pid.dtermFilter += pid.dtermAlpha2 * (derivative - pid.dtermFilter);

  float dMinGyro = fabs(pid.dtermFilter) * pid.dMinGain;
  float dMinSetpoint = fabs(setpointDerivative) * pid.dMinSetpointGain;

  float dMinDrive = fmax(dMinGyro, dMinSetpoint);

  float dMinFactor = pid.dMinPercent + (1.0f - pid.dMinPercent) * dMinDrive;

  if (dMinFactor > 1.0f)
    dMinFactor = 1.0f;

  pid.dMinFilter += pid.dMinAlpha * (dMinFactor - pid.dMinFilter);
  pid.dMinFilter = fmax(pid.dMinPercent, fmin(pid.dMinFilter, 1.0f));

  float D = -pid.kd * pid.dtermFilter * tpaFactor * pid.dMinFilter;

  pid.ffState += pid.ffAlpha * (setpointDerivative - pid.ffState);
  float F = pid.kf * pid.ffState;

  pid.prevGyro = filteredGyro;
  pid.prevSetpoint = setpoint;

  return P + pid.integral + D + F;
}