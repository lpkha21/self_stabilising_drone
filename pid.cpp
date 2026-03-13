#include "pid.h"
#include <math.h>

PIDAxis pid[3];

void pidReset(PIDAxis &pid) {
  pid.integral = 0;
  pid.dMinFilter = pid.dMinPercent;
  pid.setpointFiltered = 0.0f;
  pid.prevPD = 0.0f;
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

void pidController(float output[4], float setpoint[3], float gyro[3], float dt,
                   float throttle) {

  if (dt <= 0.0001f) {
    return;
  }

  rotateIterm(gyro, dt);

  for (int i = 0; i < 3; i++) {

    /* ------------------------------------------------ */
    /* Setpoint Low Pass Filter                        */
    /* ------------------------------------------------ */

    pid[i].setpointFiltered +=
        pid[i].setpointAlpha * (setpoint[i] - pid[i].setpointFiltered);

    float target = pid[i].setpointFiltered;

    float error = target - gyro[i];

    /* ------------------------------------------------ */
    /* Throttle PID Attenuation                        */
    /* ------------------------------------------------ */

    float tpaBreakpoint = 0.65f;
    float tpaStrength = 0.4f;

    float tpaFactor = 1.0f;

    if (throttle > tpaBreakpoint) {

      float t = (throttle - tpaBreakpoint) / (1.0f - tpaBreakpoint);

      tpaFactor = 1.0f - t * tpaStrength;
    }

    /* ------------------------------------------------ */
    /* P Term                                          */
    /* ------------------------------------------------ */

    float P = pid[i].kp * error * tpaFactor;

    /* ------------------------------------------------ */
    /* Feedforward derivative                          */
    /* ------------------------------------------------ */

    float setpointDerivative = (target - pid[i].prevSetpoint) / dt;

    /* ------------------------------------------------ */
    /* I-Term Relax                                    */
    /* ------------------------------------------------ */

    float relaxFactor =
        1.0f / (1.0f + pid[i].itermRelaxGain * fabsf(setpointDerivative));

    if (relaxFactor < pid[i].itermRelaxMin)
      relaxFactor = pid[i].itermRelaxMin;

    bool allowIterm = true;

    if ((pid[i].integral > 0 && error > 0) ||
        (pid[i].integral < 0 && error < 0)) {
      allowIterm = false;
    }

    if (allowIterm) {
      pid[i].integral += error * pid[i].ki * dt * relaxFactor;
    }

    if (pid[i].integral > pid[i].itermLimit)
      pid[i].integral = pid[i].itermLimit;

    if (pid[i].integral < -pid[i].itermLimit)
      pid[i].integral = -pid[i].itermLimit;

    /* ------------------------------------------------ */
    /* Gyro filtering                                  */
    /* ------------------------------------------------ */

    float filteredGyro =
        pid[i].dtermState + pid[i].dtermAlpha * (gyro[i] - pid[i].dtermState);

    pid[i].dtermState = filteredGyro;

    float derivative = (filteredGyro - pid[i].prevGyro) / dt;

    pid[i].dtermFilter +=
        pid[i].dtermAlpha2 * (derivative - pid[i].dtermFilter);

    /* ------------------------------------------------ */
    /* Dynamic D-Min                                   */
    /* ------------------------------------------------ */

    float dMinGyro = fabsf(pid[i].dtermFilter) * pid[i].dMinGain;
    float dMinSetpoint = fabsf(setpointDerivative) * pid[i].dMinSetpointGain;

    float dMinDrive = fmaxf(dMinGyro, dMinSetpoint);

    float dMinFactor =
        pid[i].dMinPercent + (1.0f - pid[i].dMinPercent) * dMinDrive;

    if (dMinFactor > 1.0f)
      dMinFactor = 1.0f;

    pid[i].dMinFilter += pid[i].dMinAlpha * (dMinFactor - pid[i].dMinFilter);

    if (pid[i].dMinFilter < pid[i].dMinPercent)
      pid[i].dMinFilter = pid[i].dMinPercent;

    if (pid[i].dMinFilter > 1.0f)
      pid[i].dMinFilter = 1.0f;

    float D = -pid[i].kd * pid[i].dtermFilter * tpaFactor * pid[i].dMinFilter;

    /* ------------------------------------------------ */
    /* Feedforward                                     */
    /* ------------------------------------------------ */

    pid[i].ffState += pid[i].ffAlpha * (setpointDerivative - pid[i].ffState);

    float F = pid[i].kf * pid[i].ffState * tpaFactor;

    /* ------------------------------------------------ */
    /* PD limiter (ArduPilot style)                    */
    /* ------------------------------------------------ */

    float pd = P + D;

    if (fabsf(pd) > pid[i].pdLimit) {

      float scale = pid[i].pdLimit / fabsf(pd);

      P *= scale;
      D *= scale;

      pd = P + D;
    }

    /* ------------------------------------------------ */
    /* Slew Rate Limiter                               */
    /* ------------------------------------------------ */

    float delta = (pd - pid[i].prevPD) / dt;

    if (fabsf(delta) > pid[i].slewLimit) {

      pd = pid[i].prevPD + copysignf(pid[i].slewLimit * dt, delta);
    }

    pid[i].prevPD = pd;

    /* ------------------------------------------------ */
    /* Final Output                                    */
    /* ------------------------------------------------ */

    output[i] = pd + pid[i].integral + F;

    pid[i].prevGyro = filteredGyro;
    pid[i].prevSetpoint = target;
  }
}