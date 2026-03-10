#pragma once

typedef struct PIDAxis {
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float kf = 0;

  float integral = 0;
  float prevGyro = 0;
  float prevSetpoint = 0;
  float dtermState = 0;
  float dtermAlpha = 0.1;
  float dtermFilter = 0;
  float dtermAlpha2 = 0.2f;

  float dMinPercent = 0.25f;
  float dMinGain = 0.02f;

  float dMinFilter = 0;
  float dMinAlpha = 0.1f;
  float dMinSetpointGain = 0;

  float itermRelaxGain = 0.002f;
  float itermRelaxMin = 0.2f;
  float itermLimit = 1.0;

  float ffState = 0;
  float ffAlpha = 0.2f;
} PIDAxis;

extern PIDAxis pid[3];

void rotateIterm(float gyro[3], float dt);

void pidController(double output[3], float setpoint[3], float gyro[3], float dt,
                   float throttle, float motorSaturation);
void pidReset(PIDAxis &pid);