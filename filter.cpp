#include "filter.h"
#include <stdint.h>
#define M_PI_FLOAT 3.14159265358979323846f

#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5 8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9 0

float sin_approx(float x) {
  int32_t xint = x;
  if (xint < -32 || xint > 32)
    return 0.0f; // Stop here on error input (5 * 360 Deg)
  while (x > M_PI_FLOAT)
    x -= (2.0f * M_PI_FLOAT); // always wrap input angle to -PI..PI
  while (x < -M_PI_FLOAT)
    x += (2.0f * M_PI_FLOAT);
  if (x > (0.5f * M_PI_FLOAT))
    x = (0.5f * M_PI_FLOAT) -
        (x - (0.5f * M_PI_FLOAT)); // We just pick -90..+90 Degree
  else if (x < -(0.5f * M_PI_FLOAT))
    x = -(0.5f * M_PI_FLOAT) - ((0.5f * M_PI_FLOAT) + x);
  float x2 = x * x;
  return x +
         x * x2 *
             (sinPolyCoef3 +
              x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x) { return sin_approx(x + (0.5f * M_PI_FLOAT)); }

void biquadFilterInit(biquadFilter_t &filter, filterType_e type,
                      float filterFreq, uint32_t refreshRate, float Q) {

  const float omega = 2.0f * M_PI_FLOAT * filterFreq * refreshRate * 0.000001f;
  const float sn = sin_approx(omega);
  const float cs = cos_approx(omega);
  const float alpha = sn / (2.0f * Q);

  float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;
  switch (type) {
  case FILTER_LPF:
    b0 = (1 - cs) * 0.5f;
    b1 = 1 - cs;
    b2 = (1 - cs) * 0.5f;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;
    break;
  case FILTER_NOTCH:
    b0 = 1;
    b1 = -2 * cs;
    b2 = 1;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;
    break;
  }
  filter.b0 = b0 / a0;
  filter.b1 = b1 / a0;
  filter.b2 = b2 / a0;
  filter.a1 = a1 / a0;
  filter.a2 = a2 / a0;

  // zero initial samples
  filter.x1 = filter.x2 = 0;
  filter.y1 = filter.y2 = 0;
}

float biquadFilterApply(biquadFilter_t *filter, float input) {
  const float result =
      filter->b0 * input + filter->x1; // y[n+1] = b0 * x[n+1] + x1

  filter->x1 = filter->b1 * input - filter->a1 * result +
               filter->x2; // x1 = b1 * x[n] - a1 * y[n] + x2

  filter->x2 =
      filter->b2 * input - filter->a2 * result; // x2 = b2 * x[n] - a2 * y[n]

  return result;
}