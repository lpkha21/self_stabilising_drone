#include <stdint.h>

typedef struct {
  float b0, b1, b2;
  float a1, a2;

  float x1, x2;
  float y1, y2;
} biquadFilter_t;

typedef enum {
  FILTER_LPF,
  FILTER_NOTCH,
} filterType_e;

void biquadFilterInit(biquadFilter_t &filter, filterType_e type,
                      float filterFreq, uint32_t refreshRate, float Q);

float biquadFilterApply(biquadFilter_t *filter, float input);