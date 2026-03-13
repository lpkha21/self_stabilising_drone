#include "mixer.h"
#include <cmath>
#include <gz/msgs/actuators.pb.h>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

void mixer(gz::transport::Node::Publisher &motorPub, float output[4]) {

  float roll_out = output[0];
  float pitch_out = output[1];
  float yaw_out = output[2];
  float throttle = output[3];

  // roll_out = 0;
  // pitch_out = 0;
  // yaw_out = 0;

  float mix[4];
  float clampVal = 0.2f;
  roll_out = std::clamp(roll_out, -clampVal, clampVal);
  pitch_out = std::clamp(pitch_out, -clampVal, clampVal);
  yaw_out = std::clamp(yaw_out, -clampVal, clampVal);

  mix[0] = -roll_out - pitch_out - yaw_out;
  mix[1] = roll_out + pitch_out - yaw_out;
  mix[2] = roll_out - pitch_out + yaw_out;
  mix[3] = -roll_out + pitch_out + yaw_out;

  float throttleLimit = 0.85f;
  throttle = std::min(throttle, throttleLimit);

  float m1 = throttle + mix[0];
  float m2 = throttle + mix[1];
  float m3 = throttle + mix[2];
  float m4 = throttle + mix[3];

  float maxMotor = std::max(std::max(m1, m2), std::max(m3, m4));

  if (maxMotor > 1.0f) {
    float excess = maxMotor - 1.0f;
    m1 -= excess;
    m2 -= excess;
    m3 -= excess;
    m4 -= excess;
  }
  float minMotor = std::min(std::min(m1, m2), std::min(m3, m4));

  // Shift up if below 0.0
  if (minMotor < 0.0f) {
    float deficit = -minMotor;
    m1 += deficit;
    m2 += deficit;
    m3 += deficit;
    m4 += deficit;
  }

  m1 = std::clamp(m1, 0.0f, 1.0f);
  m2 = std::clamp(m2, 0.0f, 1.0f);
  m3 = std::clamp(m3, 0.0f, 1.0f);
  m4 = std::clamp(m4, 0.0f, 1.0f);
  // sending motor data
  gz::msgs::Actuators msg;
  msg.add_velocity(m1 * 1500); // front-right
  msg.add_velocity(m2 * 1500); // back-left
  msg.add_velocity(m3 * 1500); // front-left
  msg.add_velocity(m4 * 1500); // back-right
  motorPub.Publish(msg);
}