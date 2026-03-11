#include "mixer.h"
#include <cmath>
#include <gz/msgs/actuators.pb.h>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

float mixer(gz::transport::Node::Publisher &motorPub, float output[4]) {

  float throttle = output[3];
  float roll_out = output[0];
  float pitch_out = output[1];
  float yaw_out = output[2];

  float mix[4];
  roll_out = std::clamp(roll_out, -1.0f, 1.0f);
  pitch_out = std::clamp(pitch_out, -1.0f, 1.0f);
  yaw_out = std::clamp(yaw_out, -0.6f, 0.6f);

  mix[0] = -roll_out - pitch_out - yaw_out;
  mix[1] = roll_out + pitch_out - yaw_out;
  mix[2] = roll_out - pitch_out + yaw_out;
  mix[3] = -roll_out + pitch_out + yaw_out;

  float mixMax = mix[0];
  float mixMin = mix[0];

  for (int i = 1; i < 4; i++) {
    if (mix[i] > mixMax)
      mixMax = mix[i];
    if (mix[i] < mixMin)
      mixMin = mix[i];
  }

  float mixRange = mixMax - mixMin;

  if (mixRange > 1.0f) {
    for (int i = 0; i < 4; i++) {
      mix[i] /= mixRange;
    }

    mixMax /= mixRange;
    mixMin /= mixRange;
  }

  throttle = std::clamp(throttle, -mixMin, 1.0f - mixMax);

  float m1 = throttle + mix[0];
  float m2 = throttle + mix[1];
  float m3 = throttle + mix[2];
  float m4 = throttle + mix[3];

  float maxMotor = std::max(std::max(m1, m2), std::max(m3, m4));
  float minMotor = std::min(std::min(m1, m2), std::min(m3, m4));

  float motorSaturation = 0.0f;

  if (maxMotor > 1.0f) {
    motorSaturation = maxMotor - 1.0f;
  }

  if (minMotor < 0.0f) {
    motorSaturation = std::max(motorSaturation, -minMotor);
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
  return motorSaturation;
}