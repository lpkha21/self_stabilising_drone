#include "mixer.h"
#include <cmath>
#include <gz/msgs/actuators.pb.h>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

float mixer(gz::transport::Node::Publisher &motorPub, double output[4]) {

  double throttle = output[3];
  double roll_out = output[0];
  double pitch_out = output[1];
  double yaw_out = output[2];

  double m1 = throttle - roll_out - pitch_out - yaw_out;
  double m2 = throttle + roll_out + pitch_out - yaw_out;
  double m3 = throttle + roll_out - pitch_out + yaw_out;
  double m4 = throttle - roll_out + pitch_out + yaw_out;
  float motorSaturation = 0.0f;
  float scale = 1.0f;

  float maxMotor = fmax(fmax(m1, m2), fmax(m3, m4));

  if (maxMotor > 1.0f) {
    motorSaturation = fmax(motorSaturation, maxMotor - 1.0f);
    m1 /= maxMotor;
    m2 /= maxMotor;
    m3 /= maxMotor;
    m4 /= maxMotor;
  }

  if (motorSaturation > 1.0f) {
    motorSaturation = 1.0f;
  }

  m1 = std::clamp(m1, 0.0, 1.0);
  m2 = std::clamp(m2, 0.0, 1.0);
  m3 = std::clamp(m3, 0.0, 1.0);
  m4 = std::clamp(m4, 0.0, 1.0);
  // sending motor data
  gz::msgs::Actuators msg;
  msg.add_velocity(m1 * 1500); // front-right
  msg.add_velocity(m2 * 1500); // back-left
  msg.add_velocity(m3 * 1500); // front-left
  msg.add_velocity(m4 * 1500); // back-right
  motorPub.Publish(msg);
  return motorSaturation;
}