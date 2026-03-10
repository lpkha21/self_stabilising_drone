#pragma once
#include <gz/msgs/actuators.pb.h>
#include <gz/transport/Node.hh>

float mixer(gz::transport::Node::Publisher &motorPub, double throttle,
            double roll_out, double pitch_out, double yaw_out);