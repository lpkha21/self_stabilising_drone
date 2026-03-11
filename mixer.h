#pragma once
#include <gz/msgs/actuators.pb.h>
#include <gz/transport/Node.hh>

float mixer(gz::transport::Node::Publisher &motorPub, float output[4]);