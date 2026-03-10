#include "mixer.h"
#include "pid.h"
#include <algorithm>
#include <fcntl.h>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <linux/input.h>
#include <mutex>
#include <unistd.h>

#define _USE_MATH_DEFINES
#include <cmath>

#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define YAW 3

#define RATE 3

using namespace gz;
using namespace sim;

class controller : public System,
                   public ISystemConfigure,
                   public ISystemPreUpdate {
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &_ecm, EventManager &) override {
    this->model = Model(_entity);

    if (!this->model.Valid(_ecm)) {
      gzerr << "Controller plugin should be attached to a model\n";
      return;
    }

    bool ok = node.Subscribe("/imu", &controller::OnImu, this);

    if (ok)
      gzmsg << "Subscribed to /imu\n";
    else
      gzerr << "Failed to subscribe to /imu\n";

    this->motorPub =
        this->node.Advertise<gz::msgs::Actuators>("/x500/command/motor_speed");

    js_fd = open("/dev/input/event21", O_RDONLY | O_NONBLOCK);

    if (js_fd < 0) {
      gzerr << "Failed to open joystick\n";
    } else {
      gzmsg << "Joystick opened\n";
    }

    gzmsg << "Controller initialized\n";

    pid.pitch.kp = pid.roll.kp = 0.25;
    pid.pitch.ki = pid.roll.ki = 0.12;
    pid.pitch.kd = pid.roll.kd = 0.003;
    pid.pitch.kf = pid.roll.kf = 0.004;
    pid.roll.itermRelaxGain = pid.pitch.itermRelaxGain = 0.002f;
    pid.roll.itermRelaxMin = pid.pitch.itermRelaxMin = 0.2f;
    pid.roll.ffAlpha = 0.2;
    pid.pitch.ffAlpha = 0.2;
    pid.roll.dMinPercent = pid.pitch.dMinPercent = 0.25f;
    pid.roll.dMinGain = pid.pitch.dMinGain = 0.02f;
    pid.roll.dMinAlpha = pid.pitch.dMinAlpha = 0.1f;
    pid.roll.dMinFilter = pid.roll.dMinPercent;
    pid.pitch.dMinFilter = pid.pitch.dMinPercent;
    pid.roll.dMinSetpointGain = 0.01f;
    pid.pitch.dMinSetpointGain = 0.01f;

    pid.yaw.kp = 0.15;
    pid.yaw.ki = 0.08;
    pid.yaw.kd = 0;
    pid.yaw.kf = 0.002;
    pid.yaw.itermRelaxGain = 0.001f;
    pid.yaw.itermRelaxMin = 0.3f;
    pid.yaw.ffAlpha = 0.15;
    pid.yaw.dMinPercent = 0.0f;
    pid.yaw.dMinGain = 0.0f;
    pid.yaw.dMinAlpha = 0.1f;
    pid.yaw.dMinFilter = pid.yaw.dMinPercent;
  }

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override {
    if (_info.paused) {
      pidReset(pid.pitch);
      pidReset(pid.roll);
      pidReset(pid.yaw);
      return;
    }
    // 1000 Hz controll loop
    double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0) {
      return;
    }

    controlTimer += dt;
    rcTimer += dt;
    if (controlTimer < controlDt) {
      return;
    }
    controlTimer -= controlDt;

    // radio input
    if (rcTimer >= rcDt) {
      ReadJoystick();
      rcTimer -= rcDt;
    }
    // reading the IMU
    gz::msgs::IMU imu;
    {
      std::lock_guard<std::mutex> lock(imuMutex);
      imu = imuMsg;
    }
    double gx = imu.angular_velocity().x();
    double gy = imu.angular_velocity().y();
    double gz = imu.angular_velocity().z();

    double ax = imu.linear_acceleration().x();
    double ay = imu.linear_acceleration().y();
    double az = imu.linear_acceleration().z();

    double throttle = (rc[THROTTLE] + 1) / 2.0;
    // PID
    double roll_out = pidController(pid.roll, rc[ROLL] * RATE, gx, controlDt,
                                    throttle, motorSaturation);
    double pitch_out = pidController(pid.pitch, rc[PITCH] * RATE, gy, controlDt,
                                     throttle, motorSaturation);
    double yaw_out = pidController(pid.yaw, -rc[YAW] * RATE, gz, controlDt,
                                   throttle, motorSaturation);

    motorSaturation = mixer(motorPub, throttle, roll_out, pitch_out, yaw_out);
    // gzmsg << m1 << std::endl;
    // gzmsg << -rc[PITCH] << " " << gy << "\n";
  }

private:
  Model model{kNullEntity};
  gz::msgs::IMU imuMsg;
  std::mutex imuMutex;
  double controlTimer = 0.0;
  double rcTimer = 0.0;
  double rcDt = 1 / 50.0;
  double freq = 1000; // Hz
  double controlDt = 1.0 / freq;
  int js_fd = -1;
  PID pid;
  float motorSaturation = 0.0f;

  double gx_prev = 0;
  double gy_prev = 0;
  double gz_prev = 0;

  double ax_prev = 0;
  double ay_prev = 0;
  double az_prev = 0;

  double rc[4] = {0, 0, 0, 0};

  transport::Node node;
  transport::Node::Publisher motorPub;
  void OnImu(const gz::msgs::IMU &_msg) {
    std::lock_guard<std::mutex> lock(imuMutex);
    imuMsg = _msg;
  }
  double Normalize(int v) {
    const double c = 1024.0;
    return (v - c) / c;
  }
  void ReadJoystick() {
    if (js_fd < 0)
      return;

    struct input_event ev;

    while (read(js_fd, &ev, sizeof(ev)) > 0) {
      if (ev.type == EV_ABS) {
        if (ev.code <= YAW) {
          rc[ev.code] = Normalize(ev.value);
        }
      }
    }
  }
};

GZ_ADD_PLUGIN(controller, gz::sim::System, controller::ISystemConfigure,
              controller::ISystemPreUpdate)