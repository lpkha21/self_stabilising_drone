#include "filter.h"
#include "mixer.h"
#include "pid.h"
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
#define YAW 2
#define THROTTLE 3

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

    pid[PITCH].kp = pid[ROLL].kp = 0.12;
    pid[PITCH].ki = pid[ROLL].ki = 0.035;
    pid[PITCH].kd = pid[ROLL].kd = 0.01;
    pid[PITCH].kf = pid[ROLL].kf = 0.007;
    pid[ROLL].itermRelaxGain = pid[PITCH].itermRelaxGain = 0.002f;
    pid[ROLL].itermRelaxMin = pid[PITCH].itermRelaxMin = 0.2f;
    pid[ROLL].ffAlpha = 0.2;
    pid[PITCH].ffAlpha = 0.2;
    pid[ROLL].dMinPercent = pid[PITCH].dMinPercent = 0.25f;
    pid[ROLL].dMinGain = pid[PITCH].dMinGain = 0.02f;
    pid[ROLL].dMinAlpha = pid[PITCH].dMinAlpha = 0.1f;
    pid[ROLL].dMinFilter = pid[ROLL].dMinPercent;
    pid[PITCH].dMinFilter = pid[PITCH].dMinPercent;
    pid[ROLL].dMinSetpointGain = 0.01f;
    pid[PITCH].dMinSetpointGain = 0.01f;

    pid[YAW].kp = 0.15f;
    pid[YAW].ki = 0.08f;
    pid[YAW].kf = 0.0f;
    pid[YAW].kd = 0.0f;
    pid[YAW].itermRelaxGain = 0.001f;
    pid[YAW].itermRelaxMin = 0.3f;
    pid[YAW].ffAlpha = 0.15;
    pid[YAW].dMinPercent = 0.0f;
    pid[YAW].dMinGain = 0.0f;
    pid[YAW].dMinAlpha = 0.1f;
    pid[YAW].dMinFilter = pid[YAW].dMinPercent;
    float lpfQ = 0.707f;
    float notchQ = 5;
    float refreshRate = controlDt * 1e6;
    biquadFilterInit(lowpassFilterX, FILTER_LPF, 100, refreshRate, lpfQ);
    biquadFilterInit(notchFilterX, FILTER_NOTCH, 250, refreshRate, notchQ);

    biquadFilterInit(lowpassFilterY, FILTER_LPF, 100, refreshRate, lpfQ);
    biquadFilterInit(notchFilterY, FILTER_NOTCH, 250, refreshRate, notchQ);

    biquadFilterInit(lowpassFilterZ, FILTER_LPF, 100, refreshRate, lpfQ);
    biquadFilterInit(notchFilterZ, FILTER_NOTCH, 250, refreshRate, notchQ);

    gzmsg << "Controller initialized\n";
  }

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override {
    if (_info.paused) {
      pidReset(pid[PITCH]);
      pidReset(pid[ROLL]);
      pidReset(pid[YAW]);
      return;
    }

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
    float gx = imu.angular_velocity().x();
    float gy = imu.angular_velocity().y();
    float gz = imu.angular_velocity().z();

    gx = biquadFilterApply(notchFilterX, gx);
    gy = biquadFilterApply(notchFilterY, gy);
    gz = biquadFilterApply(notchFilterZ, gz);

    gx = biquadFilterApply(lowpassFilterX, gx);
    gy = biquadFilterApply(lowpassFilterY, gy);
    gz = biquadFilterApply(lowpassFilterZ, gz);

    double ax = imu.linear_acceleration().x();
    double ay = imu.linear_acceleration().y();
    double az = imu.linear_acceleration().z();

    double throttle = (rc[2] + 1) / 2.0;
    // PID
    float gyro[3] = {gx, gy, gz};
    float output[4];
    float sp[3] = {rc[0] * RATE, rc[1] * RATE, -rc[3] * RATE};
    pidController(output, sp, gyro, controlDt, throttle);
    output[3] = throttle;
    mixer(motorPub, output);

    // gzmsg << m1 << std::endl;
    // gzmsg << -rc[PITCH] << " " << gy << "\n";
  }

private:
  Model model{kNullEntity};
  gz::msgs::IMU imuMsg;
  std::mutex imuMutex;
  biquadFilter_t lowpassFilterX;
  biquadFilter_t notchFilterX;

  biquadFilter_t lowpassFilterY;
  biquadFilter_t notchFilterY;

  biquadFilter_t lowpassFilterZ;
  biquadFilter_t notchFilterZ;
  double controlTimer = 0.0;
  double rcTimer = 0.0;
  double rcDt = 1 / 100.0;
  double freq = 1000; // Hz
  double controlDt = 1.0 / freq;
  int js_fd = -1;

  double gx_prev = 0;
  double gy_prev = 0;
  double gz_prev = 0;

  double ax_prev = 0;
  double ay_prev = 0;
  double az_prev = 0;

  float rc[4] = {0, 0, 0, 0};

  transport::Node node;
  transport::Node::Publisher motorPub;
  void OnImu(const gz::msgs::IMU &_msg) {
    std::lock_guard<std::mutex> lock(imuMutex);
    imuMsg = _msg;
  }
  float Normalize(int v) {
    const float c = 1024.0;
    return (v - c) / c;
  }
  void ReadJoystick() {
    if (js_fd < 0)
      return;

    struct input_event ev;

    while (read(js_fd, &ev, sizeof(ev)) > 0) {
      if (ev.type == EV_ABS) {
        if (ev.code <= THROTTLE) {
          rc[ev.code] = Normalize(ev.value);
        }
      }
    }
  }
};

GZ_ADD_PLUGIN(controller, gz::sim::System, controller::ISystemConfigure,
              controller::ISystemPreUpdate)