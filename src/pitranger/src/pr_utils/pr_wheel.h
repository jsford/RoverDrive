#pragma once
#include <Roboteq.h>
#include <thread>

namespace pr {

constexpr int WHEELS_LEFT_ROBOTEQ_CAN_ID  = 1;
constexpr int WHEELS_RIGHT_ROBOTEQ_CAN_ID = 3;

constexpr int WHEELS_RIGHT_FRONT_CHANNEL = 1;
constexpr int WHEELS_RIGHT_REAR_CHANNEL = 2;
constexpr int WHEELS_LEFT_FRONT_CHANNEL = 1;
constexpr int WHEELS_LEFT_REAR_CHANNEL = 2;

constexpr int WHEELS_ENCODER_PPR = 7;
constexpr int WHEELS_MAX_WHEEL_RPM = 36;
constexpr int WHEELS_MOTOR_GEAR_RATIO = 188;
constexpr int WHEELS_MAX_MOTOR_RPM = WHEELS_MAX_WHEEL_RPM*WHEELS_MOTOR_GEAR_RATIO;

class WheelController {
    public:
        WheelController();
        ~WheelController();

        double set_left_rpm(double rpm);
        double set_right_rpm(double rpm);

        double set_front_right_rpm(double rpm);
        double set_front_left_rpm(double rpm);
        double set_rear_right_rpm(double rpm);
        double set_rear_left_rpm(double rpm);

        double get_front_right_rpm();
        double get_front_left_rpm();
        double get_rear_right_rpm();
        double get_rear_left_rpm();

        double get_front_right_amps();
        double get_front_left_amps();
        double get_rear_right_amps();
        double get_rear_left_amps();

        int get_front_right_encoder();
        int get_front_left_encoder();
        int get_rear_right_encoder();
        int get_rear_left_encoder();

    private:
        double fr_rpm = 0.0;
        double fl_rpm = 0.0;
        double rr_rpm = 0.0;
        double rl_rpm = 0.0;

        RoboteqDevice  left;
        RoboteqDevice right;

        void disable_watchdog(RoboteqDevice& dev);
        void disable_loop_error_detection(RoboteqDevice& dev);
        void use_velocity_mode(RoboteqDevice& dev);
        void use_quadrature_encoder(RoboteqDevice& dev, int encoder_ppr);
        void set_max_vel(RoboteqDevice& dev, int rpm);
        void set_kp(RoboteqDevice& dev, double kp);
        void set_ki(RoboteqDevice& dev, double kp);
        void set_kd(RoboteqDevice& dev, double kp);
};

} // namespace pr
