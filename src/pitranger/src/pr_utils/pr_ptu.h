#pragma once
#include <string>
#include <fmt/format.h>
#include <Roboteq.h>

namespace pr {

constexpr char PTU_SERIAL_PORT[] = "/dev/ttyTHS2";

// Tilt Axis Configuration
constexpr int PTU_TILT_ENCODER_ANALOG_INPUT      = 1;
constexpr int PTU_TILT_FWD_LIMIT_DIGITAL_INPUT   = 5;
constexpr int PTU_TILT_REV_LIMIT_DIGITAL_INPUT   = 6;

constexpr int    PTU_TILT_ENCODER_MIN       = 1560;
constexpr int    PTU_TILT_ENCODER_CENTER    = 2300;
constexpr int    PTU_TILT_ENCODER_MAX       = 3250;

constexpr double PTU_TILT_MIN_DEG            = -76.0;
constexpr double PTU_TILT_CENTER_DEG         =   0.0;
constexpr double PTU_TILT_MAX_DEG            =  90.0;

constexpr int    PTU_TILT_MOTOR_RPM         = 100;
constexpr double PTU_TILT_PID[]             = {16.0, 0.0, 0.0};
constexpr int    PTU_TILT_DEFAULT_CMD       = PTU_TILT_ENCODER_CENTER;
constexpr int    PTU_TILT_CHANNEL           = 1;

constexpr double PTU_TILT_EPSILON_DEG       = 2.0;


// Pan Axis Configuration
constexpr int PTU_PAN_ENCODER_ANALOG_INPUT      = 2;
constexpr int PTU_PAN_FWD_LIMIT_DIGITAL_INPUT   = 4;
constexpr int PTU_PAN_REV_LIMIT_DIGITAL_INPUT   = 3;

constexpr int    PTU_PAN_ENCODER_MIN        = 1600;
constexpr int    PTU_PAN_ENCODER_CENTER     = 2270;
constexpr int    PTU_PAN_ENCODER_MAX        = 3000;

constexpr double PTU_PAN_MIN_DEG            = -67.0;
constexpr double PTU_PAN_CENTER_DEG         =   0.0;
constexpr double PTU_PAN_MAX_DEG            =  67.0;

constexpr int    PTU_PAN_MOTOR_RPM          = 50;
constexpr double PTU_PAN_PID[]              = {3.0, 0.1, 1.6};
constexpr int    PTU_PAN_DEFAULT_CMD        = PTU_PAN_ENCODER_MIN;
constexpr int    PTU_PAN_CHANNEL            = 2;

constexpr double PTU_PAN_EPSILON_DEG        = 2.0;

class PanTiltController {
    public:
        PanTiltController();
        ~PanTiltController();

        int set_pan_deg(int deg);
        int set_tilt_deg(int deg);

        [[nodiscard]] int get_pan_deg();
        [[nodiscard]] int get_tilt_deg();

    private:
        RoboteqDevice device;
        int pan_cmd  =  PTU_PAN_DEFAULT_CMD;
        int tilt_cmd = PTU_TILT_DEFAULT_CMD;

        void use_position_mode(int chan); 
        void set_position_mode_velocity(int chan, int rpm);

        void config_absolute_encoder(int chan, int analog_in, int mv_min, int mv_mid, int mv_max);
        void config_limit_switches(int chan, int fwd_in, int rev_in);

        void disable_watchdog();
        void disable_loop_error_detection(int chan); 

        void set_kp(int chan, float kp);
        void set_ki(int chan, float ki);
        void set_kd(int chan, float kd);

        void set_motor_cmd(int chan, int cmd);
        int get_encoder_value(int chan);
};

}
