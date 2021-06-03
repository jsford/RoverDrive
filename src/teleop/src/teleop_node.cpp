#include "joystick/joystick.hh"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fmt/format.h>
#include <string>

constexpr double max_speed_mps   = 0.05;
constexpr double max_yaw_rps     = 0.1;
constexpr double wheel_spacing_m = 0.62;

const double turbo_gear0 = 1.0;
const double turbo_gear1 = 2.0;
const double turbo_gear2 = 3.0;
double turbo_gear = turbo_gear0;


std::pair<double,double> joy2twist(double joy_x, double joy_y) {
  if( joy_x == 0.0 && joy_y == 0.0 ) {
    return std::make_pair(0.0, 0.0);
  }

  const double abs_x = std::abs(joy_x);
  const double abs_y = std::abs(joy_y);
  const double mag = std::sqrt(joy_x*joy_x + joy_y*joy_y);
  const double deg = 180.0/M_PI * std::acos(abs_x) / mag;

  const double tcoeff = -1.0 + (deg / 90.0) * 2.0;
  const double turn = tcoeff * std::abs(abs_y-abs_x);
  const double mov = std::max(abs_y, abs_x);

  double rawLeft = 0.0;
  double rawRight = 0.0;

  if( (joy_x >= 0 && joy_y >= 0) || (joy_x < 0 && joy_y < 0) ) {
    rawLeft = mov;
    rawRight = turn;
  } else {
    rawLeft = turn;
    rawRight = mov;
  }

  if( joy_y < 0 ) {
    rawLeft *= -1;
    rawRight *= -1;
  }

  rawLeft *= turbo_gear;
  rawRight *= turbo_gear;

  const double ang = max_yaw_rps * std::atan((rawRight-rawLeft)/(wheel_spacing_m));
  const double lin = max_speed_mps * (rawLeft+rawRight)/2.0;
  return std::make_pair(lin, ang);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop");
  ros::NodeHandle nh;

  const std::string joy_path = "/dev/input/js0";
  Joystick joystick(joy_path, false);

  if(!joystick.isFound()) {
    fmt::print("ERROR: No joystick found.\n");
    return 1;
  }
  fmt::print("Connected to joystick: {}\n", joy_path);

  auto twist_pub = nh.advertise<geometry_msgs::Twist>("/teleop/out/twist_cmd", 10);

  double joy_x = 0;
  double joy_y = 0;
  double speed_mps = max_speed_mps;

  ros::Rate rate(30);
  while(ros::ok()) {
    JoystickEvent event;

    // Eat events so we don't lag behind.
    bool valid_event = false;
    for(int i=0; i<100; ++i) {
      JoystickEvent tmpEvent;
      // Quit if there are no events to process.
      if(!joystick.sample(&tmpEvent)) { break; }
      // Don't skip button events.
      if(tmpEvent.isButton()) { valid_event = true; event = tmpEvent; break; }
      // Don't skip events that zero an axis.
      if(tmpEvent.isAxis() && tmpEvent.value == 0) { valid_event = true; event = tmpEvent; break; }
      valid_event = true;
      event = tmpEvent;
    }

    if(valid_event) {
      if(event.isButton()) {
        //fmt::print("Button {} is {}.\n", event.number, event.value == 0 ? "up":"down");
        switch (event.number) {
          case 6:
            turbo_gear = event.value ? turbo_gear1 : turbo_gear0;
            break;
          case 7:
            turbo_gear = event.value ? turbo_gear2 : turbo_gear0;
            break;
        }
      } else if(event.isAxis()) {
        //fmt::print("Axis {} is {}.\n", event.number, event.value);
        switch (event.number) {
          case 0: // Left Joystick X-Axis
            joy_x = event.value / (double)JoystickEvent::MAX_AXES_VALUE;
            break;
          case 1: // Left Joystick Y-Axis
            joy_y = -1 * event.value / (double)JoystickEvent::MAX_AXES_VALUE;
            break;
        }
      }
    }

    double lin, ang;
    std::tie(lin, ang) = joy2twist(joy_x, joy_y);

    auto msg = geometry_msgs::Twist();
    msg.linear.x = lin;
    msg.angular.z = ang;
    twist_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
