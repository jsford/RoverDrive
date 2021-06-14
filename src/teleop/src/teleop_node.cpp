#include "joystick/joystick.hh"
#include "pitranger/SetPanTilt.h"
#include "pitranger/PitCamCapture.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fmt/format.h>
#include <string>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

constexpr double max_speed_mps   = 0.05;
constexpr double max_yaw_rps     = 0.1;
constexpr double wheel_spacing_m = 0.62;

const double turbo_gear0 = 1.0;
const double turbo_gear1 = 2.0;
const double turbo_gear2 = 3.0;
double turbo_gear = turbo_gear0;

// Mappings for the wired Nintendo Switch PowerA controller.
const int BUTTON_A = 2;
const int BUTTON_B = 1;
const int BUTTON_X = 3;
const int BUTTON_Y = 0;
const int TRIGGER_ZL = 6;
const int TRIGGER_ZR = 7;
const int LEFT_JOY_X = 0;
const int LEFT_JOY_Y = 1;
const int RIGHT_JOY_X =  2;
const int RIGHT_JOY_Y =  3;
const int BUTTON_HOME = 12;
const int BUTTON_CIRCLE = 13;
const int BUTTON_PLUS =  9;
const int BUTTON_MINUS =  8;
const int DPAD_X = 4;
const int DPAD_Y = 5;


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

bool set_pan_tilt(ros::ServiceClient& client, int pan, int tilt) {
  pitranger::SetPanTilt srv;
  srv.request.pan_deg = pan;
  srv.request.tilt_deg = tilt;
  return client.call(srv);
}

pitranger::PitCamCapture::Response pitcam_capture(ros::ServiceClient& client) {
  pitranger::PitCamCapture srv;
  srv.request.exposure_us = 67041; // Use autoexposure.
  client.call(srv);
  return srv.response;
}

void save_png(pitranger::PitCamCapture::Response& resp) {
  static int count = 0;
  std::string fname = fmt::format("teleop-{}_{}_{}_{}.jpg", count++, resp.exposure_us, resp.pan_deg, resp.tilt_deg);
  stbi_write_jpg(fname.c_str(), resp.image.width, resp.image.height, 3, resp.image.data.data(), 80);
  fmt::print("Teleop: Image saved as {}\n", fname);
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

  auto set_pan_tilt_client = nh.serviceClient<pitranger::SetPanTilt>("pitcam/set_pan_tilt");
  set_pan_tilt_client.waitForExistence();
  
  auto capture_client = nh.serviceClient<pitranger::PitCamCapture>("pitcam/capture");
  capture_client.waitForExistence();

  int  pan_idx = 4;
  int tilt_idx = 5;
  std::vector<int> pans {-60, -45, -30, -15, 0, 15, 30, 45, 60};
  std::vector<int> tilts {-70, -60, -50, -40, -30, -20, -10, 0, 30, 60, 90};
  set_pan_tilt(set_pan_tilt_client, pans[pan_idx], tilts[tilt_idx]);

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
          case TRIGGER_ZL:
            turbo_gear = event.value ? turbo_gear1 : turbo_gear0;
            break;
          case TRIGGER_ZR:
            turbo_gear = event.value ? turbo_gear2 : turbo_gear0;
            break;
	  case BUTTON_HOME:
	    if(event.value) {
              auto img = pitcam_capture(capture_client);
              fmt::print("Received image with dimensions {}x{}\n", img.image.width, img.image.height);
              save_png(img);
	    }
	    break;
        }
      } else if(event.isAxis()) {
        //fmt::print("Axis {} is {}.\n", event.number, event.value);
        switch (event.number) {
          case LEFT_JOY_X: // Left Joystick X-Axis
            joy_x = event.value / (double)JoystickEvent::MAX_AXES_VALUE;
            break;
          case LEFT_JOY_Y: // Left Joystick Y-Axis
            joy_y = -1 * event.value / (double)JoystickEvent::MAX_AXES_VALUE;
            break;
          case DPAD_X:
            if(event.value < 0) { pan_idx = std::max<int>(pan_idx-1, 0); }
            else if(event.value > 0) { pan_idx = std::min<int>(pan_idx+1, pans.size()-1); }
            set_pan_tilt(set_pan_tilt_client, pans[pan_idx], tilts[tilt_idx]);
            break;
          case DPAD_Y:
            if(event.value < 0) { tilt_idx = std::max<int>(tilt_idx-1, 0); }
            else if(event.value > 0) { tilt_idx = std::min<int>(tilt_idx+1, tilts.size()-1); }
            set_pan_tilt(set_pan_tilt_client, pans[pan_idx], tilts[tilt_idx]);
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
