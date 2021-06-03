#include "pr_utils/pr_ptu.h"
#include "pr_utils/pr_wheel.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

double rpm_to_rad_per_sec(const double rpm) {
  return rpm * 2*M_PI / 60.0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pitranger");
  ros::NodeHandle nh;

  // TODO(Jordan): Should these come from the launch file?
  const std::string wheel_odom_frame_id = "odom";
  const std::string wheel_odom_child_frame_id = "base_link";
  const double wheel_diam_m = 0.18;
  const double wheel_spacing_m = 0.62;

  pr::WheelController wheels;
  pr::PanTiltController  ptu;

  // Attach a subscriber to set wheel velocities.
  auto wheel_cb = [&wheels, wheel_diam_m, wheel_spacing_m]
    (const geometry_msgs::TwistConstPtr& msg) {
    try {
      double lin = msg->linear.x;
      double ang = msg->angular.z;

      double v_delta = std::tan(ang)*wheel_spacing_m;
      double l_mps = lin - v_delta/2.0;
      double r_mps = lin + v_delta/2.0;

      wheels.set_left_rpm(l_mps * 60.0 / wheel_diam_m);
      wheels.set_right_rpm(r_mps * 60.0 / wheel_diam_m);
    } catch(const std::exception& e) {
      fmt::print("WARN: pitranger node failed to set motor velocities.\n");
    }
  };
  auto wheel_vel_sub = nh.subscribe<geometry_msgs::Twist>("/pitranger/in/twist_cmd", 1, wheel_cb);

  // Create a publisher for the wheel odometry.
  auto wheel_odom_pub = nh.advertise<nav_msgs::Odometry>("/pitranger/out/wheel_odom", 100);

  // Track the robot state in x,y,yaw.
  double robot_x   = 0.0;
  double robot_y   = 0.0;
  double robot_yaw = 0.0;

  ros::WallTime prev_iter_time = ros::WallTime::now();
  ros::WallTime curr_iter_time = prev_iter_time;
  
  ros::Rate rate(10);
  unsigned long iter = 0;
  while( ros::ok() ) {
    // Compute wheel odometry and publish it.
    {
      const double fr_rpm = wheels.get_front_right_rpm();
      const double fl_rpm = wheels.get_front_left_rpm();
      const double rr_rpm = wheels.get_rear_right_rpm();
      const double rl_rpm = wheels.get_rear_left_rpm();
      const double  left_rpm = 0.5*(fl_rpm + rl_rpm);
      const double right_rpm = 0.5*(fr_rpm + rr_rpm);
      const double  left_rps = left_rpm * 2.0*M_PI/60.0;
      const double right_rps = right_rpm * 2.0*M_PI/60.0;

      // Get elapsed time since previous iteration.
      curr_iter_time = ros::WallTime::now();
      const double dt = (curr_iter_time-prev_iter_time).toNSec() * 1.0e-9;
      prev_iter_time = curr_iter_time;

      // Get average velocity (m/s) of the left and ride sides of the robot.
      const double  v_left_mps = left_rps*(wheel_diam_m/(2*M_PI));
      const double v_right_mps = right_rps*(wheel_diam_m/(2*M_PI));
      const double v_mps = (v_left_mps + v_right_mps) / 2.0;

      // Calculate velocity of robot in x,y,yaw.
      const double vx = std::cos(robot_yaw) * v_mps;
      const double vy = -1*std::sin(robot_yaw) * v_mps;
      const double vyaw = std::atan((v_right_mps-v_left_mps)/(wheel_spacing_m));

      // Calculate displacement in x,y,yaw.
      const double dx = vx*dt;
      const double dy = vy*dt;
      const double dyaw = vyaw*dt;

      // Update the robot state.
      robot_x += std::cos(robot_yaw)*dx - std::sin(robot_yaw)*dy;
      robot_y += std::sin(robot_yaw)*dx + std::cos(robot_yaw)*dy;
      robot_yaw += dyaw;

      // Construct the wheel odometry message header.
      nav_msgs::Odometry msg;
      msg.header.seq = iter++;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = wheel_odom_frame_id;
      msg.child_frame_id = wheel_odom_child_frame_id;

      // Construct the wheel odometry message pose.
      msg.pose.pose.position.x = robot_x;
      msg.pose.pose.position.y = robot_y;
      msg.pose.pose.position.z = 0.0;

      msg.pose.pose.orientation.x = 0.0;
      msg.pose.pose.orientation.y = 0.0;
      msg.pose.pose.orientation.z = std::sin(robot_yaw/2.0);
      msg.pose.pose.orientation.w = std::cos(robot_yaw/2.0);

      msg.pose.covariance[0*6+0] = 0.3;        // X-to-X
      msg.pose.covariance[1*6+1] = 0.3;        // Y-to-Y
      msg.pose.covariance[2*6+2] = 1000.0;     // Z-to-Z
      msg.pose.covariance[3*6+3] = 1000.0;     // Roll-to-Roll
      msg.pose.covariance[4*6+4] = 1000.0;     // Pitch-to-Pitch
      msg.pose.covariance[5*6+5] = 10.0;       // Yaw-to-Yaw

      // Construct the wheel odometry message twist.
      msg.twist.twist.linear.x = vx;
      msg.twist.twist.linear.y = 0.0;
      msg.twist.twist.linear.z = 0.0;

      msg.twist.twist.angular.x = 0.0;
      msg.twist.twist.angular.y = 0.0;
      msg.twist.twist.angular.z = vyaw;

      msg.twist.covariance[0*6+0] =   0.01;     // X-to-X
      msg.twist.covariance[1*6+1] =   10.0;     // Y-to-Y
      msg.twist.covariance[2*6+2] = 1000.0;     // Z-to-Z
      msg.twist.covariance[3*6+3] = 1000.0;     // Roll-to-Roll
      msg.twist.covariance[4*6+4] = 1000.0;     // Pitch-to-Pitch
      msg.twist.covariance[5*6+5] = 0.3;        // Yaw-to-Yaw

      wheel_odom_pub.publish(msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
