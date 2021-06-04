#include "pr_utils/pr_ptu.h"
#include "pr_utils/pr_pitcam.h"
#include "pitranger/SetPanTilt.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

const double hinge_x = 0.225; // hinge displacement from center of robot.
const double hinge_z = 0.320; // height of hinge above the ground.
const double panel_length = 0.52; // hinge to camera length

std::unique_ptr<pr::PanTiltController> ptu;

using Mat4d = Eigen::Matrix<double,4,4>;
using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaternion<double>;

inline double deg2rad(double d) { return M_PI * d / 180.0; }

Mat4d yaw2mat(double yaw) {
  Mat4d M;
  double cy = std::cos(yaw);
  double sy = std::sin(yaw);
  M << cy,-sy,0,0,
       sy, cy,0,0,
        0,  0,1,0,
        0,  0,0,1;
  return M;
}

Mat4d roll2mat(double roll) {
  Mat4d M;
  double cr = std::cos(roll);
  double sr = std::sin(roll);
  M <<  1, 0,  0,0,
        0,cr,-sr,0,
        0,sr, cr,0,
        0, 0,  0,1;
  return M;
}
Mat4d pitch2mat(double pitch) {
  Mat4d M;
  double cp = std::cos(pitch);
  double sp = std::sin(pitch);
  M << cp,0,sp,0,
        0,1, 0,0,
      -sp,0,cp,0,
        0,0, 0,1;
  return M;
}
Mat4d rpy2mat(const Vec3d& rpy) {
  Mat4d Y = yaw2mat(rpy[2]);
  Mat4d P = pitch2mat(rpy[1]);
  Mat4d R = roll2mat(rpy[0]);
  return Y * P * R;
}
Mat4d translate(const Mat4d& M, const Vec3d& t) {
  Mat4d M2 = M;
  M2(0,3) += t[0];
  M2(1,3) += t[1];
  M2(2,3) += t[2];
  return M2;
}

Mat4d rotate(const Mat4d& M, const Vec3d& rpy) {
  Vec3d t = M.block<3,1>(0,3);
  Mat4d M2 = translate(M, -t);
  M2 = rpy2mat(rpy) * M2;
  M2 = translate(M2, t);
  return M2;
}

Mat4d compute_pitcam_frame(double pan_deg, double tilt_deg) {
  double  pan = -1 * deg2rad(pan_deg);
  double tilt = -1 * deg2rad(tilt_deg);

  Mat4d H = Mat4d::Identity();
  H = translate(H, Vec3d {hinge_x, 0.0, hinge_z});
  H = rotate(H, Vec3d {0.0, tilt, 0.0});
  H = translate(H, Vec3d {panel_length*std::sin(tilt), 0.0, panel_length*std::cos(tilt)});

  // Pan around local z axis. Ugh.
  Vec3d t = H.block<3,1>(0,3);
  H = translate(H, -t);
  H = H * yaw2mat(pan);
  H = translate(H, t);

  return H;
}

// Establish a service for setting the pan/tilt angle of the panel/camera.
bool set_pan_tilt(pitranger::SetPanTilt::Request &req, pitranger::SetPanTilt::Response &res) {
  ptu->set_pan_deg(req.pan_deg);
  ptu->set_tilt_deg(req.tilt_deg);
  res.pan_deg = ptu->get_pan_deg();
  res.tilt_deg = ptu->get_tilt_deg();
  return true;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pitcam");
  ros::NodeHandle nh;

  ptu = std::make_unique<pr::PanTiltController>();
  tf2_ros::TransformBroadcaster tf_broadcaster;

  // Attach a service to set the camera pan and tilt angles.
  auto ptu_service = nh.advertiseService("/pitranger/set_pan_tilt", set_pan_tilt);

  ros::Rate rate(10);
  unsigned long iter = 0;
  while( ros::ok() ) {

    // Compute the pitcam coordinate frame and broadcast it.
    {
      const double pitcam_pan  = ptu->get_pan_deg();
      const double pitcam_tilt = ptu->get_tilt_deg();
      
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = ros::Time::now();
      tf_msg.header.frame_id = "base_link";
      tf_msg.child_frame_id = "pitcam";

      const Mat4d H = compute_pitcam_frame(pitcam_pan, pitcam_tilt);
      const Quatd q(H.block<3,3>(0,0));

      tf_msg.transform.translation.x = H(0,3);
      tf_msg.transform.translation.y = H(1,3);
      tf_msg.transform.translation.z = H(2,3);

      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster.sendTransform(tf_msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
