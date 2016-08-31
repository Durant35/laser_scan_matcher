/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * *** Modified also to publish horizontal velocities:                       ***
 * *** WARNING: This is not the original laser_scan_matcher from CCNY,       ***
 * *** it's been slightly modified in a dirty fashion.                       ***
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */
 
#ifndef LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
#define LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
 
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/time.h>
 
#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max
 
namespace scan_tools1
{
 
// inputs
const std::string scan_topic_  = "scan";
const std::string cloud_topic_ = "cloud";
const std::string odom_topic_  = "odom";
const std::string imu_topic_   = "imu";
 
// outputs
const std::string pose_topic_ = "pose2D";
 
typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
 
class LaserScanMatcher
{
  private:
 
    // **** ros  
 
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
 
    ros::Subscriber scan_subscriber_;
    ros::Subscriber cloud_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber imu_subscriber_;
 
    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
 
    tf::Transform base_to_laser_;
    tf::Transform laser_to_base_;
 
    ros::Publisher  test_pub_;
    ros::Publisher  pose_publisher_;
    ros::Publisher  vel_publisher_;
 
    // **** parameters
 
    std::string base_frame_;
    std::string fixed_frame_;
    double cloud_range_min_;
    double cloud_range_max_;
    bool publish_tf_;
    bool publish_pose_;
    bool publish_vel_;
 
    bool use_cloud_input_;
 
    // **** What predictions are available to speed up the ICP?
    // 1) imu - [theta] from imu yaw angle - /odom topic
    // 2) odom - [x, y, theta] from wheel odometry - /imu topic
    // 3) alpha_beta - [x, y, theta] from simple tracking filter - no topic req.
    // If more than one is enabled, priority is imu > odom > alpha_beta
 
    bool use_imu_;
    bool use_odom_;
    bool use_alpha_beta_;
 
    double alpha_;
    double beta_;
 
    // **** state variables
 
    bool initialized_;
 
    bool received_imu_;
    bool received_odom_;
 
    boost::mutex mutex_;
 
    tf::Transform w2b_; // world-to-base tf (pose of base frame)
 
    double v_x_;  // velocity estimated by the alpha-beta tracker
    double v_y_;  
    double v_a_;
 
    double vel_x_, vel_y_, vel_a_;  // simple velocity estimates
    double last_vel_x_, last_vel_y_, last_vel_a_;
 
    ros::Time last_icp_time_;
 
    double latest_imu_yaw_;
    double last_imu_yaw_;
 
    nav_msgs::Odometry latest_odom_;
    nav_msgs::Odometry last_odom_;
    nav_msgs::Odometry odom;
 
    std::vector<double> a_cos_;
    std::vector<double> a_sin_;
 
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;
 
    // **** methods
 
    void processScan(LDP& curr_ldp_scan, const ros::Time& time);
 
    void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                              LDP& ldp);
    void PointCloudToLDP(const PointCloudT::ConstPtr& cloud,
                               LDP& ldp);
 
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void cloudCallback (const PointCloudT::ConstPtr& cloud);
 
    void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg);
    void imuCallback (const sensor_msgs::ImuPtr& imu_msg);
 
    void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    bool getBaseToLaserTf (const std::string& frame_id);
    void initParams();
 
    void getPrediction(double& pr_ch_x, double& pr_ch_y,
                       double& pr_ch_a, double dt);
 
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
    double getYawFromQuaternion(const tf::Quaternion& quaternion);
    void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
 
  public:
 
    LaserScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~LaserScanMatcher();
};
 
} //namespace
 
#endif // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
