#ifndef MARKERFILTER_H
#define MARKERFILTER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <tf/transform_listener.h>

#include <iostream>

//Messages
 #include <geometry_msgs/PoseStamped.h>



void PRINT3(tf::Vector3 vec)
{
  ROS_INFO("x:%f   y:%f   z:%f ", vec.getX(),vec.getY(),vec.getZ());
}

class MarkerFilter
{
public:
  MarkerFilter(std::string cam_frame_, std::string IF_frame_,float angle_threshold_,size_t Filter_size);

  ~MarkerFilter()
  {
    ROS_WARN("MarkerFilter: Instant destructed, Ciao!");
  }
  void Debug_mode();

  tf::StampedTransform TfTransFromPose(geometry_msgs::PoseStamped p);

  geometry_msgs::Quaternion QuatXYZ(tf::Vector3 v);

  void set_v1(geometry_msgs::PoseStamped p);

  void set_v2(geometry_msgs::PoseStamped p);

  bool getPose1(geometry_msgs::PoseStamped& out,double& lin_var, double& ang_var);
  bool getPose2(geometry_msgs::PoseStamped& out,double& lin_var, double& ang_var);
private:
  size_t vec_size;
  std::vector <std::vector <tf::Vector3> > v1; // <tf::Vector3 [xyz],tf::Vector3 [RPY]>
  std::vector <std::vector <tf::Vector3> > v2;
  std::string cam_frame;
  std::string IF_frame;
  tf::StampedTransform cam_IF_trans;
  ros::Time cam_IF_time,last_pose_time;
  float angle_threshold;
  bool debug;
  std::string frame_id;
  ros::Duration time_telorance;
protected:
  void update_cam_IF(unsigned int Filter_sample);
};

#endif // MARKERFILTER_H
