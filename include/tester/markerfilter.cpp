#include "markerfilter.h"

/* TO CMAKELIST  Mind the addresses
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(markerfilter
   include/${PROJECT_NAME}/markerfilter.cpp
)

target_link_libraries(markerfilter
   ${catkin_LIBRARIES}
 )

target_link_libraries(your_node markerfilter
   ${catkin_LIBRARIES}
 )

 */

MarkerFilter::MarkerFilter(std::string cam_frame_, std::string IF_frame_,float angle_threshold_,size_t Filter_size)
{
  cam_frame = cam_frame_;
  IF_frame = IF_frame_;
  angle_threshold = angle_threshold_;
  cam_IF_time = ros::Time(0);
  vec_size = Filter_size;
  debug = false;
  time_telorance = ros::Duration(0.80);
}

void MarkerFilter::Debug_mode()
{
  ROS_WARN("####### DEBUG MODE #####");
  debug = true;
}

tf::StampedTransform MarkerFilter::TfTransFromPose(geometry_msgs::PoseStamped p)
{
  tf::StampedTransform out;
  out.setOrigin(tf::Vector3(p.pose.position.x,p.pose.position.y,p.pose.position.z));
  tf::Quaternion quat;
  tf::quaternionMsgToTF(p.pose.orientation,quat);
  out.setRotation(quat);
  //ROS_ERROR_STREAM(p.pose.orientation);
  //ROS_INFO("x:%f  y:%f  z:%f   w:%f", quat.x(),quat.y(),quat.z(),quat.w());
  return out;
}
geometry_msgs::Quaternion MarkerFilter::QuatXYZ(tf::Vector3 v)
{
  geometry_msgs::Quaternion msg;
  msg.x = v.getX();
  msg.y = v.getY();
  msg.z = v.getZ();
  msg.w = sqrt(1-pow(msg.x,2)-pow(msg.y,2)-pow(msg.z,2));
  //ROS_WARN_STREAM(msg);
  return msg;
}

void MarkerFilter::set_v1(geometry_msgs::PoseStamped p)
{
  if(!v1.empty())
  {
    if(ros::Time::now()-cam_IF_time > ros::Duration(0.2))
    {
      update_cam_IF(3);

    }
    if(ros::Time::now() - last_pose_time > time_telorance )
    {
      ROS_WARN("MarkerFilter: too old pose time diff: %f , the vector is reset", (ros::Time::now() - last_pose_time).toSec());
      v1.clear();
    }
  }
  else update_cam_IF(3);
  frame_id = p.header.frame_id;
  ROS_INFO_COND(debug,"vector size is %d",(int)v1.size());
  tf::Transform pose_if = cam_IF_trans*TfTransFromPose(p); //PoseIF = Cam-IF * pose_camera;
  tf::Matrix3x3 rotMat;
  rotMat.setRotation(pose_if.getRotation());
  double roll,pitch,yaw;
//  //ROS_WARN("x: %f  y: %f   z: %f", pose_if.getOrigin().getX(),pose_if.getOrigin().getY(),pose_if.getOrigin().getZ());

  rotMat.getRPY(roll,pitch,yaw);
// // ROS_INFO("roll:%f   pitch:%f   yaw:%f",roll,pitch,yaw);
  if (fabs(roll) < angle_threshold && fabs(pitch) < angle_threshold)
  {
    std::vector <tf::Vector3> tmp_vec;
    tmp_vec.push_back(tf::Vector3(p.pose.position.x,p.pose.position.y,p.pose.position.z));
    tmp_vec.push_back(tf::Vector3(p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z));
    v1.push_back(tmp_vec);
    last_pose_time = ros::Time::now();
    if(v1.size()>vec_size+3)
      v1.erase(v1.begin(),v1.begin()+1);
  }
  else ROS_ERROR_COND(debug,"MarkerFilter: bad pose rejected ");
}

void MarkerFilter::set_v2(geometry_msgs::PoseStamped p)
{
  if(!v2.empty())
  {
    if(ros::Time::now()-cam_IF_time > ros::Duration(0.2))
    {
      update_cam_IF(3);

    }
    if(ros::Time::now() - last_pose_time > time_telorance )
    {
      ROS_WARN("MarkerFilter: too old pose time diff: %f , the vector is reset", (ros::Time::now() - last_pose_time).toSec());
      v2.clear();
    }
  }
  else update_cam_IF(3);
  frame_id = p.header.frame_id;
  ROS_INFO_COND(debug,"vector size is %d",(int)v2.size());
  tf::Transform pose_if = cam_IF_trans*TfTransFromPose(p); //PoseIF = Cam-IF * pose_camera;
  tf::Matrix3x3 rotMat;
  rotMat.setRotation(pose_if.getRotation());
  double roll,pitch,yaw;
//  //ROS_WARN("x: %f  y: %f   z: %f", pose_if.getOrigin().getX(),pose_if.getOrigin().getY(),pose_if.getOrigin().getZ());

  rotMat.getRPY(roll,pitch,yaw);
// // ROS_INFO("roll:%f   pitch:%f   yaw:%f",roll,pitch,yaw);
  if (fabs(roll) < angle_threshold && fabs(pitch) < angle_threshold)
  {
    std::vector <tf::Vector3> tmp_vec;
    tmp_vec.push_back(tf::Vector3(p.pose.position.x,p.pose.position.y,p.pose.position.z));
    tmp_vec.push_back(tf::Vector3(p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z));
    v2.push_back(tmp_vec);
    last_pose_time = ros::Time::now();
    if(v2.size()>vec_size+3)
      v2.erase(v2.begin(),v2.begin()+1);
  }
  else ROS_ERROR_COND(debug,"MarkerFilter: bad pose rejected ");
}


bool MarkerFilter::getPose1(geometry_msgs::PoseStamped& out,double& lin_var, double& ang_var)
{
  if(v1.size()> 0)
  {
    tf::Vector3 VEC(v1[0][0]);
    tf::Vector3 ANG(v1[0][1]);
    int v1_filter_size = std::min((int)v1.size(),(int)vec_size);
    for(int i=1; i < v1_filter_size; i++)
    {
      VEC+=v1[i][0];
      ANG+=v1[i][1];
//        ROS_WARN("vec i: %d",i);
//        PRINT3(v1[i][0]);
//        PRINT3(v1[i][1]);
//        ROS_ERROR("-------");
    }
    VEC/=v1_filter_size;
    ANG/=v1_filter_size;
    lin_var = 0.0;
    ang_var = 0.0;
    for(int i=0;i < v1_filter_size; i++)
    {
      lin_var += pow((v1[i][0].length()-VEC.length()),2);
      ang_var += pow((v1[i][1].length()-ANG.length()),2);
    }
//      ROS_INFO("AVERAGE");
//      PRINT3(VEC);
//      PRINT3(ANG);
    out.pose.position.x = VEC.getX();
    out.pose.position.y = VEC.getY();
    out.pose.position.z = VEC.getZ();
    out.pose.orientation = QuatXYZ(ANG);
  }
  else
  {
    ROS_ERROR_COND(debug,"MarkerFilter: vector is empty");
    return false;
  }
  out.header.frame_id = frame_id;
  out.header.stamp = ros::Time::now();
  return true;
}

bool MarkerFilter::getPose2(geometry_msgs::PoseStamped& out,double& lin_var, double& ang_var)
{
  if(v2.size()> 0)
  {
    tf::Vector3 VEC(v2[0][0]);
    tf::Vector3 ANG(v2[0][1]);
    int v2_filter_size = std::min((int)v2.size(),(int)vec_size);
    for(int i=1; i < v2_filter_size; i++)
    {
      VEC+=v2[i][0];
      ANG+=v2[i][1];
//        ROS_WARN("vec i: %d",i);
//        PRINT3(v2[i][0]);
//        PRINT3(v2[i][1]);
//        ROS_ERROR("-------");
    }
    VEC/=v2_filter_size;
    ANG/=v2_filter_size;
    lin_var = 0.0;
    ang_var = 0.0;
    for(int i=0;i < v2_filter_size; i++)
    {
      lin_var += pow((v2[i][0].length()-VEC.length()),2);
      ang_var += pow((v2[i][1].length()-ANG.length()),2);
    }
//      ROS_INFO("AVERAGE");
//      PRINT3(VEC);
//      PRINT3(ANG);
    out.pose.position.x = VEC.getX();
    out.pose.position.y = VEC.getY();
    out.pose.position.z = VEC.getZ();
    out.pose.orientation = QuatXYZ(ANG);
  }
  else
  {
    ROS_ERROR_COND(debug,"MarkerFilter: vector is empty");
    return false;
  }
  out.header.frame_id = frame_id;
  out.header.stamp = ros::Time::now();
  return true;
}

void MarkerFilter::update_cam_IF(unsigned int Filter_sample)
{
  using namespace tf;
  TransformListener listener;
  StampedTransform transform;
  StampedTransform TRANS;
  unsigned int sample = 0;
  int attempt = 0;
  if(!listener.waitForTransform(IF_frame,cam_frame,ros::Time(0),ros::Duration(2)))
  {
    ROS_ERROR_STREAM("MarkerFilter: could not find the frame from " << IF_frame << " to " << cam_frame);
  }

  while(sample < Filter_sample )
  {
    try{
        listener.lookupTransform(IF_frame, cam_frame, ros::Time(0), transform);
        sample++;
        if (sample == 1)
          TRANS = transform;
        else
        {
          TRANS.setOrigin((transform.getOrigin()+TRANS.getOrigin())/2);
        }
        attempt = 0;
    }
    catch (tf::TransformException ex){
      ROS_ERROR_COND(attempt%50 == 0,"%s",ex.what());
      ros::Duration(0.01).sleep();
      attempt ++;
    }

    if (attempt > 100)
    {
      ROS_FATAL("Transform map to baselink is lost");
      break;
    }
  }
  cam_IF_trans = TRANS;
  cam_IF_time = ros::Time::now();
}
