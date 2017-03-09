#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pcl_analyser/cnmap.h>
#include <pcl_analyser/costmap.h>
#include <math.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>

//Messages
 #include <nav_msgs/OccupancyGrid.h>
 #include <nav_msgs/Path.h>
 #include <geometry_msgs/PoseArray.h>
 #include <pcl_analyser/Lookuptbl.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "markerfilter.h"
//#include<pcl_analyser/RoverPath.h>
#include <pcl_analyser/pathsolver.h>


//read write
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

char* str2char( std::string str ) {
  char *c = new char[ str.length()+1 ];
  strcpy( c, str.c_str());
  return c;
}

class tester
{
  public:

    tester(ros::NodeHandle& node)
    {
       n_=node;
       cnmap_pub_ptr = new ros::Publisher(n_.advertise<nav_msgs::OccupancyGrid>("cnmap_tester", 10));
       pose_p_pub = n_.advertise<geometry_msgs::PoseStamped> ("/pose_noise",1);
       pose_f_pub = n_.advertise<geometry_msgs::PoseStamped> ("/pose_filtered",1);
       map_sub   = n_.subscribe("/map", 1, &tester::map_cb,this);
       //base_map_ptr = 0;
       cn_ptr = 0;
       new_map = false;
       LP_msg_ptr = new pcl_analyser::Lookuptbl();
       bagptr = new rosbag::Bag();

       //Filter
       filter_ptr = new MarkerFilter("cam","base",0.5,10);
    }

    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
      ROS_INFO("Map Received");
      map_msg_Ptr = msg;
      new_map = true;
    }


    void writetobag()
    {
         //start
         std::string path = ros::package::getPath("tester") + "/conf/lookuptable.bag";
         bagptr->open(path, rosbag::bagmode::Write);
         //
         LP_msg_ptr->id = 1;
         LP_msg_ptr->note = "test_map";
         LP_msg_ptr->pathes = std::vector< pcl_analyser::Lpath > (1);
         // prepare the pathes
         nav_msgs::Path p1,p2,p3;
         p1.poses = std::vector< geometry_msgs::PoseStamped > (3);
         p2.poses = std::vector< geometry_msgs::PoseStamped > (3);
         p3.poses = std::vector< geometry_msgs::PoseStamped > (3);
         p1.header.frame_id = "helllo";
         p1.poses[0].pose.position.x = 0.3;
         p2.poses[1].pose.position.y = -1.0;
         p3.poses[2].pose.position.z = 6.2;
         //prepare the Lpathes
         pcl_analyser::Lpath lp1,lp2,lp3;
         lp1.a = 2; lp1.id = 0;
         lp2.b = 3; lp2.id = 1;
         lp3.c = 10; lp3.id = 2;
         LP_msg_ptr->pathes.push_back(lp1);
         LP_msg_ptr->pathes.push_back(lp2);
         LP_msg_ptr->pathes.push_back(lp3);

        bagptr->write("lookuptable", ros::Time::now(), *LP_msg_ptr);
        bagptr->close();

    }

    void readfrombag()
    {
        std::string path = ros::package::getPath("tester") + "/conf/lookuptable.bag";
        bagptr->open(path, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(std::string("lookuptable"));
        rosbag::View view(*bagptr, rosbag::TopicQuery(topics));
        pcl_analyser::Lookuptbl::ConstPtr s;
        foreach(rosbag::MessageInstance const m, view)
        {
            s = m.instantiate<pcl_analyser::Lookuptbl>();
        }
        ROS_WARN_STREAM(s->note);

        bagptr->close();

    }

    void handle()
    {
       rate = 10.0;
       ros::Rate r(rate);     
       while(n_.ok())
       {
         if(new_map)
         {
           //base_map.UpdateFromMap(*msg);
           base_map_ptr = new costmap(map_msg_Ptr,false);
         }

         if(new_map)
         {
            if(cn_ptr == 0)
            {
              cn_ptr = new cnmap(base_map_ptr,10,false);
              cn_ptr->set_home(1.0,2.0);
              cn_ptr->update();
              //cn_ptr->test();
              new_map = false;
              //set home here, it has to get values in the world format
            }
            else
            {
               cn_ptr->update();
               new_map = false;
            }
            cn_ptr->publish_ROS(cnmap_pub_ptr);
         }
         //if(cn_ptr != 0) cn_ptr->publish_ROS(cnmap_pub_ptr);

         ros::spinOnce();
         r.sleep();
       }
    }

    void test_cnmap()
    {
      rate = 10.0;
      ros::Rate r(rate);
      cnmap cn(&n_,"/map",10,false);

      ros::Time t1,t2;
      t1 = ros::Time::now();
      while(n_.ok())
      {
         t2 = ros::Time::now();
         cn.publish_ROS();

         if((t2-t1).toSec()>3)
         {
           cn.set_home(1.0,2.0);
           cn.update();
           //cn.find_discovered(10,3);
           cn.find_discovered(2,10);
           t1 = t2;
         }
         ros::spinOnce();
         r.sleep();
      }
    }
    double rand_noise()
    {
      int n = 120 - (rand() % 40);
      return n/100.0;
    }
    void test_filter()
    {
      ros::Rate r(10);
      int n = rand() % 20;
      geometry_msgs::PoseStamped p_base,p_noise,p_filtered;
      p_base.pose.position.x = 2.0;
      p_base.pose.position.y = 0.2;
      double p_b_roll, p_b_pitch,p_b_yaw;
      p_b_roll = 0.0;
      p_b_pitch = -0.7;
      p_b_yaw = 0.4;
      p_base.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(p_b_roll,p_b_pitch,p_b_yaw);
      p_noise = p_base;
      p_noise.header.frame_id = "cam";
      p_base.header.frame_id = "cam";
      p_filtered = p_base;
      //filter_ptr->Debug_mode();
      while(ros::ok())
      {
        p_noise.pose.position.x = p_base.pose.position.x;//*rand_noise();
        p_noise.pose.position.y = p_base.pose.position.y;//*rand_noise();
        p_noise.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(p_b_roll,p_b_pitch*rand_noise(),p_b_yaw);
        p_noise.header.stamp = ros::Time::now();
        p_base.header.stamp = ros::Time::now();
        pose_p_pub.publish(p_noise);
        filter_ptr->set_v1(p_noise);
        geometry_msgs::PoseStamped filter_pose;
        double lin_var,ang_var;
        if(filter_ptr->getPose1(filter_pose,lin_var,ang_var))
        {
          ROS_INFO("lin_var:%f   ang_var:%f",lin_var,ang_var);
          pose_f_pub.publish(filter_pose);
        }
        r.sleep();
        ros::spinOnce();
      }
    }

    void test_path_solver()
    {
      ps_ptr = new pathsolver(&n_,"global_costmap","elevation_grid_",0.0,3.00,50,"pcl_analyser_node");
      ps_ptr->test();
    }

protected:
  /*state here*/
  ros::NodeHandle n_;
  pcl_analyser::Lookuptbl* LP_msg_ptr;
  rosbag::Bag *bagptr;
  // Subscribers

  ros::Subscriber map_sub;
  MarkerFilter* filter_ptr;
  // Publishers
  ros::Publisher *cnmap_pub_ptr;
  ros::Publisher pose_p_pub;
  ros::Publisher pose_f_pub;
  double rate;

private:
  costmap *base_map_ptr;
  cnmap   *cn_ptr;
  nav_msgs::OccupancyGrid::ConstPtr map_msg_Ptr;
  bool new_map;
  pathsolver* ps_ptr;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tester_node");
  ros::NodeHandle node;

  tester tt(node);
  //tt.handle();
  //tt.test_cnmap();
  //tt.test_filter();
  tt.test_path_solver();
  return 0;
}
