#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cnmap.h>
#include <costmap.h>
#include <math.h>

//Messages
 #include <nav_msgs/OccupancyGrid.h>

class tester
{
  public:

    tester(ros::NodeHandle& node)
    {
       n_=node;
       cnmap_pub = n_.advertise<nav_msgs::OccupancyGrid>("cnmap", 10);
    }

    void handle()
    {
       rate = 10.0;
       ros::Rate r(rate);
       while(n_.ok())
       {


         r.sleep();
         ros::spinOnce();
       }

    }

  protected:
  /*state here*/
  ros::NodeHandle n_;

  // Subscribers
  ros::Subscriber subFromJoystick_;

  // Publishers
  ros::Publisher cnmap_pub;
  double rate;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tester_node");
  ros::NodeHandle node;

  tester tt(node);
  tt.handle();
  return 0;
}
