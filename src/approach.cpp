#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include<pcl_analyser/pathsolver.h>

//Messages
 #include <nav_msgs/OccupancyGrid.h>

class approach
{
  public:

    approach(ros::NodeHandle& node)
    {
       n_=node;
       solver_ptr = new pathsolver(&n_,"global_costmap","elevation_costmap",0.0,3.00,50,"pcl_analyser_node");
    }
    void handle()
    {
       rate = 10.0;
       ros::Rate r(rate);
       solver_ptr->drone_approach();
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
  ros::Subscriber subUAVpose_;

  // Publishers
  ros::Publisher cnmap_pub;

  geometry_msgs::PoseStamped drone_pose;
  pathsolver *solver_ptr;
  double rate;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tester_node");
  ros::NodeHandle node;

  approach tt(node);
  tt.handle();
  return 0;
}
