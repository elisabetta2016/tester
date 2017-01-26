#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pcl_analyser/cnmap.h>
#include <pcl_analyser/costmap.h>
#include <math.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <rosbag/bag.h>
//Messages
 #include <nav_msgs/OccupancyGrid.h>
 #include <nav_msgs/Path.h>
 #include <geometry_msgs/PoseArray.h>
 #include <pcl_analyser/Lookuptbl.h>
 #include <rosbag/view.h>
 #include <std_msgs/Int32.h>
 #include <std_msgs/String.h>

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
       cnmap_pub_ptr = new ros::Publisher(n_.advertise<nav_msgs::OccupancyGrid>("cnmap", 10));
       map_sub   = n_.subscribe("/map", 1, &tester::map_cb,this);
       //base_map_ptr = 0;
       cn_ptr = 0;
       new_map = false;
       char* path = str2char(ros::package::getPath("tester") + "/conf/lookuptable.txt");
       file_ptr = new std::fstream(path,std::ios::out | std::ios::in | std::ios::trunc | std::ios::binary);
       LP_msg_ptr = new pcl_analyser::Lookuptbl();
       bagptr = new rosbag::Bag();

    }

    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
      ROS_INFO("Map Received");
      map_msg_Ptr = msg;
      new_map = true;
    }

    void read_write_vector()
    {
       char* path = str2char(ros::package::getPath("tester") + "/conf/lookuptable.txt");
       //std::ofstream myfile;
       //myfile.open (str2char(path));
       //myfile << "Writing this to a file.\n";
       //myfile.close();
       int myints[] = {16,2,77,29};
       int newints[] = {};
       std::vector<int> myVector(myints, myints + sizeof(myints) / sizeof(int) );
       std::vector<int> newVector(newints, newints + sizeof(newints) / sizeof(int) );

       //std::vector<std::vector<float> > myVector( 10, std::vector<float> ( 3, -12.78 ) );
       //std::vector<std::vector<float> > newVector;


       std::ofstream FILE(path,std::ios::out);//path,std::ios::out | std::ofstream::binary

       std::copy(myVector.begin(),myVector.end(),std::ostreambuf_iterator<char>(FILE));
       FILE.flush(); // required here


       std::ifstream INFILE(path,std::ios::in);
       std::istreambuf_iterator<char> iter(INFILE);

       std::copy(iter,std::istreambuf_iterator<char>{},std::back_inserter(newVector));



       for(int i=0; i< newVector.size();i++)
           ROS_INFO_STREAM(newVector[i]);
    }
    void re_wr()
    {
        char* path = str2char(ros::package::getPath("tester") + "/conf/lookuptable.txt");
        geometry_msgs::PoseArray p1,p2,p3;
        p1.poses = std::vector< geometry_msgs::Pose > (3);
        p2.poses = std::vector< geometry_msgs::Pose > (3);
        p3.poses = std::vector< geometry_msgs::Pose > (3);
        p1.header.frame_id = "helllo";
        p1.poses[0].position.x = 0.3;
        p2.poses[1].position.y = -1.0;
        p3.poses[2].position.z = 6.2;

        std::map <size_t,geometry_msgs::PoseArray> LP,LPnew; //lookuptable
        LP[1] = p1;
        LP[2] = p2;
        LP[3] = p3;

        //std::fstream FILE(path,std::ios::out | std::ios::in | std::ios::trunc | std::ios::binary );//path,std::ios::out | std::ofstream::binary
        if(!file_ptr->is_open())
        {
          ROS_ERROR("error reading the file");
          return;
        }

        file_ptr->write((char *)&LP,sizeof(std::map <size_t,geometry_msgs::PoseArray>));
        file_ptr->seekg(0);
        file_ptr->read((char *)&LPnew,sizeof(std::map <size_t,geometry_msgs::PoseArray>));

        ROS_WARN_STREAM(LPnew[1].header.frame_id);

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
         //read_write_vector();
         //re_wr(); crash
         writetobag();
         ros::spinOnce();
         r.sleep();
         readfrombag();
       }
    }

protected:
  /*state here*/
  ros::NodeHandle n_;
  std::fstream* file_ptr;
  pcl_analyser::Lookuptbl* LP_msg_ptr;
  rosbag::Bag *bagptr;
  // Subscribers
  ros::Subscriber map_sub;

  // Publishers
  ros::Publisher *cnmap_pub_ptr;
  double rate;

private:
  costmap *base_map_ptr;
  cnmap   *cn_ptr;
  nav_msgs::OccupancyGrid::ConstPtr map_msg_Ptr;
  bool new_map;


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tester_node");
  ros::NodeHandle node;

  tester tt(node);
  tt.handle();
  return 0;
}
