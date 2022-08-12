#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "nav_msgs/OccupancyGrid.h"

#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "quadtree.h"
#include "geometry_msgs/Pose.h"


void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

  std_msgs::Header header = msg->header;
  
  // ros map saver
  std::string mapdatafile = "testmap.pgm";
  ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  FILE* out = fopen(mapdatafile.c_str(), "w");
  if (!out) {
    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    return;
  }

    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
    msg->info.resolution, msg->info.width, msg->info.height);
    for(unsigned int y = 0; y < msg->info.height; y++) {
       for(unsigned int x = 0; x < msg->info.width; x++) {
          unsigned int i = x + (msg->info.height - y - 1) * msg->info.width;
          if (msg->data[i] >= 0 && msg->data[i] <= 0) {
              //occ [0,0.1)
              fputc(254, out);
          }
          else if (msg->data[i] <= 100 && msg->data[i] >= 100) {
              //occ (0.65,1]
              fputc(000, out);
          }
          else {
              //occ [0.1,0.65]
              fputc(205, out);
          }
       }
    }
    fclose(out);
// Ende map saver
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_converter");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map", 1000, mapConvert);

  ros::spin();

  return 0;
}
