#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "lama/sdm/simple_occupancy_map.h"
#include "lama/sdm/export.h"
#include "lama/types.h"
#include "lama/sdm/map.h"
#include "lama/pose2d.h"
#include <tf/tf.h>
#include <math.h>

lama::SimpleOccupancyMap *map1;



/**
 * @brief Updates the area on the map cleaned by robot
 *          
 * @param msg 
 */
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  
  lama::Pose2D poseRobot(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));

  //map1->computeRay(, , path_pos);

  // Update the map with the calculated points
  Eigen::Vector3d pos;
  pos.head<2>() = poseRobot.xy();
  map1->setFree(pos);

}


/**
 * @brief Creates a representative map of the coordenates already covered by the robot
 * 
 * @param msg 
 */
void mapCallback(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO("I heard: [%f]",msg.header.stamp.toSec());
  ROS_INFO("Width: [%u] , Height: [%u], Resolution: [%f]",msg.info.width,msg.info.height,msg.info.resolution);
  ROS_INFO("x: [%f] , y: [%f], z: [%f]",msg.info.origin.position.x,msg.info.origin.position.y,msg.info.origin.position.z);


  unsigned int width = msg.info.width;
  unsigned int height= msg.info.height;

  for (unsigned int j = 0; j < height; ++j)
  {
    for (unsigned int i = 0; i < width;  ++i)
    {
        Eigen::Vector3d coords;
        coords.x() = msg.info.origin.position.x + i * msg.info.resolution;
        coords.y() = msg.info.origin.position.y + j * msg.info.resolution;

        char value = msg.data[i + j*width];
        if (value == 100) {
            map1->setOccupied(coords);
        }
    }
  }
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "rm_cleaner");
  ros::NodeHandle n;

  map1  = new lama::SimpleOccupancyMap(0.05);

  ros::Subscriber subPose = n.subscribe("/amcl_pose", 1000, poseCallback);
  ros::Subscriber subMap = n.subscribe("/map", 1000, mapCallback);

  ros::spin();

  lama::sdm::export_to_png(*map1,"rm_cleaner.png");
  return 0;
}
