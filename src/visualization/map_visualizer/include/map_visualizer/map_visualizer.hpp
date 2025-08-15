#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace visualization
{
    class MapVisualizer
    {
        public:
            MapVisualizer();
            ~MapVisualizer();
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;

            // publisher
            //// rviz 3D map
            ros::Publisher pub_rviz_3dmap_;
            void publishRviz3DMapOfMaze(); // for map: maze
            void publishRviz3DMapOfCylinderGarden(); // for map: cylinder_garden
    };
} // namespace visualization
