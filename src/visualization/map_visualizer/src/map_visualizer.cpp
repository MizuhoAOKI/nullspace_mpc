#include "map_visualizer/map_visualizer.hpp"

namespace visualization
{
    MapVisualizer::MapVisualizer()
        : nh_(""), private_nh_("~")
    {
        // load parameters
        std::string map_name;
        private_nh_.param<std::string>("map_name", map_name, "");

        //// publishing topic names
        std::string rviz_3dmap_topic;
        private_nh_.param<std::string>("rviz_3dmap_topic", rviz_3dmap_topic, "/rviz_3dmap");

        // initialize publishers
        pub_rviz_3dmap_ = nh_.advertise<visualization_msgs::MarkerArray>(rviz_3dmap_topic, 10);

        // wait for a few seconds
        ros::Duration(5.0).sleep();

        // publish the 3D map only once
        if (map_name == "maze")
        {
            publishRviz3DMapOfMaze();
        }
        else if (map_name == "cylinder_garden")
        {
            publishRviz3DMapOfCylinderGarden();
        }
        else
        {
            ROS_ERROR("[MapVisualizer] Unknown map name: %s", map_name.c_str());
        }
    }

    MapVisualizer::~MapVisualizer()
    {
        // No Contents
    }

    void MapVisualizer::publishRviz3DMapOfMaze()
    {
        visualization_msgs::MarkerArray marker_array;
        ros::Time now = ros::Time::now();
    
        // === Configuration constants ===
        const double WALL_HEIGHT_OFFSET_Z = -0.5;
        const double FLOOR_HEIGHT_Z       = -0.5;
        const double FLOOR_THICKNESS      = 0.05;
    
        const double WALL_COLOR_R = 0.2, WALL_COLOR_G = 0.2, WALL_COLOR_B = 0.3, WALL_COLOR_A = 0.6;
        const double FLOOR_COLOR_R = 0.65, FLOOR_COLOR_G = 0.62, FLOOR_COLOR_B = 0.58, FLOOR_COLOR_A = 1.0;
        const double FLOOR_SCALE_X = 20.0, FLOOR_SCALE_Y = 20.0;
    
        int marker_id = 0;
    
        struct WallInfo {
            double x, y, z;
            double yaw;
            double size_x, size_y, size_z;
        };
    
        std::vector<WallInfo> walls = {
            {6.25, 5.0, 0.5, 1.57, 2.5, 0.3, 1.5},  {6.25, -5.0, 0.5, 1.57, 2.5, 0.3, 1.5},
            {3.75, 0.0, 0.5, 1.57, 2.5, 0.3, 1.5},  {3.75, -5.0, 0.5, 1.57, 2.5, 0.3, 1.5},
            {1.25, 2.5, 0.5, 1.57, 2.5, 0.3, 1.5},  {1.25, -5.0, 0.5, 1.57, 2.5, 0.3, 1.5},
            {3.75, 6.25, 0.5, 0.0, 5.0, 0.3, 1.5},  {5.0, 3.75, 0.5, 0.0, 2.5, 0.3, 1.5},
            {5.0, 1.25, 0.5, 0.0, 2.5, 0.3, 1.5},   {2.5, -1.25, 0.5, 0.0, 2.5, 0.3, 1.5},
            {5.0, -3.75, 0.5, 0.0, 2.5, 0.3, 1.5},  {2.5, -6.25, 0.5, 0.0, 2.5, 0.3, 1.5},
            {0.0, -3.75, 0.5, 0.0, 2.5, 0.3, 1.5},  {-1.25, -2.5, 0.5, 1.57, 2.5, 0.3, 1.5},
            {-3.75, 5.0, 0.5, 1.57, 2.5, 0.3, 1.5}, {-3.75, -1.25, 0.5, 1.57, 5.0, 0.3, 1.5},
            {-6.25, 3.75, 0.5, 1.57, 5.0, 0.3, 1.5},{-6.25, -5.0, 0.5, 1.57, 2.5, 0.3, 1.5},
            {-2.5, 6.25, 0.5, 0.0, 2.5, 0.3, 1.5},  {-3.75, 3.75, 0.5, 0.0, 5.0, 0.3, 1.5},
            {-2.5, 1.25, 0.5, 0.0, 2.5, 0.3, 1.5},  {-5.0, -1.25, 0.5, 0.0, 2.5, 0.3, 1.5},
            {-3.75, -6.25, 0.5, 0.0, 5.0, 0.3, 1.5},{8.75, 0.0, 0.5, 1.57, 20.0, 0.3, 1.5},
            {-8.75, 0.0, 0.5, 1.57, 20.0, 0.3, 1.5},{0.0, 8.75, 0.5, 0.0, 20.0, 0.3, 1.5},
            {0.0, -8.75, 0.5, 0.0, 20.0, 0.3, 1.5}
        };
    
        for (const WallInfo& wall : walls)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = now;
            marker.ns = "maze_walls";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
    
            marker.pose.position.x = wall.x;
            marker.pose.position.y = wall.y;
            marker.pose.position.z = wall.z + WALL_HEIGHT_OFFSET_Z;
    
            tf2::Quaternion q;
            q.setRPY(0, 0, wall.yaw);
            marker.pose.orientation = tf2::toMsg(q);
    
            marker.scale.x = wall.size_x;
            marker.scale.y = wall.size_y;
            marker.scale.z = wall.size_z;
    
            marker.color.r = WALL_COLOR_R;
            marker.color.g = WALL_COLOR_G;
            marker.color.b = WALL_COLOR_B;
            marker.color.a = WALL_COLOR_A;
    
            marker_array.markers.push_back(marker);
        }
    
        // === Floor ===
        visualization_msgs::Marker floor;
        floor.header.frame_id = "map";
        floor.header.stamp = now;
        floor.ns = "maze_floor";
        floor.id = marker_id++;
        floor.type = visualization_msgs::Marker::CUBE;
        floor.action = visualization_msgs::Marker::ADD;
    
        floor.pose.position.x = 0.0;
        floor.pose.position.y = 0.0;
        floor.pose.position.z = FLOOR_HEIGHT_Z + FLOOR_THICKNESS / 2.0;
        floor.pose.orientation.w = 1.0;
    
        floor.scale.x = FLOOR_SCALE_X;
        floor.scale.y = FLOOR_SCALE_Y;
        floor.scale.z = FLOOR_THICKNESS;
    
        floor.color.r = FLOOR_COLOR_R;
        floor.color.g = FLOOR_COLOR_G;
        floor.color.b = FLOOR_COLOR_B;
        floor.color.a = FLOOR_COLOR_A;
    
        marker_array.markers.push_back(floor);
    
        pub_rviz_3dmap_.publish(marker_array);
    }
    
    void MapVisualizer::publishRviz3DMapOfCylinderGarden()
    {
        visualization_msgs::MarkerArray marker_array;
        ros::Time now = ros::Time::now();
    
        // === Configuration constants ===
        const double WALL_HEIGHT_OFFSET_Z     = -0.5;
        const double CYLINDER_HEIGHT_OFFSET_Z = -0.5;
        const double FLOOR_HEIGHT_Z           = -0.5;
        const double FLOOR_THICKNESS          = 0.05;
    
        const double WALL_COLOR_R = 0.2, WALL_COLOR_G = 0.2, WALL_COLOR_B = 0.3, WALL_COLOR_A = 0.6;
        const double CYL_COLOR_R  = 0.3, CYL_COLOR_G  = 0.3, CYL_COLOR_B  = 0.35, CYL_COLOR_A  = 0.8;
        const double FLOOR_COLOR_R = 0.65, FLOOR_COLOR_G = 0.62, FLOOR_COLOR_B = 0.58, FLOOR_COLOR_A = 1.0;
        const double FLOOR_SCALE_X = 20.0, FLOOR_SCALE_Y = 20.0;
    
        int marker_id = 0;
    
        // === Cylinders ===
        const double CYL_RADIUS = 0.15;
        const double CYL_HEIGHT = 1.0;
        const double POP_OFFSET_Z = 0.2;
        const double START_Z = CYL_HEIGHT / 2.0 + POP_OFFSET_Z + CYLINDER_HEIGHT_OFFSET_Z;
    
        const int ROWS = 8, COLS = 8;
        const double STEP_X = 2.5, STEP_Y = 2.5;
        const double ORIGIN_X = -((COLS - 1) * STEP_X) / 2.0;
        const double ORIGIN_Y = -((ROWS - 1) * STEP_Y) / 2.0;
    
        for (int r = 0; r < ROWS; ++r)
        {
            for (int c = 0; c < COLS; ++c)
            {
                visualization_msgs::Marker cyl;
                cyl.header.frame_id = "map";
                cyl.header.stamp = now;
                cyl.ns = "cylinder_garden";
                cyl.id = marker_id++;
                cyl.type = visualization_msgs::Marker::CYLINDER;
                cyl.action = visualization_msgs::Marker::ADD;
    
                cyl.pose.position.x = ORIGIN_X + c * STEP_X;
                cyl.pose.position.y = ORIGIN_Y + r * STEP_Y;
                cyl.pose.position.z = START_Z;
                cyl.pose.orientation.w = 1.0;
    
                cyl.scale.x = CYL_RADIUS * 2;
                cyl.scale.y = CYL_RADIUS * 2;
                cyl.scale.z = CYL_HEIGHT;
    
                cyl.color.r = CYL_COLOR_R;
                cyl.color.g = CYL_COLOR_G;
                cyl.color.b = CYL_COLOR_B;
                cyl.color.a = CYL_COLOR_A;
    
                marker_array.markers.push_back(cyl);
            }
        }
    
        // === Walls ===
        struct WallInfo { double x, y, z, yaw, size_x, size_y, size_z; };
        std::vector<WallInfo> walls = {
            {10.0, 0.0, 1.0, 1.57, 20.0, 0.3, 2.0},
            {-10.0, 0.0, 1.0, 1.57, 20.0, 0.3, 2.0},
            {0.0, -10.0, 1.0, 0.0, 20.0, 0.3, 2.0},
            {0.0, 10.0, 1.0, 0.0, 20.0, 0.3, 2.0}
        };
    
        for (const WallInfo& wall : walls)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = now;
            marker.ns = "cylinder_walls";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
    
            marker.pose.position.x = wall.x;
            marker.pose.position.y = wall.y;
            marker.pose.position.z = wall.z + WALL_HEIGHT_OFFSET_Z;
    
            tf2::Quaternion q;
            q.setRPY(0, 0, wall.yaw);
            marker.pose.orientation = tf2::toMsg(q);
    
            marker.scale.x = wall.size_x;
            marker.scale.y = wall.size_y;
            marker.scale.z = wall.size_z;
    
            marker.color.r = WALL_COLOR_R;
            marker.color.g = WALL_COLOR_G;
            marker.color.b = WALL_COLOR_B;
            marker.color.a = WALL_COLOR_A;
    
            marker_array.markers.push_back(marker);
        }
    
        // === Floor ===
        visualization_msgs::Marker floor;
        floor.header.frame_id = "map";
        floor.header.stamp = now;
        floor.ns = "cylinder_floor";
        floor.id = marker_id++;
        floor.type = visualization_msgs::Marker::CUBE;
        floor.action = visualization_msgs::Marker::ADD;
    
        floor.pose.position.x = 0.0;
        floor.pose.position.y = 0.0;
        floor.pose.position.z = FLOOR_HEIGHT_Z + FLOOR_THICKNESS / 2.0;
        floor.pose.orientation.w = 1.0;
    
        floor.scale.x = FLOOR_SCALE_X;
        floor.scale.y = FLOOR_SCALE_Y;
        floor.scale.z = FLOOR_THICKNESS;
    
        floor.color.r = FLOOR_COLOR_R;
        floor.color.g = FLOOR_COLOR_G;
        floor.color.b = FLOOR_COLOR_B;
        floor.color.a = FLOOR_COLOR_A;
    
        marker_array.markers.push_back(floor);
    
        pub_rviz_3dmap_.publish(marker_array);
    }

} // namespace visualization