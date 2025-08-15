#include "map_visualizer/map_visualizer.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_visualizer");
    visualization::MapVisualizer map_visualizer;
    ros::spin();
    return 0;
};
