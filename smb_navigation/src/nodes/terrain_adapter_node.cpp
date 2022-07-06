#include "smb_navigation/terrain_adapter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "terrain_adapter");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("EHRE");
    smb_navigation::TerrainAdapter terrain_adapter(nodeHandle);

    if (!terrain_adapter.isInitialized())
    {
        ROS_ERROR("[Terrain Adapter] Node not initialized. Aborting...");
        return -1;
    }
    else
    {
        ROS_INFO("[Terrain Adapter] Node initialized");
    }
    terrain_adapter.readCsv();
    terrain_adapter.run();

    return 0;
}
