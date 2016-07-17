#include <smmap_experiment_params/ros_params.hpp>

#include "custom_scene/custom_scene.h"

int main(int argc, char* argv[])
{
    // Read in all ROS parameters
    ros::init(argc, argv, "custom_scene", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    // Set some defaults for our internal configuration details
    GeneralConfig::scale = 20.0;

    if (smmap::GetDeformableType(nh) == smmap::DeformableType::CLOTH)
    {
        ViewerConfig::cameraHomePosition = btVector3(0.65, 0.7, 1.8) * METERS;
        ViewerConfig::pointCameraLooksAt = btVector3(-0.35, -0.30, -0.1) * METERS;

        if (smmap::GetTaskType(nh) == smmap::TaskType::WAFR)
        {
            ViewerConfig::cameraHomePosition = btVector3(-0.662754, 1.35221, 1.61409) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(-0.556761, -0.556197, 0.215254) * METERS;
        }
//        ViewerConfig::cameraHomePosition = btVector3(9, 0, 42);
//        ViewerConfig::pointCameraLooksAt = btVector3(0, 0, 0);
    }
    else if (smmap::GetDeformableType(nh) == smmap::DeformableType::ROPE)
    {
        ViewerConfig::cameraHomePosition = btVector3(0, 1, 3.5) * METERS;
        ViewerConfig::pointCameraLooksAt = btVector3(0, -0.25, 0) * METERS;
    }

    BulletConfig::dt = (float)smmap::GetRobotControlPeriod(nh);
    BulletConfig::internalTimeStep = 0.01f;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(ViewerConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());

    // Read in any user supplied configuration parameters
    parser.read(argc, argv);

    // TODO move these settings to a CustomSceneConfig class?
    CustomScene cs(nh, smmap::GetDeformableType(nh), smmap::GetTaskType(nh));
    const bool syncTime = false;
    cs.run(ROSHelpers::GetParam(ph, "start_bullet_viewer", true), syncTime);

    return 0;
}
