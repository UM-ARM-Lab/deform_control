#include <deformable_manipulation_experiment_params/ros_params.hpp>

#include "custom_scene/custom_scene.h"

int main(int argc, char* argv[])
{
    // Read in all ROS parameters
    ros::init(argc, argv, "deform_simulator_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    // Set some defaults for our internal configuration details
    GeneralConfig::scale = 20.0;

    if (smmap::GetDeformableType(nh) == smmap::DeformableType::CLOTH)
    {
        ViewerConfig::cameraHomePosition = btVector3(0.65f, 0.7f, 1.8f) * METERS;
        ViewerConfig::pointCameraLooksAt = btVector3(-0.35f, -0.30f, -0.1f) * METERS;

        if (smmap::GetTaskType(nh) == smmap::TaskType::CLOTH_WAFR)
        {
            ViewerConfig::cameraHomePosition = btVector3(-0.662754f, 1.35221f, 1.71409f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(-0.556761f, -0.556197f, 0.315254f) * METERS;
        }

        if (smmap::GetTaskType(nh) == smmap::TaskType::CLOTH_SINGLE_POLE)
        {
            // Video Recording
            ViewerConfig::cameraHomePosition = btVector3(0.723598f, 0.803744f, 1.70666f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(-0.276402f, -0.196255f, -0.193338f) * METERS;

            // Initial State
//            ViewerConfig::cameraHomePosition = btVector3(0.670564f, 0.0f, 1.21329f) * METERS;
//            ViewerConfig::pointCameraLooksAt = btVector3(-0.361743f, 0.0f, -0.167028f) * METERS;

            // Deadlock Predicted
//            ViewerConfig::cameraHomePosition = btVector3(0.1f, 0.0f, 1.40236f) * METERS;
//            ViewerConfig::pointCameraLooksAt = btVector3(-0.146453f, 0.0f, -0.22481f) * METERS;

            // Task Done
//            ViewerConfig::cameraHomePosition = btVector3(0.36572f, 0.631779f, 0.558584f) * METERS;
//            ViewerConfig::pointCameraLooksAt = btVector3(-0.377759f, -0.0735531f, 0.00621016f) * METERS;
        }

        if (smmap::GetTaskType(nh) == smmap::TaskType::CLOTH_DOUBLE_SLIT)
        {
            // Video Recording
            ViewerConfig::cameraHomePosition = btVector3(0.106015f, 0.0124157f, 1.87433f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(-0.112106f, 0.0, -0.129015f) * METERS;

            // Initial State
//            ViewerConfig::cameraHomePosition = btVector3(0.0412289f, 0.0f, 1.43198f) * METERS;
//            ViewerConfig::pointCameraLooksAt = btVector3(-0.129453f, 0.0f, -0.210937f) * METERS;

            // Deadlock Predicted
//            ViewerConfig::cameraHomePosition = btVector3(0.969562f, 0.0f, 0.87805f) * METERS;
//            ViewerConfig::pointCameraLooksAt = btVector3(-0.359844f, 0.0f, -0.0331455f) * METERS;

            // Task Done
//            ViewerConfig::cameraHomePosition = btVector3(0.36572f, 0.631779f, 0.558584f) * METERS;
//            ViewerConfig::pointCameraLooksAt = btVector3(-0.377759f, -0.0735531f, 0.00621016f) * METERS;
        }

//        ViewerConfig::cameraHomePosition = btVector3(9, 0, 42);
//        ViewerConfig::pointCameraLooksAt = btVector3(0, 0, 0);
    }
    else if (smmap::GetDeformableType(nh) == smmap::DeformableType::ROPE)
    {
        if (smmap::GetTaskType(nh) == smmap::ROPE_MAZE)
        {
            // From bottom
//            ViewerConfig::cameraHomePosition = btVector3(0.0f, 0.0f, 4.0f) * METERS;
//            ViewerConfig::pointCameraLooksAt = btVector3(0.0f, 0.0f, 0.0f) * METERS;

            // From Top
            ViewerConfig::cameraHomePosition = btVector3(0.555412f, 0.251752f, 3.07464f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(0.555412f, 0.251752f, 0.0f) * METERS;
        }
        else
        {
            ViewerConfig::cameraHomePosition = btVector3(0.0f, 1.0f, 3.5f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(0.0f, -0.25f, 0.0f) * METERS;
        }
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
