#include <QApplication>
#include <deformable_manipulation_experiment_params/ros_params.hpp>

#include "custom_scene/custom_scene.h"
#include "custom_scene/rviz_marker_manager.h"

int main(int argc, char* argv[])
{
    // Initialize QT settings
    QApplication qt_app(argc, argv);

    // Read in all ROS parameters
    ros::init(argc, argv, "deform_simulator_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    // Set some defaults for our internal configuration details
    GeneralConfig::scale = 20.0;


    ViewerConfig::cameraUp = btVector3(0.0f, 0.0f, 1.0f);
    if (smmap::GetDeformableType(nh) == smmap::DeformableType::CLOTH)
    {
        ViewerConfig::cameraHomePosition = btVector3(0.65f, 0.7f, 1.8f) * METERS;
        ViewerConfig::pointCameraLooksAt = btVector3(-0.35f, -0.30f, -0.1f) * METERS;

        if (smmap::GetTaskType(nh) == smmap::TaskType::CLOTH_WAFR)
        {
            ViewerConfig::cameraHomePosition = btVector3(-0.662754f, 1.35221f, 1.71409f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(-0.556761f, -0.556197f, 0.315254f) * METERS;
        }
    }
    else if (smmap::GetDeformableType(nh) == smmap::DeformableType::ROPE)
    {
        if (smmap::GetTaskType(nh) == smmap::ROPE_MAZE)
        {
            ViewerConfig::cameraHomePosition = btVector3(0.0f, 0.0f, 6.0f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(0.0f, 0.0f, 0.0f) * METERS;
        }
        else if (smmap::GetTaskType(nh) == smmap::ROPE_ZIG_MATCH)
        {
            ViewerConfig::cameraHomePosition = btVector3(0.7f, 0.5f, 3.0f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(0.7f, 0.5f, 0.0f) * METERS;
        }
        else if (smmap::GetTaskType(nh) == smmap::ROPE_TABLE_LINEAR_MOTION)
        {
            ViewerConfig::cameraHomePosition = btVector3(0.4f, 2.6f, 2.5f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(0.2f, -0.5f, 0.0f) * METERS;
        }
        else
        {
            ViewerConfig::cameraHomePosition = btVector3(0.0f, 1.0f, 3.5f) * METERS;
            ViewerConfig::pointCameraLooksAt = btVector3(0.0f, -0.25f, 0.0f) * METERS;
        }
    }

    if (ROSHelpers::GetParam(ph, "camera_override", false))
    {
        ViewerConfig::cameraHomePosition.setX((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/home_x", __func__).GetImmutable() * METERS);
        ViewerConfig::cameraHomePosition.setY((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/home_y", __func__).GetImmutable() * METERS);
        ViewerConfig::cameraHomePosition.setZ((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/home_z", __func__).GetImmutable() * METERS);

        ViewerConfig::pointCameraLooksAt.setX((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/looks_at_x", __func__).GetImmutable() * METERS);
        ViewerConfig::pointCameraLooksAt.setY((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/looks_at_y", __func__).GetImmutable() * METERS);
        ViewerConfig::pointCameraLooksAt.setZ((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/looks_at_z", __func__).GetImmutable() * METERS);

        ViewerConfig::cameraUp.setX((btScalar)ROSHelpers::GetParam(ph, "camera/up_x", 0.0) * METERS);
        ViewerConfig::cameraUp.setY((btScalar)ROSHelpers::GetParam(ph, "camera/up_y", 0.0) * METERS);
        ViewerConfig::cameraUp.setZ((btScalar)ROSHelpers::GetParam(ph, "camera/up_z", 1.0) * METERS);
        ViewerConfig::cameraUp.normalize();
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
    const bool sync_time = false;
    cs.initialize(sync_time);
    // We need to run QT and CS in their own threads.
    // QT requires that it be started in the main thread, so run CS in a new thread
    std::thread cs_thread(&CustomScene::run, &cs);

    // Start the QT managed window
    RVizMarkerManager marker_manager(nh, cs);
    marker_manager.show();
    const int rv = qt_app.exec();

    cs_thread.join();
    return rv;
}
