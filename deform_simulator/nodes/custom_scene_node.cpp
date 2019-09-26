#include <QApplication>
#include <arc_utilities/ros_helpers.hpp>
#include <deformable_manipulation_experiment_params/ros_params.hpp>

#include "custom_scene/custom_scene.h"
#include "custom_scene/rviz_marker_manager.h"

static constexpr int EXIT_RESTART = -50;

static std::mutex data_mtx;
static QApplication* qt_ptr = nullptr;
static CustomScene* cs_ptr = nullptr;

bool RestartSimulationCallback(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
{
    (void)req;
    (void)res;
    ROS_INFO("Restarting simulation");
    std::lock_guard<std::mutex> lock(data_mtx);
    if (qt_ptr != nullptr)
    {
        qt_ptr->exit(EXIT_RESTART);
        qt_ptr = nullptr;
    }
    if (cs_ptr != nullptr)
    {
        cs_ptr->terminate();
        while (!cs_ptr->finished())
        {
            arc_helpers::Sleep(0.02);
        }
        cs_ptr = nullptr;
    }
    return true;
}

bool TerminateSimulationCallback(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
{
    (void)req;
    (void)res;
    ROS_INFO("Terminating simulation");
    std::lock_guard<std::mutex> lock(data_mtx);
    if (qt_ptr != nullptr)
    {
        qt_ptr->exit(EXIT_SUCCESS);
        qt_ptr = nullptr;
    }
    if (cs_ptr != nullptr)
    {
        cs_ptr->terminate();
        while (!cs_ptr->finished())
        {
            arc_helpers::Sleep(0.02);
        }
        cs_ptr = nullptr;
    }
    return true;
}

int main(int argc, char* argv[])
{
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
        ViewerConfig::cameraHomePosition.setX((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/home_x", __func__) * METERS);
        ViewerConfig::cameraHomePosition.setY((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/home_y", __func__) * METERS);
        ViewerConfig::cameraHomePosition.setZ((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/home_z", __func__) * METERS);

        ViewerConfig::pointCameraLooksAt.setX((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/looks_at_x", __func__) * METERS);
        ViewerConfig::pointCameraLooksAt.setY((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/looks_at_y", __func__) * METERS);
        ViewerConfig::pointCameraLooksAt.setZ((btScalar)ROSHelpers::GetParamRequired<double>(ph, "camera/looks_at_z", __func__) * METERS);

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

    // Setup callbacks for restarting and terminating the simulation
    ros::ServiceServer restart_srv = nh.advertiseService(
                smmap::GetRestartSimulationTopic(nh), &RestartSimulationCallback);
    ros::ServiceServer terminate_srv = nh.advertiseService(
                smmap::GetTerminateSimulationTopic(nh), &TerminateSimulationCallback);
    // Tell the compiler that it's okay that these service variables are never referenced again
    (void)restart_srv;
    (void)terminate_srv;
    // Spinning is handled by CustomScene, so don't start one here

    int rv = EXIT_SUCCESS;
    do
    {
        CustomScene cs(nh, smmap::GetDeformableType(nh), smmap::GetTaskType(nh));
        cs.initialize();
        // We need to run QT and CS in their own threads.
        // QT requires that it be started in the main thread, so run CS in a new thread
        std::thread cs_thread(&CustomScene::run, &cs);

        // Start the QT managed window if needed
        if (cs.drawingOn)
        {
            QApplication qt(argc, argv);
            RVizMarkerManager marker_manager(nh, &cs);
            marker_manager.show();
            {
                std::lock_guard<std::mutex> lock(data_mtx);
                cs_ptr = &cs;
                qt_ptr = &qt;
            }
            rv = qt.exec();
        }
        cs_thread.join();
    }
    while (rv == EXIT_RESTART);
    return rv;
}
