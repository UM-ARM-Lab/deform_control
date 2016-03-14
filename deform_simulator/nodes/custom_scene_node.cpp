#include "custom_scene/custom_scene.h"

#include <smmap/ros_params.hpp>
#include <smmap/robot_interface.hpp>

int main(int argc, char* argv[])
{
    // Read in all ROS parameters
    ros::init( argc, argv, "custom_scene", ros::init_options::NoSigintHandler );
    ros::NodeHandle nh;

    // Set some defaults for our internal configuration details
    GeneralConfig::scale = 20.0;

    if ( smmap::GetDeformableType( nh ) == smmap::DeformableType::CLOTH )
    {
        ViewerConfig::cameraHomePosition = btVector3(20, 0, 25);
        ViewerConfig::pointCameraLooksAt = btVector3(-10, 0, 10);
//        ViewerConfig::cameraHomePosition = btVector3(9, 0, 42);
//        ViewerConfig::pointCameraLooksAt = btVector3(0, 0, 0);
    }
    else if ( smmap::GetDeformableType( nh ) == smmap::DeformableType::ROPE )
    {
        ViewerConfig::cameraHomePosition = btVector3(0, 20, 70);
        ViewerConfig::pointCameraLooksAt = btVector3(0, -5, 0);
    }

    BulletConfig::dt = smmap::RobotInterface::DT;
    BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup( GeneralConfig() );
    parser.addGroup( ViewerConfig() );
    parser.addGroup( BulletConfig() );
    parser.addGroup( SceneConfig() );

    // Read in any user supplied configuration parameters
    parser.read( argc, argv );

    // TODO move these settings to a CustomSceneConfig class?
    CustomScene cs( nh, smmap::GetDeformableType( nh ), smmap::GetTaskType( nh ) );
    cs.run();

    return 0;
}
