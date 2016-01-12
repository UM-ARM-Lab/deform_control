#include "custom_scene/custom_scene.h"

#include <smmap/ros_params.hpp>

int main(int argc, char* argv[])
{
    // Read in all ROS parameters
    ros::init( argc, argv, "custom_scene", ros::init_options::NoSigintHandler );

    // Set some defaults for our internal configuration details
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(20, 5, 70);
    ViewerConfig::pointCameraLooksAt = btVector3(0, 5, 0);
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup( GeneralConfig() );
    parser.addGroup( ViewerConfig() );
    parser.addGroup( BulletConfig() );
    parser.addGroup( SceneConfig() );

    // Read in any user supplied configuration parameters
    parser.read(argc, argv);

    ros::NodeHandle nh;

    // TODO move these settings to a CustomSceneConfig class?
    CustomScene cs( nh, smmap::GetDeformableType( nh ), smmap::GetTaskType( nh ) );
    cs.run();

    return 0;
}
