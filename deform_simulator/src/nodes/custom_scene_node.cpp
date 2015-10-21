#include "custom_scene/custom_scene.h"

int main(int argc, char *argv[])
{
    // Read in all ROS parameters
    ros::init( argc, argv, "custom_scene", ros::init_options::NoSigintHandler );

    // Set some defaults for our internal configuration details
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.002;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup( GeneralConfig() );
    parser.addGroup( ViewerConfig() );
    parser.addGroup( BulletConfig() );
    parser.addGroup( SceneConfig() );

    // Read in any user supplied configuration parameters
    // TODO read in any ROS params that we want
    parser.read(argc, argv);

    ros::NodeHandle nh;

    // TODO move these settings to a CustomSceneConfig class
    CustomScene cs = CustomScene(
            CustomScene::DeformableType::ROPE,
            CustomScene::TaskType::COVERAGE,
            nh );
    cs.run();

    return 0;
}