#include "custom_scene/custom_scene.h"

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.002;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(ViewerConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());

    parser.read(argc, argv);

    CustomScene cs = CustomScene(
            CustomScene::DeformableType::ROPE,
            CustomScene::TaskType::COVERAGE);
    cs.run();

    return 0;
}
