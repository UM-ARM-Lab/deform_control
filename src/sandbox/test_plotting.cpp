#include "simulation/plotting.h"
#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
using boost::shared_ptr;
#include <iostream>
using namespace std;

int main(int argc, char* argv[]) {
  SceneConfig::enableIK = SceneConfig::enableHaptics = SceneConfig::enableRobot = false;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);

  Scene s;

  if (argc < 2) {
    cout << "must supply an argument 1,2,3,..." << endl;
    return 0;
  }

  if (!strcmp(argv[1],"1")) {
    shared_ptr<PlotPoints> plot1(new PlotPoints(20));
    vector<btVector3> pts;
    pts.push_back(btVector3(0,0,0));
    pts.push_back(btVector3(1,0,0));
    pts.push_back(btVector3(0,1,0));
    pts.push_back(btVector3(1,1,0));
    pts.push_back(btVector3(0,0,1));
    pts.push_back(btVector3(1,0,1));
    pts.push_back(btVector3(0,1,1));
    pts.push_back(btVector3(1,1,1));
    plot1->setPoints(pts);
    s.env->add(plot1);

  }  


  if (!strcmp(argv[1],"2")) {
    shared_ptr<PlotLines> plot2(new PlotLines());
    vector<btVector3> pts;

    btVector3 v(1,2,3);

    pts.push_back(btVector3(0,0,0));
    pts.push_back(btVector3(0,0,0)+v);
    pts.push_back(btVector3(0,0,1));
    pts.push_back(btVector3(0,0,1)+v);
    pts.push_back(btVector3(0,1,0));
    pts.push_back(btVector3(0,1,0)+v);
    pts.push_back(btVector3(0,1,1));
    pts.push_back(btVector3(0,1,1)+v);

    pts.push_back(btVector3(1,0,0));
    pts.push_back(btVector3(1,0,0)+v);
    pts.push_back(btVector3(1,0,1));
    pts.push_back(btVector3(1,0,1)+v);
    pts.push_back(btVector3(1,1,0));
    pts.push_back(btVector3(1,1,0)+v);
    pts.push_back(btVector3(1,1,1));
    pts.push_back(btVector3(1,1,1)+v);

    plot2->setPoints(pts);
    s.env->add(plot2);


  }  

  s.startViewer();
  s.startLoop();
}
