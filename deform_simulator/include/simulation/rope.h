#pragma once
#include "simulation/environment.h"
#include "simulation/basicobjects.h"
#include <btBulletDynamicsCommon.h>
#include <vector>

class CapsuleRope : public CompoundObject<BulletObject> {
private:
  float angStiffness;
  float angDamping;
  float linDamping;
  float angLimit;
public:
  typedef boost::shared_ptr<CapsuleRope> Ptr;
  std::vector<boost::shared_ptr<btCollisionShape> > shapes;
  std::vector<BulletConstraint::Ptr> joints;
  btScalar radius;
  int nLinks;

  CapsuleRope(const std::vector<btVector3>& ctrlPoints, float radius_, float angStiffness_=.1f, float angDamping_=1, float linDamping_=.75f, float angLimit_=.4f);
  void init();
  void destroy();
  std::vector<btVector3> getNodes();
  std::vector<btVector3> getControlPoints();
};
