#include "update_bodies.h"
#include "dist_math.h"
#include "utils_perception.h"
#include "config_perception.h"
using namespace Eigen;

PlotLines::Ptr plots::linesAB;
PlotLines::Ptr plots::linesBA;


void initTrackingPlots() {
  plots::linesAB.reset(new PlotLines(3));
  plots::linesAB->setDefaultColor(1,1,0,1);
  plots::linesBA.reset(new PlotLines(3));
  plots::linesBA->setDefaultColor(0,1,1,1);
}

vector<btVector3> getSoftBodyNodes(BulletSoftObject::Ptr psb) {
  btAlignedObjectArray<btSoftBody::Node> nodes = psb->softBody->m_nodes;
  vector<btVector3> out(nodes.size());
  for (int i=0; i < nodes.size(); i++) out[i] = nodes[i].m_x;
  return out;
}


vector<btVector3> clothOptImpulses(BulletSoftObject::Ptr psb, const vector<btVector3>& obs) {
  const vector<btVector3> est = getSoftBodyNodes(psb);
  int nObs = obs.size();
  int nEst = est.size();

  vector<btVector3> impulses(nEst);
  vector<btVector3> abLinePoints;

  vector<int> indObsFromEst = getNNInds(est, obs);
  for (int iEst=0; iEst < nEst; iEst++) {
    const btVector3 estPt = est[iEst];
    const btVector3 obsPt = obs[indObsFromEst[iEst]];
    impulses[iEst] = TrackingConfig::impulseSize * (obsPt - estPt);
    abLinePoints.push_back(obsPt);
    abLinePoints.push_back(estPt);
  }
  plots::linesAB->setPoints(abLinePoints);


  vector<btVector3> baLinePoints;

  vector<int> indEstFromObs = getNNInds(obs, est);
  for (int iObs=0; iObs < nObs; iObs++) {
    int iEst = indEstFromObs[iObs];
    const btVector3 obsPt = obs[iObs];
    const btVector3 estPt = est[iEst];
    impulses[iEst] += TrackingConfig::impulseSize*(obsPt - estPt);
    baLinePoints.push_back(obsPt);
    baLinePoints.push_back(estPt);
  }

  plots::linesBA->setPoints(baLinePoints);

  return impulses;

}


void applyImpulses(const vector<btVector3>& impulses, BulletSoftObject::Ptr psb) {
  for (int i=0; i<impulses.size(); i++)
    psb->softBody->addForce(impulses[i],i);
}
