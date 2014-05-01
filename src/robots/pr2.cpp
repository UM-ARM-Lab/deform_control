#include "pr2.h"
#include "simulation/environment.h"
#include "thread_socket_interface.h"

static const char LEFT_GRIPPER_LEFT_FINGER_NAME[] = "l_gripper_l_finger_tip_link";
static const char LEFT_GRIPPER_RIGHT_FINGER_NAME[] = "l_gripper_r_finger_tip_link";

static const char RIGHT_GRIPPER_LEFT_FINGER_NAME[] = "r_gripper_l_finger_tip_link";
static const char RIGHT_GRIPPER_RIGHT_FINGER_NAME[] = "r_gripper_r_finger_tip_link";

// adapted from btSoftBody.cpp (btSoftBody::appendAnchor)
static void btSoftBody_appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence=1) {
    btSoftBody::Anchor a = { 0 };
    a.m_node = node;
    a.m_body = body;
    a.m_local = body->getWorldTransform().inverse()*a.m_node->m_x;
    a.m_node->m_battach = 1;
    a.m_influence = influence;
    psb->m_anchors.push_back(a);
}

// Fills in the rcontacs array with contact information between psb and pco
static void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts) {
    // custom contact checking adapted from btSoftBody.cpp and btSoftBodyInternals.h
    struct Custom_CollideSDF_RS : btDbvt::ICollide {
        Custom_CollideSDF_RS(btSoftBody::tRContactArray &rcontacts_) : rcontacts(rcontacts_) { }

        void Process(const btDbvtNode* leaf) {
            btSoftBody::Node* node=(btSoftBody::Node*)leaf->data;
            DoNode(*node);
        }

        void DoNode(btSoftBody::Node& n) {
            const btScalar m=n.m_im>0?dynmargin:stamargin;
            btSoftBody::RContact c;
            if (!n.m_battach && psb->checkContact(m_colObj1,n.m_x,m,c.m_cti)) {
                const btScalar  ima=n.m_im;
                const btScalar  imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
                const btScalar  ms=ima+imb;
                if(ms>0) {
                    // there's a lot of extra information we don't need to compute
                    // since we just want to find the contact points
#if 0
                    const btTransform&      wtr=m_rigidBody?m_rigidBody->getWorldTransform() : m_colObj1->getWorldTransform();
                    static const btMatrix3x3        iwiStatic(0,0,0,0,0,0,0,0,0);
                    const btMatrix3x3&      iwi=m_rigidBody?m_rigidBody->getInvInertiaTensorWorld() : iwiStatic;
                    const btVector3         ra=n.m_x-wtr.getOrigin();
                    const btVector3         va=m_rigidBody ? m_rigidBody->getVelocityInLocalPoint(ra)*psb->m_sst.sdt : btVector3(0,0,0);
                    const btVector3         vb=n.m_x-n.m_q; 
                    const btVector3         vr=vb-va;
                    const btScalar          dn=btDot(vr,c.m_cti.m_normal);
                    const btVector3         fv=vr-c.m_cti.m_normal*dn;
                    const btScalar          fc=psb->m_cfg.kDF*m_colObj1->getFriction();
#endif
                    c.m_node        =       &n;
#if 0
                    c.m_c0          =       ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
                    c.m_c1          =       ra;
                    c.m_c2          =       ima*psb->m_sst.sdt;
                    c.m_c3          =       fv.length2()<(btFabs(dn)*fc)?0:1-fc;
                    c.m_c4          =       m_colObj1->isStaticOrKinematicObject()?psb->m_cfg.kKHR:psb->m_cfg.kCHR;
#endif
                    rcontacts.push_back(c);
#if 0
                    if (m_rigidBody)
                            m_rigidBody->activate();
#endif
                }
            }
        }
        btSoftBody*             psb;
        btCollisionObject*      m_colObj1;
        btRigidBody*    m_rigidBody;
        btScalar                dynmargin;
        btScalar                stamargin;
        btSoftBody::tRContactArray &rcontacts;
    };

    Custom_CollideSDF_RS  docollide(rcontacts);              
    btRigidBody*            prb1=btRigidBody::upcast(pco);
    btTransform     wtr=pco->getWorldTransform();

    const btTransform       ctr=pco->getWorldTransform();
    const btScalar          timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
    const btScalar          basemargin=psb->getCollisionShape()->getMargin();
    btVector3                       mins;
    btVector3                       maxs;
    ATTRIBUTE_ALIGNED16(btDbvtVolume)               volume;
    pco->getCollisionShape()->getAabb(      pco->getWorldTransform(),
            mins,
            maxs);
    volume=btDbvtVolume::FromMM(mins,maxs);
    volume.Expand(btVector3(basemargin,basemargin,basemargin));             
    docollide.psb           =       psb;
    docollide.m_colObj1 = pco;
    docollide.m_rigidBody = prb1;

    docollide.dynmargin     =       basemargin+timemargin;
    docollide.stamargin     =       basemargin;
    psb->m_ndbvt.collideTV(psb->m_ndbvt.m_root,volume,docollide);
}

PR2SoftBodyGripper::PR2SoftBodyGripper(RaveRobotObject::Ptr robot_, OpenRAVE::RobotBase::ManipulatorPtr manip_, bool leftGripper) :
        robot(robot_), manip(manip_),
        leftFinger(robot->robot->GetLink(leftGripper ? LEFT_GRIPPER_LEFT_FINGER_NAME : RIGHT_GRIPPER_LEFT_FINGER_NAME)),
        rightFinger(robot->robot->GetLink(leftGripper ? LEFT_GRIPPER_RIGHT_FINGER_NAME : RIGHT_GRIPPER_RIGHT_FINGER_NAME)),
        origLeftFingerInvTrans(robot->getLinkTransform(leftFinger).inverse()),
        origRightFingerInvTrans(robot->getLinkTransform(rightFinger).inverse()),
        centerPt(util::toBtTransform(manip->GetTransform(), robot->scale).getOrigin()),
        closingNormal(manip->GetClosingDirection()[0],
                      manip->GetClosingDirection()[1],
                      manip->GetClosingDirection()[2]),
        toolDirection(util::toBtVector(manip->GetLocalToolDirection())), // don't bother scaling
        grabOnlyOnContact(false)
{
}

void PR2SoftBodyGripper::attach(bool left) {
    btRigidBody *rigidBody =
        robot->associatedObj(left ? leftFinger : rightFinger)->rigidBody.get();
    btSoftBody::tRContactArray rcontacts;
    getContactPointsWith(psb, rigidBody, rcontacts);
    cout << "got " << rcontacts.size() << " contacts\n";
    int nAppended = 0;
    for (int i = 0; i < rcontacts.size(); ++i) {
        const btSoftBody::RContact &c = rcontacts[i];
        KinBody::LinkPtr colLink = robot->associatedObj(c.m_cti.m_colObj);
        if (!colLink) continue;
        const btVector3 &contactPt = c.m_node->m_x;
        if (onInnerSide(contactPt, left)) {
            btSoftBody_appendAnchor(psb, c.m_node, rigidBody);
            ++nAppended;
        }
    }
    cout << "appended " << nAppended << " anchors\n";
}

void PR2SoftBodyGripper::grab() {
    if (grabOnlyOnContact) {
        attach(false);
        attach(true);
    } else {
        // the gripper should be closed
        const btVector3 midpt = 0.5 * (getInnerPt(false) + getInnerPt(true));
        // get point on cloth closest to midpt, and attach an anchor there
        // (brute-force iteration through every cloth node)
        btSoftBody::tNodeArray &nodes = psb->m_nodes;
        btSoftBody::Node *closestNode = NULL;
        btScalar closestDist;
        for (int i = 0; i < nodes.size(); ++i) {
            btSoftBody::Node &n = nodes[i];
            btScalar d2 = midpt.distance2(n.m_x);
            if (closestNode == NULL || d2 < closestDist) {
                closestNode = &n;
                closestDist = d2;
            }
        }
        // attach to left finger (arbitrary choice)
        if (closestNode)
            btSoftBody_appendAnchor(psb, closestNode, robot->associatedObj(leftFinger)->rigidBody.get());
    }
}


PR2Manager::PR2Manager(Scene &s) : scene(s), inputState() {
    loadRobot();
    initIK();
    initHaptics();
    registerSceneCallbacks();
}

void PR2Manager::registerSceneCallbacks() {
    Scene::Callback mousecb = boost::bind(&PR2Manager::processMouseInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::PUSH, mousecb);
    scene.addCallback(osgGA::GUIEventAdapter::DRAG, mousecb);

    Scene::Callback keycb = boost::bind(&PR2Manager::processKeyInput, this, _1);
    scene.addCallback(osgGA::GUIEventAdapter::KEYDOWN, keycb);
    scene.addCallback(osgGA::GUIEventAdapter::KEYUP, keycb);

    if (SceneConfig::enableRobot && SceneConfig::enableHaptics)
        scene.addPreStepCallback(boost::bind(&PR2Manager::processHapticInput, this));
}

void PR2Manager::loadRobot() {
    if (!SceneConfig::enableRobot) return;
    btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
    static const char ROBOT_MODEL_FILE[] = EXPAND(BULLETSIM_DATA_DIR) "/robot_model/pr2_with_kinect.dae";
    pr2.reset(new RaveRobotObject(scene.rave, ROBOT_MODEL_FILE, trans, GeneralConfig::scale));
    scene.env->add(pr2);
//    pr2->ignoreCollisionWith(ground->rigidBody.get()); // the robot's always touching the ground anyway
}

void PR2Manager::initIK() {
    if (!SceneConfig::enableIK || !SceneConfig::enableRobot) return;
    if (!pr2) {
        //LOG(warning) << "cannot initialize IK since the PR2 model is not yet loaded";
        return;
    }
    pr2Left = pr2->createManipulator("leftarm", SceneConfig::useFakeGrabber);
    pr2Right = pr2->createManipulator("rightarm", SceneConfig::useFakeGrabber);
    if (SceneConfig::useFakeGrabber) {
        scene.env->add(pr2Left->grabber);
        scene.env->add(pr2Right->grabber);
    }
    leftInitTrans = pr2Left->getTransform();
    rightInitTrans = pr2Right->getTransform();
}

void PR2Manager::initHaptics() {
    if (!SceneConfig::enableHaptics) return;

    connectionInit(); // socket connection for haptics

    hapTrackerLeft.reset(new SphereObject(0, 0.05*METERS, btTransform::getIdentity(), true));
    hapTrackerLeft->rigidBody->setCollisionFlags(
            hapTrackerLeft->rigidBody->getCollisionFlags()
            | btRigidBody::CF_NO_CONTACT_RESPONSE);
    hapTrackerLeft->setColor(1, 0, 0, 0.5);
    scene.env->add(hapTrackerLeft);

    hapTrackerRight.reset(new SphereObject(0, 0.05*METERS, btTransform::getIdentity(), true));
    hapTrackerRight->rigidBody->setCollisionFlags(
            hapTrackerRight->rigidBody->getCollisionFlags()
            | btRigidBody::CF_NO_CONTACT_RESPONSE);
    hapTrackerRight->setColor(0, 1, 0, 0.5);
    scene.env->add(hapTrackerRight);

    setHapticPollRate(10); // default 10 hz

    lEngaged = false;
    rEngaged = false;
    setHapticCb(hapticLeftBoth, boost::bind(&PR2Manager::toggleLeftEngaged, this));
    setHapticCb(hapticRightBoth, boost::bind(&PR2Manager::toggleRightEngaged, this));

}

void PR2Manager::processHapticInput() {
    if (!SceneConfig::enableRobot || !SceneConfig::enableHaptics)
        return;

    // throttle
    float currTime = scene.viewer.getFrameStamp()->getSimulationTime();
    if (currTime - inputState.lastHapticReadTime < 1./hapticPollRate)
        return;
    inputState.lastHapticReadTime = currTime;

    // read the haptic controllers
    btTransform trans0, trans1;
    bool buttons0[2], buttons1[2];
    if (!util::getHapticInput(trans0, buttons0, trans1, buttons1)) {
        cout << "failed to read haptic input" << endl;
        return;
    }

    // adjust the transforms
    btVector3 HAPTIC_OFFSET = btVector3(-0.2, 0, -1)*METERS + util::toBtVector(pr2->robot->GetLink("torso_lift_link")->GetTransform().trans)*METERS;
    static const btScalar HAPTIC_SCALE = 1. / 150. * METERS;
    trans0.setOrigin(trans0.getOrigin()*HAPTIC_SCALE + leftInitTrans.getOrigin() + HAPTIC_OFFSET);
    trans1.setOrigin(trans1.getOrigin()*HAPTIC_SCALE + rightInitTrans.getOrigin()+ HAPTIC_OFFSET);

    // util::sendRobotState(pr2Left->getTransform().getOrigin()/HAPTIC_SCALE - HAPTIC_OFFSET,
    //  		   pr2Right->getTransform().getOrigin()/HAPTIC_SCALE - HAPTIC_OFFSET);
    // set marker positions
    hapTrackerLeft->motionState->setKinematicPos(trans0);
    hapTrackerRight->motionState->setKinematicPos(trans1);

    // set manip positions by ik

    handleButtons(buttons0, buttons1);
    if (lEngaged) pr2Left->moveByIK(trans0, SceneConfig::enableRobotCollision, true);
    if (rEngaged) pr2Right->moveByIK(trans1, SceneConfig::enableRobotCollision, true);

}

void PR2Manager::handleButtons(bool left[], bool right[]) {

  static bool lastLeft[2] = { left[0], left[1] };
  static bool lastRight[2] = { right[0], right[1] };

  vector<HapticEvent> events;
  if (left[0] && !lastLeft[0]) events.push_back(hapticLeft0Down);
  if (!left[0] && lastLeft[0]) events.push_back(hapticLeft0Up);
  if (left[0]) events.push_back(hapticLeft0Hold);
  if (left[1] && !lastLeft[1]) events.push_back(hapticLeft1Down);
  if (!left[1] && lastLeft[1]) events.push_back(hapticLeft1Up);
  if (left[1]) events.push_back(hapticLeft1Hold);
  if (right[0] && !lastRight[0]) events.push_back(hapticRight0Down);
  if (!right[0] && lastRight[0]) events.push_back(hapticRight0Up);
  if (right[0]) events.push_back(hapticRight0Hold);
  if (right[1] && !lastRight[1]) events.push_back(hapticRight1Down);
  if (!right[1] && lastRight[1]) events.push_back(hapticRight1Up);
  if (right[1]) events.push_back(hapticRight1Hold);
  if (left[0] && left[1] && !(lastLeft[0] && lastLeft[1])) events.push_back(hapticLeftBoth);
  if (right[0] && right[1] && !(lastRight[0] && lastRight[1])) events.push_back(hapticRightBoth);

  lastLeft[0] = left[0];
  lastLeft[1] = left[1];
  lastRight[0] = right[0];
  lastRight[1] = right[1];

  BOOST_FOREACH(HapticEvent evt, events) if (hapticEvent2Func.find(evt) != hapticEvent2Func.end()) hapticEvent2Func[evt]();

}

bool PR2Manager::processKeyInput(const osgGA::GUIEventAdapter &ea) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case '1':
            inputState.moveManip0 = true; break;
        case '2':
            inputState.moveManip1 = true; break;
        case 'q':
            inputState.rotateManip0 = true; break;
        case 'w':
            inputState.rotateManip1 = true; break;
        case '!':
            cycleIKSolution(0); break;
        case '@':
            cycleIKSolution(1); break;
        }
        break;
    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '1':
            inputState.moveManip0 = false; break;
        case '2':
            inputState.moveManip1 = false; break;
        case 'q':
            inputState.rotateManip0 = false; break;
        case 'w':
            inputState.rotateManip1 = false; break;
        }
        break;
    }
    return false;
}

void PR2Manager::cycleIKSolution(int manipNum) {
    BOOST_ASSERT(manipNum == 0 || manipNum == 1);
    RaveRobotObject::Manipulator::Ptr manip =
        manipNum == 0 ? pr2Left : pr2Right;
    int &ikSolnNum = manipNum == 0 ? inputState.ikSolnNum0 : inputState.ikSolnNum1;

    vector<vector<dReal> > solns;
    if (!manip->solveAllIK(manip->getTransform(), solns)) return;
    if (ikSolnNum >= solns.size()) ikSolnNum = 0;
    cout << "arm " << manipNum << ": setting ik solution number " << ikSolnNum << endl;
    pr2->setDOFValues(manip->manip->GetArmIndices(), solns[ikSolnNum]);
    ++ikSolnNum;
}

bool PR2Manager::processMouseInput(const osgGA::GUIEventAdapter &ea) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {
        inputState.startDragging = true;
    } else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG) {
        // drag the active manipulator in the plane of view
        if (SceneConfig::enableRobot && SceneConfig::enableIK &&
              (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
              (inputState.moveManip0 || inputState.moveManip1 ||
               inputState.rotateManip0 || inputState.rotateManip1)) {
            if (inputState.startDragging) {
                inputState.dx = inputState.dy = 0;
            } else {
                inputState.dx = inputState.lastX - ea.getXnormalized();
                inputState.dy = ea.getYnormalized() - inputState.lastY;
            }
            inputState.lastX = ea.getXnormalized(); inputState.lastY = ea.getYnormalized();
            inputState.startDragging = false;

            // get our current view
            osg::Vec3d osgCenter, osgEye, osgUp;
            scene.manip->getTransformation(osgCenter, osgEye, osgUp);
            btVector3 from(util::toBtVector(osgEye));
            btVector3 to(util::toBtVector(osgCenter));
            btVector3 up(util::toBtVector(osgUp)); up.normalize();

            // compute basis vectors for the plane of view
            // (the plane normal to the ray from the camera to the center of the scene)
            btVector3 normal = (to - from).normalized();
            btVector3 yVec = (up - (up.dot(normal))*normal).normalized(); //FIXME: is this necessary with osg?
            btVector3 xVec = normal.cross(yVec);
            btVector3 dragVec = SceneConfig::mouseDragScale * (inputState.dx*xVec + inputState.dy*yVec);

            RaveRobotObject::Manipulator::Ptr manip;
            if (inputState.moveManip0 || inputState.rotateManip0)
                manip = pr2Left;
            else
                manip = pr2Right;

            btTransform origTrans = manip->getTransform();
            btTransform newTrans(origTrans);

            if (inputState.moveManip0 || inputState.moveManip1)
                // if moving the manip, just set the origin appropriately
                newTrans.setOrigin(dragVec + origTrans.getOrigin());
            else if (inputState.rotateManip0 || inputState.rotateManip1) {
                // if we're rotating, the axis is perpendicular to the
                // direction the mouse is dragging
                btVector3 axis = normal.cross(dragVec);
                btScalar angle = dragVec.length();
                btQuaternion rot(axis, angle);
                // we must ensure that we never get a bad rotation quaternion
                // due to really small (effectively zero) mouse movements
                // this is the easiest way to do this:
                if (rot.length() > 0.99f && rot.length() < 1.01f)
                    newTrans.setRotation(rot * origTrans.getRotation());
            }
            manip->moveByIK(newTrans, SceneConfig::enableRobotCollision, true);
            return true;
        }
    }
    return false;
}
