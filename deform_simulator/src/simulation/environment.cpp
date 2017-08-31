#include "simulation/environment.h"
#include "utils/config.h"
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
// Struct used to create and interact with an OpenSceneGraph.                 //
////////////////////////////////////////////////////////////////////////////////

OSGInstance::OSGInstance()
{
    root = new osg::Group();
}

////////////////////////////////////////////////////////////////////////////////
// Struct used to create and interact with a Bullet dynamics world.           //
////////////////////////////////////////////////////////////////////////////////

BulletInstance::BulletInstance()
{
    broadphase = new btDbvtBroadphase();
//    broadphase = new btAxisSweep3(btVector3(-2*METERS, -2*METERS, -1*METERS), btVector3(2*METERS, 2*METERS, 3*METERS));
    collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->getDispatchInfo().m_enableSPU = true;

    softBodyWorldInfo.m_broadphase = broadphase;
    softBodyWorldInfo.m_dispatcher = dispatcher;
    softBodyWorldInfo.m_sparsesdf.Initialize();
}

BulletInstance::~BulletInstance()
{
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
}

void BulletInstance::setGravity(const btVector3 &gravity)
{
    dynamicsWorld->setGravity(gravity);
    softBodyWorldInfo.m_gravity = gravity;
}

////////////////////////////////////////////////////////////////////////////////
// Struct used to manage the collections of objects in both Bullet and        //
// OpenSceneGraph at the same time.                                           //
////////////////////////////////////////////////////////////////////////////////

Environment::Environment(BulletInstance::Ptr bullet_, OSGInstance::Ptr osg_)
    : bullet(bullet_)
    , osg(osg_)
{}

Environment::~Environment()
{
    for (ConstraintList::iterator i = constraints.begin(); i != constraints.end(); ++i)
    {
        (*i)->destroy();
    }
    for (ObjectList::iterator i = objects.begin(); i != objects.end(); ++i)
    {
        (*i)->destroy();
    }
}

void Environment::add(EnvironmentObject::Ptr obj)
{
    obj->setEnvironment(this);
    obj->init();
    objects.push_back(obj);
    // objects are reponsible for adding themselves
    // to the dynamics world and the osg root
}

void Environment::remove(EnvironmentObject::Ptr obj)
{
    for (ObjectList::iterator i = objects.begin(); i != objects.end(); ++i)
    {
        if (obj == *i)
        {
            (*i)->destroy();
            objects.erase(i);
            return;
        }
    }
}

void Environment::addConstraint(EnvironmentObject::Ptr cnt)
{
    cnt->setEnvironment(this);
    cnt->init();
    constraints.push_back(cnt);
}

void Environment::removeConstraint(EnvironmentObject::Ptr cnt)
{
    for (ConstraintList::iterator i = constraints.begin(); i != constraints.end(); ++i)
    {
        if (cnt == *i)
        {
            (*i)->destroy();
            constraints.erase(i);
            return;
        }
    }
}

double Environment::step(btScalar dt, int maxSubSteps, btScalar fixedTimeStep)
{
    ObjectList::iterator i;
    for (i = objects.begin(); i != objects.end(); ++i)
    {
        (*i)->prePhysics();
    }

    // std::cerr << "Start bullet step\n";
    int numFixedSteps = bullet->dynamicsWorld->stepSimulation(dt, maxSubSteps, fixedTimeStep);
    // std::cerr << "End bullet step\n";

    for (i = objects.begin(); i != objects.end(); ++i)
    {
        (*i)->preDraw();
    }
    bullet->softBodyWorldInfo.m_sparsesdf.GarbageCollect();

    

    return numFixedSteps*fixedTimeStep;
}
