#include "simulation/environment.h"
#include "utils/config.h"

#define COPY_ARRAY(a, b) { (a).resize((b).size()); for (int z = 0; z < (b).size(); ++z) (a)[z] = (b)[z]; }

////////////////////////////////////////////////////////////////////////////////
// Struct used to create and interact with an OpenSceneGraph.                 //
////////////////////////////////////////////////////////////////////////////////

OSGInstance::OSGInstance()
{
    root = new osg::Group;
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
    solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->getDispatchInfo().m_enableSPU = true;

    softBodyWorldInfo.m_broadphase = broadphase;
    softBodyWorldInfo.m_dispatcher = dispatcher;
    softBodyWorldInfo.m_sparsesdf.Initialize();
}







void deepCopyDbvt(btDbvt& copy, const btDbvt& orig)
{
    btDbvt::IClone iclone;
    orig.clone(copy, &iclone);
}

//btDbvtProxy*			m_stageRoots[STAGECOUNT+1];	// Stages list
btDbvtProxy* deepCopyDbvtProxyPointer(const btDbvtProxy* const orig)
{
    if (orig == NULL)
    {
        return NULL;
    }
    else
    {

    }

}

//btOverlappingPairCache*	m_paircache;				// Pair cache
void deepCopyOverlappingPairCache(btHashedOverlappingPairCache* copy, const btHashedOverlappingPairCache* const orig)
{
    COPY_ARRAY(copy->getOverlappingPairArray(), orig->getOverlappingPairArray());
    copy->m_overlapFilterCallback = orig->m_overlapFilterCallback; // TODO: this is probably wrong, but doesnt' matter for WAFR trials
    copy->m_blockedForChanges = orig->m_blockedForChanges;

    COPY_ARRAY(copy->m_hashTable, orig->m_hashTable);
    COPY_ARRAY(copy->m_next, orig->m_next);
    copy->m_ghostPairCallback = orig->m_ghostPairCallback; // TODO: This is probably wrong, but doesn't matter for WAFR trials
}

btDbvtBroadphase* deepCopyBroadphase(const btDbvtBroadphase* const orig)
{
    btDbvtBroadphase* copy = new btDbvtBroadphase();

    deepCopyDbvt(copy->m_sets[0], orig->m_sets[0]);
    deepCopyDbvt(copy->m_sets[1], orig->m_sets[1]);
    for (int i = 0; i < btDbvtBroadphase::STAGECOUNT + 1; i++)
    {
        copy->m_stageRoots[i] = deepCopyDbvtProxyPointer(orig->m_stageRoots[i]);
    }
    deepCopyOverlappingPairCache((btHashedOverlappingPairCache*)copy->m_paircache, (btHashedOverlappingPairCache*)orig->m_paircache);
    copy->m_prediction          = orig->m_prediction;
    copy->m_stageCurrent        = orig->m_stageCurrent;
    copy->m_fupdates            = orig->m_fupdates;
    copy->m_dupdates            = orig->m_dupdates;
    copy->m_cupdates            = orig->m_cupdates;
    copy->m_newpairs            = orig->m_newpairs;
    copy->m_fixedleft           = orig->m_fixedleft;
    copy->m_updates_call        = orig->m_updates_call;
    copy->m_updates_done        = orig->m_updates_done;
    copy->m_updates_ratio       = orig->m_updates_ratio;
    copy->m_pid                 = orig->m_pid;
    copy->m_cid                 = orig->m_cid;
    copy->m_gid                 = orig->m_gid;
    copy->m_releasepaircache    = orig->m_releasepaircache;
    copy->m_needcleanup         = orig->m_needcleanup;

    return copy;
}





btSoftBodyRigidBodyCollisionConfiguration* deepCopyCollisionConfiguration(const btSoftBodyRigidBodyCollisionConfiguration* const orig)
{
    return new btSoftBodyRigidBodyCollisionConfiguration();
}

btCollisionDispatcher* deepCopyDispatcher(const btCollisionDispatcher* const orig, btSoftBodyRigidBodyCollisionConfiguration* collisionConfiguration)
{
    return new btCollisionDispatcher(collisionConfiguration);
}

btSequentialImpulseConstraintSolver* deepCopySolver(const btSequentialImpulseConstraintSolver* const orig)
{
    return new btSequentialImpulseConstraintSolver();
}


BulletInstance::BulletInstance(const Ptr& other)
    : broadphase(nullptr)
    , collisionConfiguration(nullptr)
    , dispatcher(nullptr)
    , solver(nullptr)
    , dynamicsWorld(nullptr)
    , softBodyWorldInfo(other->softBodyWorldInfo)
{
    softBodyWorldInfo.m_broadphase = nullptr;
    softBodyWorldInfo.m_dispatcher = nullptr;


    broadphase = deepCopyBroadphase((btDbvtBroadphase *)other->broadphase);
    collisionConfiguration = deepCopyCollisionConfiguration(other->collisionConfiguration);
    dispatcher = deepCopyDispatcher(other->dispatcher, collisionConfiguration);
    solver = deepCopySolver(other->solver);
    dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->getDispatchInfo().m_enableSPU = true;

    softBodyWorldInfo.m_broadphase = broadphase;
    softBodyWorldInfo.m_dispatcher = dispatcher;
    softBodyWorldInfo.m_sparsesdf.Initialize();
    setGravity(other->softBodyWorldInfo.m_gravity);
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

void BulletInstance::contactTest(btCollisionObject *obj,
                                 CollisionObjectSet &out,
                                 const CollisionObjectSet *ignore)
{
    struct ContactCallback : public btCollisionWorld::ContactResultCallback
    {
        const CollisionObjectSet *ignore;
        CollisionObjectSet &out;
        ContactCallback(const CollisionObjectSet *ignore_, CollisionObjectSet &out_)
            : ignore(ignore_), out(out_)
        {}

        btScalar addSingleResult(btManifoldPoint &,
                                 const btCollisionObject *colObj0, int, int,
                                 const btCollisionObject *colObj1, int, int)
        {
            (void)colObj0;
            if (ignore && ignore->find(colObj1) == ignore->end())
                out.insert(colObj1);
            return 0;
        }

    } cb(ignore, out);

    dynamicsWorld->contactTest(obj, cb);
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

    int numFixedSteps = bullet->dynamicsWorld->stepSimulation(dt, maxSubSteps, fixedTimeStep);

    for (i = objects.begin(); i != objects.end(); ++i)
    {
        (*i)->preDraw();
    }
    bullet->softBodyWorldInfo.m_sparsesdf.GarbageCollect();

    return numFixedSteps*fixedTimeStep;
}
