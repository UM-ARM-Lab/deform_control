#ifndef _BASICOBJECTS_H_
#define _BASICOBJECTS_H_

#include <osg/MatrixTransform>
#include "simulation/environment.h"


// Not a real object; just wraps a set of child objects.
template<class ChildType>
class CompoundObject : public EnvironmentObject
{
    protected:
        typedef std::vector<typename ChildType::Ptr> ChildVector;
        ChildVector children;

    public:
        typedef boost::shared_ptr<CompoundObject<ChildType> > Ptr;

        ChildVector& getChildren()
        {
            return children;
        }

        CompoundObject()
        {}

        EnvironmentObject::Ptr copy(Fork &f) const
        {
            Ptr o(new CompoundObject<ChildType>());
            internalCopy(o, f);
            return o;
        }

        void internalCopy(CompoundObject::Ptr o, Fork &f) const
        {
            o->children.reserve(children.size());
            for (typename ChildVector::const_iterator ittr = children.begin(); ittr != children.end(); ++ittr)
            {
                if (*ittr)
                {
                    o->children.push_back(boost::static_pointer_cast<ChildType>((*ittr)->copy(f)));
                }
                else
                {
                    o->children.push_back(typename ChildType::Ptr());
                }
            }
        }

        virtual void init()
        {
            for (typename ChildVector::iterator ittr = children.begin(); ittr != children.end(); ++ittr)
            {
                if (*ittr)
                {
                    (*ittr)->setEnvironment(getEnvironment());
                    (*ittr)->init();
                }
            }
        }

        virtual void prePhysics()
        {
            for (typename ChildVector::iterator ittr = children.begin(); ittr != children.end(); ++ittr)
            {
                if (*ittr)
                {
                    (*ittr)->prePhysics();
                }
            }
        }

        virtual void preDraw()
        {
            for (typename ChildVector::iterator ittr = children.begin(); ittr != children.end(); ++ittr)
            {
                if (*ittr)
                {
                    (*ittr)->preDraw();
                }
            }
        }

        virtual void destroy()
        {
            for (typename ChildVector::iterator ittr = children.begin(); ittr != children.end(); ++ittr)
            {
                if (*ittr)
                {
                    (*ittr)->destroy();
                }
            }
        }
};


// An object that is entirely specified as a bullet btRigidBody
// (the OSG model will be created from the btRigidBody, and
// will be added to the scene graph and the dynamics world, respectively)
class BulletObject : public EnvironmentObject
{
    protected:
        struct MotionState : public btDefaultMotionState
        {
            typedef boost::shared_ptr<MotionState> Ptr;
            BulletObject &obj;

            MotionState(BulletObject &obj_, const btTransform &trans)
                : btDefaultMotionState(trans), obj(obj_)
            {}

            void setWorldTransform(const btTransform &t)
            {
                if (!obj.isKinematic)
                {
                    btDefaultMotionState::setWorldTransform(t);
                }
            }

            void setKinematicPos(const btTransform &pos)
            {
                if (!obj.isKinematic)
                {
                    std::cerr << "warning: calling setKinematicPos on the motionstate of a non-kinematic object" << std::endl;
                    return;
                }
                btDefaultMotionState::setWorldTransform(pos);
                // if we want to do collision detection in between timesteps,
                // we also have to directly set this
                obj.rigidBody->setCenterOfMassTransform(pos);
            }

            Ptr clone(BulletObject &newObj)
            {
                btTransform t;
                getWorldTransform(t);
                return Ptr(new MotionState(newObj, t));
            }
        };

    public:
        typedef boost::shared_ptr<BulletObject> Ptr;

        const bool isKinematic;

        // BULLET MEMBERS
        boost::shared_ptr<btRigidBody> rigidBody;
        // the motionState and collisionShape actually don't matter; the ones
        // embedded in the rigidBody are used for simulation. However,
        // placing them here will have them automatically deallocated
        // on destruction of the BulletObject
        MotionState::Ptr motionState;
        boost::shared_ptr<btCollisionShape> collisionShape;

        // OSG MEMBERS
        osg::ref_ptr<osg::Node> node;
        osg::ref_ptr<osg::MatrixTransform> transform;

        // CONSTRUCTORS
        // our own construction info class, which forces motionstate to be null
        struct CI : public btRigidBody::btRigidBodyConstructionInfo
        {
            CI(btScalar mass, btCollisionShape *collisionShape, const btVector3 &localInertia=btVector3(0,0,0))
                : btRigidBody::btRigidBodyConstructionInfo(mass, nullptr, collisionShape, localInertia)
            {}
        };

        // Constructor with a construciton info struct already filled
        BulletObject(CI ci, const btTransform &initTrans, bool isKinematic_ = false);
        // this constructor computes a ConstructionInfo for you
        BulletObject(btScalar mass, btCollisionShape *cs, const btTransform &initTrans, bool isKinematic_ = false);
        // copy constructor
        BulletObject(const BulletObject &o);

        // DESTRUCTOR
        virtual ~BulletObject() {}

        // Used to "fork" (not in the c-style process type) this object into multiple threads for Jacobian approximation
        EnvironmentObject::Ptr copy(Fork &f) const;
        void internalCopy(BulletObject::Ptr o, Fork &f) const;

        // called by Environment
        virtual void init();
        virtual void preDraw();
        virtual void destroy();

        // actions (for the user)
        class MoveAction : public Action
        {
            public:
                typedef boost::shared_ptr<MoveAction> Ptr;

                MoveAction(BulletObject *obj_)
                    : Action(), obj(obj_)
                {}

                MoveAction(BulletObject *obj_, const btTransform &start_, const btTransform &end_, float execTime)
                    : Action(execTime)
                    , obj(obj_)
                    , start(start_)
                    , end(end_)
                {}

                void setEndpoints(const btTransform &start_, const btTransform &end_)
                {
                    start = start_;
                    end = end_;
                }

                void step(float dt)
                {
                    if (done())
                    {
                        return;
                    }

                    stepTime(dt);
                    const float a = fracElapsed();
                    // linear interpolation of pos
                    const btVector3 newpos = (1-a)*start.getOrigin() + a*end.getOrigin();
                    const btQuaternion newrot = start.getRotation().slerp(end.getRotation(), a);
                    const btTransform newtrans(newrot, newpos);
                    if (obj->isKinematic)
                        obj->motionState->setKinematicPos(newtrans);
                    else
                        obj->motionState->setWorldTransform(newtrans);
                }

            private:
                BulletObject *obj;
                btTransform start, end;
        };

        MoveAction::Ptr createMoveAction();
        MoveAction::Ptr createMoveAction(const btTransform &start, const btTransform &end, float time);

        void setColor(float r, float g, float b, float a);

    protected:
        // by default uses osgBullet. Can be overridden to provide custom OSG mesh
        virtual osg::ref_ptr<osg::Node> createOSGNode();

    private:
        boost::shared_ptr<osg::Vec4f> m_color;
        void setColorAfterInit();
};


// An object that is purely a constraint for bullet, not used by OSG
class BulletConstraint : public EnvironmentObject
{
    public:
        typedef boost::shared_ptr<BulletConstraint> Ptr;

        boost::shared_ptr<btTypedConstraint> cnt;
        const bool disableCollisionsBetweenLinkedBodies;

        BulletConstraint(btTypedConstraint *cnt_, bool disableCollisionsBetweenLinkedBodies_ = false);
        BulletConstraint(boost::shared_ptr<btTypedConstraint> cnt_, bool disableCollisionsBetweenLinkedBodies_ = false);

        virtual void init();
        virtual void destroy();
        virtual EnvironmentObject::Ptr copy(Fork &f) const;

    private:
        BulletConstraint(const BulletConstraint &o);
};


// An infinite surface on the X-Y plane.
// the drawHalfExtents argument to the constructor is for rendering only
class PlaneStaticObject : public BulletObject
{
    public:
        typedef boost::shared_ptr<PlaneStaticObject> Ptr;

        PlaneStaticObject(const btVector3 &planeNormal_, btScalar planeConstant_, const btTransform &initTrans, btScalar drawHalfExtents_=50.);

        EnvironmentObject::Ptr copy(Fork &f) const;

    protected:
        // must override this since osgBullet doesn't recognize btStaticPlaneShape
        osg::ref_ptr<osg::Node> createOSGNode();

    private:
        const btVector3 planeNormal;
        const btScalar planeConstant;
        const btScalar drawHalfExtents;
};


// A cylinder along the Z-axis
class CylinderStaticObject : public BulletObject
{
    public:
        typedef boost::shared_ptr<CylinderStaticObject> Ptr;

        CylinderStaticObject(btScalar mass_, btScalar radius_, btScalar height_, const btTransform &initTrans);

        EnvironmentObject::Ptr copy(Fork &f) const;

    private:
        const btScalar mass, radius, height;
};


class SphereObject : public BulletObject
{
    public:
        typedef boost::shared_ptr<SphereObject> Ptr;

        SphereObject(btScalar mass_, btScalar radius_, const btTransform &initTrans, bool isKinematic=false);

        EnvironmentObject::Ptr copy(Fork &f) const;

    private:
        const btScalar mass, radius;
};


class BoxObject : public BulletObject
{
    public:

        typedef boost::shared_ptr<BoxObject> Ptr;

        BoxObject(btScalar mass_, const btVector3 &halfExtents_, const btTransform &initTrans, bool isKinematic=false);

        EnvironmentObject::Ptr copy(Fork &f) const;

        const btScalar mass;
        const btVector3 halfExtents;
};


// A wrapper for btCapsuleShapeX
class CapsuleObject : public BulletObject
{
    public:
        typedef boost::shared_ptr<CapsuleObject> Ptr;

        CapsuleObject(btScalar mass_, btScalar radius_, btScalar height_, const btTransform &initTrans);

        EnvironmentObject::Ptr copy(Fork &f) const;

    protected:
        osg::ref_ptr<osg::Node> createOSGNode();

    private:
        btScalar mass, radius, height;
};

#endif // _BASICOBJECTS_H_
