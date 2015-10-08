#ifndef GRIPPER_KINEMATIC_OBJECT_H
#define GRIPPER_KINEMATIC_OBJECT_H

#include "colab_cloth.h"

enum GripperState { GripperState_DONE, GripperState_CLOSING, GripperState_OPENING };

class GripperKinematicObject : public CompoundObject<BoxObject>
{
    public:
        typedef boost::shared_ptr<GripperKinematicObject> Ptr;

        float apperture;
        btTransform cur_tm;
        bool bOpen;
        bool bAttached;
        btVector3 halfextents;
        std::vector<int> vattached_node_inds;
        double closed_gap;
        GripperState state;
        boost::shared_ptr<btGeneric6DofConstraint> cnt;


        GripperKinematicObject(btVector4 color = btVector4(0,0,1,0.3));
        void translate(btVector3 transvec);
        void applyTransform(btTransform tm);
        void setWorldTransform(btTransform tm);
        btTransform getWorldTransform(){return cur_tm;}
        void getWorldTransform(btTransform& in){in = cur_tm;}
        void toggle();
        void toggleattach(btSoftBody * psb, double radius = 0);
        void rigidGrab(btRigidBody* prb, int objectnodeind, Environment::Ptr env_ptr);
        void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts);
        void appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence=1);
        void releaseAllAnchors(btSoftBody * psb) {psb->m_anchors.clear();}

        EnvironmentObject::Ptr copy(Fork &f) const;
        void internalCopy(GripperKinematicObject::Ptr o, Fork &f) const;
        void step_openclose(btSoftBody * psb);
};

#endif
