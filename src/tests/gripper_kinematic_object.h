#pragma once

#include "colab_cloth_defines.h"

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"

enum GripperState { GripperState_DONE, GripperState_CLOSING, GripperState_OPENING };

class GripperKinematicObject : public CompoundObject<BoxObject>{
public:

    float apperture;
    btTransform cur_tm;
    bool bOpen;
    bool bAttached;
    btVector3 halfextents;
    std::vector<int> vattached_node_inds;
    double closed_gap;
    GripperState state;
    boost::shared_ptr<btGeneric6DofConstraint> cnt;

    typedef boost::shared_ptr<GripperKinematicObject> Ptr;

    GripperKinematicObject(btVector4 color = btVector4(0,0,1,0.3));
    void translate(btVector3 transvec);
    void applyTransform(btTransform tm);
    void setWorldTransform(btTransform tm);
    btTransform getWorldTransform(){return cur_tm;}
    void getWorldTransform(btTransform& in){in = cur_tm;}
    void toggleOpen();
    void toggleAttach(btSoftBody * psb, double radius = 0);
    void rigidGrab(btRigidBody* prb, int objectnodeind, Environment::Ptr env_ptr);
    void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts);
    void appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence=1);
    void releaseAllAnchors(btSoftBody * psb) {psb->m_anchors.clear();}
    void step_openclose(btSoftBody * psb);

    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new GripperKinematicObject());
        internalCopy(o, f);
        return o;
    }
    void internalCopy(GripperKinematicObject::Ptr o, Fork &f) const {
        o->apperture = apperture;
        o->cur_tm = cur_tm;
        o->bOpen = bOpen;
        o->state = state;
        o->closed_gap = closed_gap;
        o->vattached_node_inds = vattached_node_inds;
        o->halfextents = halfextents;
        o->bAttached = bAttached;

        o->children.clear();
        o->children.reserve(children.size());
        ChildVector::const_iterator i;
        for (i = children.begin(); i != children.end(); ++i) {
            if (*i)
                o->children.push_back(boost::static_pointer_cast<BoxObject> ((*i)->copy(f)));
            else
                o->children.push_back(BoxObject::Ptr());
        }
    }
};
