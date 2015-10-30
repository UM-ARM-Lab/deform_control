#ifndef GRIPPER_KINEMATIC_OBJECT_H
#define GRIPPER_KINEMATIC_OBJECT_H

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"

enum GripperState { GripperState_DONE, GripperState_CLOSING, GripperState_OPENING };

class GripperKinematicObject : public CompoundObject<BoxObject>
{
    public:
        typedef boost::shared_ptr<GripperKinematicObject> Ptr;

        GripperKinematicObject( float apperture_input, btVector4 color = btVector4( 0.6, 0.6, 0.6, 0.9 ) );

        void translate( btVector3 transvec );
        void applyTransform( btTransform tm );
        void setWorldTransform( btTransform tm );
        btTransform getWorldTransform();
        void getWorldTransform( btTransform& in );

        // used only for the rope
        void rigidGrab( btRigidBody* prb, int objectnodeind, Environment::Ptr env_ptr );

        // Toggles open/close
        void toggleOpen();

        // Toggles attached/not attached (btSoftBody term)
        // if radius = 0, we are not using radius, we are using an alternate method
        void toggleAttach( btSoftBody * psb, double radius = 0 );

        void getContactPointsWith( btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts );
        void appendAnchor( btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence = 1 );
        void releaseAllAnchors( btSoftBody * psb );

        std::vector<size_t> getAttachedNodeIndices();

        // Used by the manual grippers for cloth
        void step_openclose( btSoftBody * psb );

        EnvironmentObject::Ptr copy( Fork &f ) const;
        void internalCopy( GripperKinematicObject::Ptr o, Fork &f ) const;

    private:
        btVector3 halfextents;
        btTransform cur_tm;

        GripperState state; // used only for the manual grippers (I think)
        bool bOpen;         // used only for cloth (I think)
        float apperture;
        double closed_gap;  // used only for cloth (I think)

        bool bAttached;
        std::vector<size_t> vattached_node_inds;

        boost::shared_ptr<btGeneric6DofConstraint> cnt;
};

#endif
