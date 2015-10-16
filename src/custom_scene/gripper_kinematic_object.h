#ifndef GRIPPER_KINEMATIC_OBJECT_H
#define GRIPPER_KINEMATIC_OBJECT_H

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"

enum GripperState { GripperState_DONE, GripperState_CLOSING, GripperState_OPENING };

class GripperKinematicObject : public CompoundObject<BoxObject>
{
    public:
        typedef boost::shared_ptr<GripperKinematicObject> Ptr;

        GripperKinematicObject(  btVector4 color = btVector4( 0,0,1,0.3 ), int apperture_input = 0.5  );

        void translate( btVector3 transvec );
        void applyTransform( btTransform tm );
        void setWorldTransform( btTransform tm );
        btTransform getWorldTransform()
        {
            return cur_tm;
        }
        void getWorldTransform( btTransform& in ){
            in = cur_tm;
        }
        void toggle();

        // if radius = 0, we are not using radius, we are using an alternate method
        void toggleattach( btSoftBody * psb, double radius = 0 );
        void rigidGrab( btRigidBody* prb, int objectnodeind, Environment::Ptr env_ptr );
        void getContactPointsWith( btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts );
        void appendAnchor( btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence=1 );
        void releaseAllAnchors( btSoftBody * psb )
        {
            psb->m_anchors.clear();
        }

        EnvironmentObject::Ptr copy( Fork &f ) const;
        void internalCopy( GripperKinematicObject::Ptr o, Fork &f ) const;
        void step_openclose( btSoftBody * psb );

        /// public for CustomKeyHandler
        float apperture;
        bool bOpen;
        GripperState state;

    private:
        bool bAttached;
        btTransform cur_tm;
        btVector3 halfextents;
        std::vector<int> vattached_node_inds;
        double closed_gap;
        boost::shared_ptr<btGeneric6DofConstraint> cnt;
};

#endif
