#ifndef GRIPPER_KINEMATIC_OBJECT_H
#define GRIPPER_KINEMATIC_OBJECT_H

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"

class GripperKinematicObject : public CompoundObject<BoxObject>
{
    public:
        typedef boost::shared_ptr<GripperKinematicObject> Ptr;

        // Defined stretching detection nodes helper structure.
        // Used by StretchingAvoidanceController
        struct GeoInfoToAnotherGripper
        {
            std::vector<size_t> from_nodes;
            std::vector<size_t> to_nodes;
            std::vector<double> node_contribution;
            std::string to_gripper_name;
        };

        GripperKinematicObject(
                const Environment::Ptr env_input,
                const std::string& name_input,
                const float apperture_input,
                const btVector4 color = btVector4(0.6f, 0.6f, 0.6f, 0.9f));

        virtual void destroy() override;

        void translate(btVector3 transvec);
        void applyTransform(btTransform tm);
        void setWorldTransform(btTransform tm);
        btTransform getWorldTransform();
        void getWorldTransform(btTransform& in);

        // used only for the rope
        void rigidGrab(btRigidBody* prb, size_t objectnodeind);

        // Toggles open/close
        void toggleOpen();

        // Toggles attached/not attached (btSoftBody term)
        // if radius = 0, we are not using radius, we are using an alternate method
        void toggleAttach(btSoftBody * psb, double radius = 0);

        void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts);
        void appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence = 1);
        btVector3 calculateSoftBodyForce() const;
        void releaseAllAnchors(btSoftBody * psb);

        const std::vector<size_t>& getAttachedNodeIndices() const;

        const btVector3& getHalfExtents() const;
        float getGripperRadius() const;

        // Used by the manual grippers for cloth
        void step_openclose(btSoftBody * psb);

        EnvironmentObject::Ptr copy(Fork &f) const;

        const std::string getGripperName();

        // TODO: Can we remove passing this pointer, and just use m_psb?
        void setClothGeoInfoToAnotherGripper(
                Ptr to_gripper,
                const boost::shared_ptr<btSoftBody> cloth,
                const int num_x);

        const GeoInfoToAnotherGripper& getClothGeoInfoToAnotherGripper() const;

        friend std::ostream& operator<< (std::ostream& stream, const GripperKinematicObject& gripper);

    private:
        void internalCopy(GripperKinematicObject::Ptr o, Fork &f) const;

        void attach(btSoftBody* psb, double radius);
        void detach();

        Environment::Ptr env;

        std::string name;
        btVector3 halfextents;

        btTransform cur_tm;

        enum GripperState { GripperState_DONE, GripperState_CLOSING, GripperState_OPENING };

        GripperState state;
        bool bOpen;         // used only for cloth (I think)
        float apperture;
        float closed_gap;  // used only for cloth (I think)

        bool b_attached;
        std::vector<size_t> vattached_node_inds;
        btSoftBody* m_psb = NULL;   // soft body attached to this gripper

        boost::shared_ptr<btGeneric6DofConstraint> rope_cnt;
        // Defined stretching detection nodes helper structure.
        // if more than two grippers in the future, should change it to vector
        bool to_another_gripper_info_valid;
        GeoInfoToAnotherGripper to_another_gripper_info;
};

#endif
