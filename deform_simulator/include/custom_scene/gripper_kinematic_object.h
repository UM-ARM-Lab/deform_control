#ifndef GRIPPER_KINEMATIC_OBJECT_H
#define GRIPPER_KINEMATIC_OBJECT_H

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"

class GripperKinematicObject : public CompoundObject<BoxObject>
{
    public:
        typedef boost::shared_ptr<GripperKinematicObject> Ptr;

        // Defined stretching detection nodes helper structure. --- Added by Mengyao

        struct GeoInfoToAnotherGripper
        {
            std::vector<size_t> from_nodes;
            std::vector<size_t> to_nodes;
            std::vector<double> node_contribution;
            std::string to_gripper_name;
        };


        GripperKinematicObject(
                const std::string& name_input,
                const float apperture_input,
                const btVector4 color = btVector4(0.6f, 0.6f, 0.6f, 0.9f));

        void translate(btVector3 transvec);
        void applyTransform(btTransform tm);
        void setWorldTransform(btTransform tm);
        btTransform getWorldTransform();
        void getWorldTransform(btTransform& in);

        // used only for the rope
        void rigidGrab(btRigidBody* prb, size_t objectnodeind, Environment::Ptr env_ptr);

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

        // Set the stretching vector information --- Added by Mengyao
        const std::string getGripperName();

        void setClothGeoInfoToAnotherGripper(
                Ptr to_gripper,
                const btSoftBody* cloth,
                const int num_x,
                const int num_y);

        friend std::ostream& operator<< (std::ostream& stream, const GripperKinematicObject& gripper);

    private:
        void internalCopy(GripperKinematicObject::Ptr o, Fork &f) const;

        void attach(btSoftBody* psb, double radius);
        void detach();


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

        boost::shared_ptr<btGeneric6DofSpringConstraint> rope_cnt;
        boost::shared_ptr<btGeneric6DofConstraint> top_jaw_cnt;
        boost::shared_ptr<btGeneric6DofConstraint> bottom_jaw_cnt;

    public:
        // Defined stretching detection nodes helper structure. --- Added by Mengyao
        // if more than two grippers in the future, should change it to vector
        GeoInfoToAnotherGripper to_another_gripper_info;
        std::vector<std::pair<std::string, btVector3>> stretching_to_center_offset;

};

#endif
