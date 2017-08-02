#ifndef GRIPPER_KINEMATIC_OBJECT_H
#define GRIPPER_KINEMATIC_OBJECT_H

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"

class GripperKinematicObject : public CompoundObject<BoxObject>
{
    public:
        typedef boost::shared_ptr<GripperKinematicObject> Ptr;

        // Defined stretching detection nodes helper structure. --- Added by Mengyao
        struct AttatchingToOneGripper
        {
            // First element is an indices attached to the gripper,
            // second one define the stretching detection vector from the first node.
            typedef std::vector<std::pair<ssize_t, ssize_t>> AttatchingInfo;

            AttatchingInfo all_closest_vector;
            ssize_t to_gripper_ind;
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
        void releaseAllAnchors(btSoftBody * psb);

        const std::vector<size_t>& getAttachedNodeIndices() const;

        const btVector3& getHalfExtents() const;
        float getGripperRadius() const;

        // Get force data and torque data
        std::vector<btVector3> getRopeGripperForce() const;
        std::vector<btVector3> getGripperTotalTorque() const;

        // Used by the manual grippers for cloth
        void step_openclose(btSoftBody * psb);

        EnvironmentObject::Ptr copy(Fork &f) const;

        friend std::ostream& operator<< (std::ostream& stream, const GripperKinematicObject& gripper);

    private:
        void internalCopy(GripperKinematicObject::Ptr o, Fork &f) const;

        std::string name;
        btVector3 halfextents;

        // Revised by Mengyao
        // btTransform cur_tm;
        btTransform cur_top_tm;
        btTransform cur_bottom_tm;

        enum GripperState { GripperState_DONE, GripperState_CLOSING, GripperState_OPENING };

        GripperState state;
        bool bOpen;         // used only for cloth (I think)
        float apperture;
        float closed_gap;  // used only for cloth (I think)

        bool bAttached;
        std::vector<size_t> vattached_node_inds;

        // Edited by Mengyao
      //  boost::shared_ptr<btGeneric6DofConstraint> rope_cnt;
        boost::shared_ptr<btGeneric6DofSpringConstraint> rope_cnt;
        boost::shared_ptr<btGeneric6DofConstraint> top_jaw_cnt;
        boost::shared_ptr<btGeneric6DofConstraint> bottom_jaw_cnt;

        // Defined stretching detection nodes helper structure. --- Added by Mengyao
        // if more than two grippers in the future, should change it to vector
        AttatchingToOneGripper to_another_gripper_info;

        // --- Added by Mengyao
    public:
        btVector3 boxhalfextents;
        CompoundObject::ChildVector boxes_children;
        btTransform box_cur_top_tm;
        btTransform box_cur_bottom_tm;

};

#endif
