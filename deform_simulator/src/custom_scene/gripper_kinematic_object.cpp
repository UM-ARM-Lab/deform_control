#include "custom_scene/gripper_kinematic_object.h"

#include <bullet_helpers/bullet_internal_conversions.hpp>
#include <bullet_helpers/bullet_pretty_print.hpp>

#include "utils/config.h"

using namespace BulletHelpers;

GripperKinematicObject::GripperKinematicObject(
        const std::string& name_input,
        const float apperture_input,
        const btVector4 color)
    : name(name_input)
    #warning "Gripper size magic number - move to params file"
//    , halfextents(btVector3(0.015f, 0.015f, 0.005f)*METERS)
    , halfextents(btVector3(0.015f, 0.015f, 0.005f)*METERS)
    , state(GripperState_DONE)
    , bOpen (true)
    // TODO:: move this to RosParam
    , apperture(apperture_input)
//    , apperture(0.0285f * METERS)
    , closed_gap(0.006f*METERS)
    , bAttached(false)
    , boxhalfextents(btVector3(0.0f, 0.0f, 0.00f)*METERS)
//    , boxhalfextents(btVector3(0.015f, 0.015f, 0.005f)*METERS)
{

    // The Mass of BoxObject was 0, --- Edited by Mengyao
    BoxObject::Ptr top_jaw(new BoxObject(0, halfextents,
                btTransform(btQuaternion(0, 0, 0, 1),
                            btVector3(0, 0, apperture/2 + boxhalfextents[2] * 2)), true));
    top_jaw->setColor(color[0],color[1],color[2],color[3]);
    top_jaw->collisionShape->setMargin(0.004f*METERS);

    BoxObject::Ptr bottom_jaw(new BoxObject(0, halfextents,
                btTransform(btQuaternion(0, 0, 0, 1),
                            btVector3(0, 0, -apperture/2 - boxhalfextents[2] * 2)), true));
    bottom_jaw->setColor(color[0],color[1],color[2],color[3]);
    bottom_jaw->collisionShape->setMargin(0.004f*METERS);

    // Find the center of the gripper composite object
    top_jaw->motionState->getWorldTransform(cur_tm);
    cur_tm.setOrigin(cur_tm.getOrigin() - btVector3(0,0,-apperture/2 - boxhalfextents[2] * 2));

    // TODO:: Added by Mengyao, probably should delete this one
//    bottom_jaw->motionState->getWorldTransform(cur_bottom_tm);
//    cur_bottom_tm.setOrigin(cur_bottom_tm.getOrigin() - btVector3(0,0, apperture/2 + boxhalfextents[2] * 2));

    children.push_back(top_jaw);
    children.push_back(bottom_jaw);
}

void GripperKinematicObject::translate(btVector3 transvec)
{
    btTransform tm = getWorldTransform();
    tm.setOrigin(tm.getOrigin() + transvec);
    setWorldTransform(tm);
}

void GripperKinematicObject::applyTransform(btTransform tm)
{
    setWorldTransform(getWorldTransform()*tm);
}

void GripperKinematicObject::setWorldTransform(btTransform tm)
{
    btTransform top_tm = tm;
    btTransform bottom_tm = tm;

    btTransform top_offset;

    children[0]->motionState->getWorldTransform(top_offset);
    top_offset = cur_tm.inverse()*top_offset;

    top_tm.setOrigin(top_tm.getOrigin() + top_tm.getBasis().getColumn(2)*(top_offset.getOrigin()[2]));
    bottom_tm.setOrigin(bottom_tm.getOrigin() - bottom_tm.getBasis().getColumn(2)*(top_offset.getOrigin()[2]));

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);

    cur_tm = tm;
}

btTransform GripperKinematicObject::getWorldTransform()
{
    return cur_tm;
}

void GripperKinematicObject::getWorldTransform(btTransform& in)
{
    in = cur_tm;
}


void GripperKinematicObject::rigidGrab(btRigidBody* prb, size_t objectnodeind, Environment::Ptr env_ptr)
{
    // Add dynamics squeezing boxes, --- Added by Mengyao
    // The Mass of BoxObject was 0, --- Edited by Mengyao

    // End Mengyao Added

    // Edited by Mengyao

    // Add Ros Parameters for #switch# or move the code into a new class
    // Add Constraint   ---- Revised by Mengyao
    const bool use_squeezing_boxes = false;
    if (use_squeezing_boxes)
    {
        // ************* Initialize squeezing boxes *********************
        const btVector4 color(0.95f, 0.6f, 0.6f, 1.0f);

        BoxObject::Ptr top_squeezing_jaw(new BoxObject(1, boxhalfextents,
                    btTransform(prb->getOrientation(),
                                prb->getCenterOfMassPosition() + btVector3(0, 0, apperture/2)), false));
        top_squeezing_jaw->setColor(color[0],color[1],color[2],color[3]);
        top_squeezing_jaw->collisionShape->setMargin(0.004f*METERS);

        BoxObject::Ptr bottom_squeezing_jaw(new BoxObject(1, boxhalfextents,
                    btTransform(prb->getOrientation(),
                                prb->getCenterOfMassPosition() + btVector3(0, 0, -apperture/2)), false));
        bottom_squeezing_jaw->setColor(color[0],color[1],color[2],color[3]);
        bottom_squeezing_jaw->collisionShape->setMargin(0.004f*METERS);

        top_squeezing_jaw->motionState->getWorldTransform(box_cur_top_tm);
        box_cur_top_tm.setOrigin(box_cur_top_tm.getOrigin() - btVector3(0, 0, apperture/2));

        bottom_squeezing_jaw->motionState->getWorldTransform(box_cur_bottom_tm);
        box_cur_bottom_tm.setOrigin(box_cur_bottom_tm.getOrigin() - btVector3(0, 0, -apperture/2));

        // Set best friction
        top_squeezing_jaw->rigidBody->setFriction(1000.0);
        bottom_squeezing_jaw->rigidBody->setFriction(1000.0);
        // Set restitution
        top_squeezing_jaw->rigidBody->setRestitution(0.2);
        bottom_squeezing_jaw->rigidBody->setRestitution(0.2);

    //    top_squeezing_jaw->rigidBody->setAnisotropicFriction(btVector3(0.9, 0.9, 0.9));
    //    bottom_squeezing_jaw->rigidBody->setAnisotropicFriction(btVector3(0.9, 0.9, 0.9));
    //    top_squeezing_jaw->rigidBody->applyForce((cur_tm.getOrigin() - box_cur_top_tm.getOrigin())*10000, box_cur_top_tm.getOrigin());
    //    bottom_squeezing_jaw->rigidBody->applyForce((cur_tm.getOrigin() - box_cur_bottom_tm.getOrigin())*10000, box_cur_bottom_tm.getOrigin());

        boxes_children.push_back(top_squeezing_jaw);
        boxes_children.push_back(bottom_squeezing_jaw);

        env_ptr->add(top_squeezing_jaw);
        env_ptr->add(bottom_squeezing_jaw);
        env_ptr->bullet->dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_USE_2_FRICTION_DIRECTIONS;


        // ******************* Get squeezing boxes and grippers transform *******
        btTransform top_tm;
        children[0]->motionState->getWorldTransform(top_tm);

        btTransform box_top_tm;
        boxes_children[0]->motionState->getWorldTransform(box_top_tm);

        btTransform bottom_tm;
        children[1]->motionState->getWorldTransform(bottom_tm);

        btTransform box_bottom_tm;
        boxes_children[1]->motionState->getWorldTransform(box_bottom_tm);

        // ******* Set Constraints: boxes to kinematic grippers ****************

        // Set constraint for top one
        top_jaw_cnt.reset(new btGeneric6DofConstraint(*(children[0]->rigidBody.get()),
                          *(boxes_children[0]->rigidBody.get()),
                          top_tm.inverse()*cur_tm,
                          box_top_tm.inverse()*box_cur_top_tm,
                   //       btTransform(btQuaternion(0,0,0,1),btVector3(0,0,-apperture/2)),
                   //       btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)),
                          true));

        top_jaw_cnt->setLinearLowerLimit(btVector3(0,0,0));
        top_jaw_cnt->setLinearUpperLimit(btVector3(0,0,0));
        top_jaw_cnt->setAngularLowerLimit(btVector3(0,0,0));
        top_jaw_cnt->setAngularUpperLimit(btVector3(0,0,0));
        env_ptr->bullet->dynamicsWorld->addConstraint(top_jaw_cnt.get());

        // **************** set constraint for bottom one **********************
        bottom_jaw_cnt.reset(new btGeneric6DofConstraint(*(children[0]->rigidBody.get()),
                          *(boxes_children[1]->rigidBody.get()),
                          top_tm.inverse()*cur_tm,
                          box_bottom_tm.inverse()*box_cur_bottom_tm,
                   //       btTransform(btQuaternion(0,0,0,1),btVector3(0,0,-apperture/2)),
                   //       btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)),
                          true));

        bottom_jaw_cnt->setLinearLowerLimit(btVector3(0,0,0));
        bottom_jaw_cnt->setLinearUpperLimit(btVector3(0,0,0));
        bottom_jaw_cnt->setAngularLowerLimit(btVector3(0,0,0));
        bottom_jaw_cnt->setAngularUpperLimit(btVector3(0,0,0));
        env_ptr->bullet->dynamicsWorld->addConstraint(bottom_jaw_cnt.get());


        // ***************** Set 6DOF spring constraints: boxes to rope ***********

        rope_cnt.reset(new btGeneric6DofSpringConstraint(*(boxes_children[0]->rigidBody.get()),
                       *prb,
                       box_top_tm.inverse()*box_cur_top_tm,
                       btTransform(btQuaternion(0,0,0,1),btVector3(0,0,-apperture/2+halfextents[2])),
                       true));

        rope_cnt->setLinearLowerLimit(btVector3(0,0,0));
        rope_cnt->setLinearUpperLimit(btVector3(0,0,0));
        rope_cnt->setAngularLowerLimit(btVector3(0,0,0));
        rope_cnt->setAngularUpperLimit(btVector3(0,0,0));

        rope_cnt->enableSpring(0, true);
        rope_cnt->enableSpring(1, true);
        rope_cnt->enableSpring(2, true);
        rope_cnt->enableSpring(3, true);
        rope_cnt->enableSpring(4, true);
        rope_cnt->enableSpring(5, true);

        rope_cnt->setStiffness(0, 1000.0);
        rope_cnt->setStiffness(1, 1000.0);
        rope_cnt->setStiffness(2, 1000.0);
        rope_cnt->setStiffness(3, 1000.0);
        rope_cnt->setStiffness(4, 1000.0);
        rope_cnt->setStiffness(5, 1.0);

        env_ptr->bullet->dynamicsWorld->addConstraint(rope_cnt.get());

    }
    else
    {
        btTransform top_tm;
        children[0]->motionState->getWorldTransform(top_tm);

        rope_cnt.reset(new btGeneric6DofSpringConstraint(*(children[0]->rigidBody.get()),
     //   rope_cnt.reset(new btGeneric6DofConstraint(*(children[0]->rigidBody.get()),
                       *prb,
                       top_tm.inverse()*cur_tm,
                       btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)),
                       true));

        rope_cnt->setLinearLowerLimit(btVector3(0,0,0));
        rope_cnt->setLinearUpperLimit(btVector3(0,0,0));
        rope_cnt->setAngularLowerLimit(btVector3(0,0,0));
        rope_cnt->setAngularUpperLimit(btVector3(0,0,0));
        env_ptr->bullet->dynamicsWorld->addConstraint(rope_cnt.get());

        // Try to enable spring
    //    rope_cnt->enableSpring(0, true);
    //    rope_cnt->setStiffness(0, 1000.0);
    //    rope_cnt->enableSpring(1, true);
    //    rope_cnt->setStiffness(1, 1000.0);
    }


    vattached_node_inds.clear();
    vattached_node_inds.push_back(objectnodeind);
}

void GripperKinematicObject::toggleOpen()
{
    btTransform top_tm;    btTransform bottom_tm;
    children[0]->motionState->getWorldTransform(top_tm);
    children[1]->motionState->getWorldTransform(bottom_tm);

    if(bOpen)
    {
        //std::cout << "Closing gripper\n";
        //btTransform top_offset = cur_tm.inverse()*top_tm;
        //float close_length = (1+closed_gap)*top_offset.getOrigin()[2] - children[0]->halfExtents[2];
        //float close_length = (apperture/2 - children[0]->halfExtents[2] + closed_gap/2);
        top_tm.setOrigin(cur_tm.getOrigin() + cur_tm.getBasis().getColumn(2)*(children[0]->halfExtents[2] + closed_gap/2));
        bottom_tm.setOrigin(cur_tm.getOrigin() - cur_tm.getBasis().getColumn(2)*(children[1]->halfExtents[2] + closed_gap/2));
    }
    else
    {
        //std::cout << "Opening gripper\n";
        top_tm.setOrigin(cur_tm.getOrigin() - cur_tm.getBasis().getColumn(2)*(apperture/2));
        bottom_tm.setOrigin(cur_tm.getOrigin() + cur_tm.getBasis().getColumn(2)*(apperture/2));
    }

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);

    bOpen = !bOpen;
}

void GripperKinematicObject::toggleAttach(btSoftBody * psb, double radius)
{
    //std::cout << name << " toggleAttach ";
    if(bAttached)
    {
        btAlignedObjectArray<btSoftBody::Anchor> newanchors;
        for(int i = 0; i < psb->m_anchors.size(); i++)
        {
            if(psb->m_anchors[i].m_body != children[0]->rigidBody.get() && psb->m_anchors[i].m_body != children[1]->rigidBody.get())
                newanchors.push_back(psb->m_anchors[i]);
        }
        releaseAllAnchors(psb);
        for(int i = 0; i < newanchors.size(); i++)
        {
            psb->m_anchors.push_back(newanchors[i]);
        }
        vattached_node_inds.clear();
    }
    else
    {
        if (radius >= 0)
        {
            if(radius == 0)
            {
                radius = 2 * halfextents[0];
            }
//            std::cout << "using radius contact: radius: " << radius << std::endl;

            btTransform top_tm;
            children[0]->motionState->getWorldTransform(top_tm);
            btTransform bottom_tm;
            children[1]->motionState->getWorldTransform(bottom_tm);
            size_t closest_body;
            for(int j = 0; j < psb->m_nodes.size(); j++)
            {
                if((psb->m_nodes[j].m_x - cur_tm.getOrigin()).length() < radius)
                {
                    if((psb->m_nodes[j].m_x - top_tm.getOrigin()).length() < (psb->m_nodes[j].m_x - bottom_tm.getOrigin()).length())
                        closest_body = 0;
                    else
                        closest_body = 1;

                    vattached_node_inds.push_back((size_t)j);
                    appendAnchor(psb, &psb->m_nodes[j], children[closest_body]->rigidBody.get());
//                    std::cout << "\tappending anchor, closest ind: " << j << std::endl;

                }
            }
        }
        else
        {
            std::vector<btVector3> nodeposvec;
            nodeArrayToNodePosVector(psb->m_nodes, nodeposvec);

            for(size_t k = 0; k < 2; k++)
            {
                BoxObject::Ptr part = children[k];

                btRigidBody* rigidBody = part->rigidBody.get();
                btSoftBody::tRContactArray rcontacts;
                getContactPointsWith(psb, rigidBody, rcontacts);
                //std::cout << "got " << rcontacts.size() << " contacts\n";

                //if no contacts, return without toggling bAttached
                if(rcontacts.size() == 0)
                    continue;

                for (int i = 0; i < rcontacts.size(); ++i) {
                    //const btSoftBody::RContact &c = rcontacts[i];
                    btSoftBody::Node *node = rcontacts[i].m_node;
                    //btRigidBody* colLink = c.m_cti.m_colObj;
                    //if (!colLink) continue;
                    const btVector3 &contactPt = node->m_x;
                    //if (onInnerSide(contactPt, left)) {
                        unsigned int closest_ind = 0;
                        bool closest_found = false;
                        for(unsigned int n = 0; !closest_found && n < nodeposvec.size(); n++)
                        {
                            if((contactPt - nodeposvec[n]).length() < 0.0001)
                            {
                                closest_ind = n;
                                closest_found = true;
                            }
                        }
                        assert(closest_found);

                        vattached_node_inds.push_back(closest_ind);
                        appendAnchor(psb, node, rigidBody);
                        //std::cout << "\tappending anchor, closest ind: "<< closest_ind << std::endl;
                }
            }
        }
    }

    bAttached = !bAttached;
}


// Fills in the rcontacs array with contact information between psb and pco
void GripperKinematicObject::getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts)
{
    // custom contact checking adapted from btSoftBody.cpp and btSoftBodyInternals.h
    struct Custom_CollideSDF_RS : btDbvt::ICollide {
        Custom_CollideSDF_RS(btSoftBody::tRContactArray &rcontacts_) : rcontacts(rcontacts_) { }

        void Process(const btDbvtNode* leaf) {
            btSoftBody::Node* node=(btSoftBody::Node*)leaf->data;
            DoNode(*node);
        }

        void DoNode(btSoftBody::Node& n) {
            const btScalar m=n.m_im>0?dynmargin:stamargin;
            btSoftBody::RContact c;
            if (!n.m_battach && psb->checkContact(m_colObj1,n.m_x,m,c.m_cti)) {
                const btScalar  ima=n.m_im;
                const btScalar  imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
                const btScalar  ms=ima+imb;
                if(ms>0) {
                    // there's a lot of extra information we don't need to compute
                    // since we just want to find the contact points
                    c.m_node        =       &n;

                    rcontacts.push_back(c);

                }
            }
        }
        btSoftBody*             psb;
        btCollisionObject*      m_colObj1;
        btRigidBody*    m_rigidBody;
        btScalar                dynmargin;
        btScalar                stamargin;
        btSoftBody::tRContactArray &rcontacts;
    };

    Custom_CollideSDF_RS  docollide(rcontacts);
    btRigidBody*            prb1=btRigidBody::upcast(pco);
    btTransform     wtr=pco->getWorldTransform();

    const btTransform       ctr=pco->getWorldTransform();
    const btScalar          timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
    const btScalar          basemargin=psb->getCollisionShape()->getMargin();
    btVector3                       mins;
    btVector3                       maxs;
    ATTRIBUTE_ALIGNED16(btDbvtVolume)               volume;
    pco->getCollisionShape()->getAabb(     pco->getWorldTransform(),
            mins,
            maxs);
    volume=btDbvtVolume::FromMM(mins,maxs);
    volume.Expand(btVector3(basemargin,basemargin,basemargin));
    docollide.psb           =       psb;
    docollide.m_colObj1 = pco;
    docollide.m_rigidBody = prb1;

    docollide.dynmargin     =       basemargin+timemargin;
    docollide.stamargin     =       basemargin;
    psb->m_ndbvt.collideTV(psb->m_ndbvt.m_root,volume,docollide);
}

// adapted from btSoftBody.cpp (btSoftBody::appendAnchor)
void GripperKinematicObject::appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence)
{
    btSoftBody::Anchor a;
    a.m_node = node;
    a.m_body = body;
    a.m_local = body->getWorldTransform().inverse()*a.m_node->m_x;
    a.m_node->m_battach = true;
    a.m_influence = influence;
    psb->m_anchors.push_back(a);
}

void GripperKinematicObject::releaseAllAnchors(btSoftBody * psb)
{
    psb->m_anchors.clear();
}

const std::vector<size_t>& GripperKinematicObject::getAttachedNodeIndices() const
{
    return vattached_node_inds;
}

// state is only changed by the 'b' and 'n' commands in the original software
// I think this opens and closes one gripper; was used on one auto and one manual gripper
void GripperKinematicObject::step_openclose(btSoftBody * psb)
{
    if (state == GripperState_DONE) return;

    if (state == GripperState_OPENING && bAttached)
    {
        toggleAttach(psb);
    }

    btTransform top_tm;
    btTransform bottom_tm;
    children[0]->motionState->getWorldTransform(top_tm);
    children[1]->motionState->getWorldTransform(bottom_tm);

    float step_size = 0.005f;
    if(state == GripperState_CLOSING)
    {
        top_tm.setOrigin(top_tm.getOrigin() + step_size*top_tm.getBasis().getColumn(2));
        bottom_tm.setOrigin(bottom_tm.getOrigin() - step_size*bottom_tm.getBasis().getColumn(2));
    }
    else if(state == GripperState_OPENING)
    {
        top_tm.setOrigin(top_tm.getOrigin() - step_size*top_tm.getBasis().getColumn(2));
        bottom_tm.setOrigin(bottom_tm.getOrigin() + step_size*bottom_tm.getBasis().getColumn(2));
    }

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);

//        if(state == GripperState_CLOSING && !bAttached)
//            toggleAttach(psb, 0.5);

    float cur_gap_length = (top_tm.getOrigin() - bottom_tm.getOrigin()).length();
    if(state == GripperState_CLOSING && cur_gap_length <= (closed_gap + 2*halfextents[2]))
    {
        state = GripperState_DONE;
        bOpen = false;
        if(!bAttached)
            toggleAttach(psb);

    }
    if(state == GripperState_OPENING && cur_gap_length >= apperture)
    {
        state = GripperState_DONE;
        bOpen = true;

    }

//        float frac = fracElapsed();
//        vals[0] = (1.f - frac)*startVal + frac*endVal;
//        manip->robot->setDOFValues(indices, vals);

//        if (vals[0] == CLOSED_VAL) {
//            attach(true);
//            attach(false);
//        }
}


EnvironmentObject::Ptr GripperKinematicObject::copy(Fork &f) const
{
    Ptr o(new GripperKinematicObject(name, apperture));
    internalCopy(o, f);
    return o;
}

void GripperKinematicObject::internalCopy(GripperKinematicObject::Ptr o, Fork &f) const
{
    o->apperture = apperture;
    o->cur_tm = cur_tm;
    o->bOpen = bOpen;
    o->state = state;
    o->closed_gap = closed_gap;
    o->vattached_node_inds = vattached_node_inds;
    o->halfextents = halfextents;
    o->name = name;
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

const btVector3& GripperKinematicObject::getHalfExtents() const
{
    return halfextents;
}

float GripperKinematicObject::getGripperRadius() const
{
    btTransform top_tf, bottom_tf;
    children[0]->motionState->getWorldTransform(top_tf);
    children[1]->motionState->getWorldTransform(bottom_tf);

    return std::abs(top_tf.getOrigin().z() - bottom_tf.getOrigin().z())/2.0f +
            std::max(halfextents.x(), std::max(halfextents.y(), halfextents.z()));
}

// Get force and torque data for one gripper, size of data vector is two
// --- Added by Mengyao
std::vector<btVector3> GripperKinematicObject::getRopeGripperForce() const
{
    std::vector<btVector3> forceData;
    // top and bottom box
//    int num_boxes_for_gripper = boxes_children.size();
    int num_boxes_for_gripper = children.size();
    for (int child_ind = 0; child_ind < num_boxes_for_gripper; child_ind++)
    {
//        assert(children[child_ind]->rigidBody->hasAnisotropicFriction()
//               && "Grippers don't have anisotropic friction, in custom_scene.cpp, get elemental force" );

        // getTotalForce return m_totalfoce, which is central force on the box(rigid) body
    //    forceData.push_back(children[child_ind]->rigidBody->getAnisotropicFriction());
        forceData.push_back(
                    children[child_ind]->getTotalForce());
                  //  boxes_children[child_ind]->getTotalForce());
    }
    return forceData;
}

// To be rewritten later
std::vector<btVector3> GripperKinematicObject::getGripperTotalTorque() const
{
    return getRopeGripperForce();
}

// Set the stretching vector information --- Added by Mengyao
const std::string GripperKinematicObject::getGripperName()
{
    return name;
}

void GripperKinematicObject::setClothGeoInfoToAnotherGripper(
        Ptr to_gripper,
        const btSoftBody* cloth,
        const int num_x,
        const int num_y)
{
    const std::vector<size_t>& to_gripper_attached_node_inds = to_gripper->getAttachedNodeIndices();
    btVector3 to_gripper_origin = to_gripper->getWorldTransform().getOrigin();
    btVector3 cur_origin = cur_tm.getOrigin();
    std::string to_gripper_name = to_gripper->getGripperName();

    to_another_gripper_info.from_nodes.clear();
    to_another_gripper_info.to_nodes.clear();
    to_another_gripper_info.node_contribution.clear();
    to_another_gripper_info.to_gripper_name = to_gripper_name;

    btScalar min_dis = INFINITY;
    btScalar second_min_dis = INFINITY;
    size_t min_ind = 0;
    size_t second_min_ind = 0;

    if(vattached_node_inds.size()<2)
    {
        assert(false && "number of attached nodes less than 2, no need calling this function. gripper_kinematic_object.cpp");
    }

    // Find the two nearest attached nodes to the center of the "to gripper"
    for (size_t node_ind = 0; node_ind < vattached_node_inds.size(); node_ind++)
    {
        btScalar dis_to_origin = btDistance(cloth->m_nodes[node_ind].m_x, to_gripper_origin);
        if (dis_to_origin < min_dis)
        {
            second_min_dis = min_dis;
            min_dis = dis_to_origin;
            second_min_ind = min_ind;
            min_ind = node_ind;
        }
        else if (dis_to_origin < second_min_dis)
        {
            second_min_dis = dis_to_origin;
            second_min_ind = node_ind;
        }
    }

    min_ind = vattached_node_inds.at(min_ind);
    second_min_ind = vattached_node_inds.at(second_min_ind);

    // Find the relative position of grippers, highly depends on how the
    // #makeClothTwoGrippers# works
    int position_factor = 0;

    if ((name.compare("auto_gripper0") == 0) || (name.compare("manual_gripper0") == 0))
    {
        if ((to_gripper_name.compare("auto_gripper0") == 0) || (to_gripper_name.compare("manual_gripper0") == 0))
        {
            assert(false && "stretching vectors need more than one gripper to be defined. gripper_kinematic_object.cpp");
        }
        position_factor = 1;
    }
    else if ((name.compare("auto_gripper1") == 0) || (name.compare("manual_gripper1") == 0))
    {
        if ((to_gripper_name.compare("auto_gripper1") == 0) || (to_gripper_name.compare("manual_gripper1") == 0))
        {
            assert(false && "stretching vectors need more than one gripper to be defined. gripper_kinematic_object.cpp");
        }
        position_factor = -1;
    }
    else
    {
        assert(false && "failed to set stretching detection vector for the cloth, in gripper_kinematic_object.cpp");
    }

    // Set the tracking nodes pair for stretching detection vector
    double total_distance = btDistance(cloth->m_nodes[min_ind].m_x, to_gripper_origin)
            + btDistance(cloth->m_nodes[min_ind].m_x, cur_origin);

    // If the center of the gripper is on the line inbetween two origins.
    if(total_distance == btDistance(cur_origin, to_gripper_origin))
    {
        to_another_gripper_info.from_nodes.push_back(min_ind);
        to_another_gripper_info.to_nodes.push_back(min_ind + position_factor * num_x);
        assert(((min_ind + position_factor * num_x) < cloth->m_nodes.size()) && "stretching info nodes outside bound");
        double full_contribution = 1.0;
        to_another_gripper_info.node_contribution.push_back(full_contribution);
    }
    else
    {
        to_another_gripper_info.from_nodes.push_back(min_ind);
        to_another_gripper_info.to_nodes.push_back(min_ind + position_factor * num_x);

        /*
        std::cout << "min_ind: " << min_ind << std::endl;
        std::cout << "num_x: " << num_x << std::endl;
        std::cout << "nodes on the cloth: " << cloth->m_nodes.size() << std::endl;
        */
        assert(((min_ind + position_factor * num_x) < cloth->m_nodes.size()) || "stretching info nodes outside bound");

        to_another_gripper_info.from_nodes.push_back(second_min_ind);
        to_another_gripper_info.to_nodes.push_back(second_min_ind + position_factor * num_x);
        assert(((second_min_ind + position_factor * num_x) < cloth->m_nodes.size()) || "stretching info nodes outside bound");

        double min_con = btDistance(cur_origin, cloth->m_nodes[min_ind].m_x);
        double second_min_con = btDistance(cur_origin, cloth->m_nodes[second_min_ind].m_x);
        double sum_con = min_con + second_min_con;
        to_another_gripper_info.node_contribution.push_back(min_con/sum_con);
        to_another_gripper_info.node_contribution.push_back(second_min_con/sum_con);
    }

}



std::ostream& operator<< (std::ostream& stream, const GripperKinematicObject& gripper)
{
    stream << "Gripper:" << gripper.name << std::endl
            << PrettyPrint::PrettyPrint(gripper.cur_tm) << std::endl
            << "apperture: " << gripper.apperture
            << " open: " << gripper.bOpen
            << " attached: " << gripper.bAttached
            << " closed_gap: " << gripper.closed_gap
            << " half extents: " << " x: " << gripper.halfextents.x()
                                 << " y: " << gripper.halfextents.y()
                                 << " z: " << gripper.halfextents.z()
            << " attached nodes:";

    for (size_t ind: gripper.vattached_node_inds)
    {
        stream << " " << ind;
    }

    stream << std::endl;

    return stream;
}
