#include "colab_cloth.h"



void nodeArrayToNodePosVector(const btAlignedObjectArray<btSoftBody::Node> &m_nodes, std::vector<btVector3> &nodeposvec)
{
    nodeposvec.resize(m_nodes.size());
    for(int i =0; i < m_nodes.size(); i++)
    {
        nodeposvec[i] = m_nodes[i].m_x;
    }
}


void GripperKinematicObject::applyTransform(btTransform tm)
{

    //btTransform _tm = getWorldTransform();
    //cout << "tm: " << _tm.getOrigin()[0] << " " << _tm.getOrigin()[1] << " " << _tm.getOrigin()[2] << endl;
    //btTransform a = _tm*tm;
    //cout << "tm: " << a.getOrigin()[0] << " " << a.getOrigin()[1] << " " <<  a.getOrigin()[2] << endl;
    setWorldTransform(getWorldTransform()*tm);
}


void GripperKinematicObject::translate(btVector3 transvec)
{
    btTransform tm = getWorldTransform();
    tm.setOrigin(tm.getOrigin() + transvec);
    setWorldTransform(tm);
}

void GripperKinematicObject::toggleattach(btSoftBody * psb, double radius) {

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
#ifdef USE_RADIUS_CONTACT
        std::cout << "use radius contact?????" << std::endl;
        if(radius == 0)
            radius = halfextents[0];
        btTransform top_tm;
        children[0]->motionState->getWorldTransform(top_tm);
        btTransform bottom_tm;
        children[1]->motionState->getWorldTransform(bottom_tm);
        int closest_body = -1;
        for(int j = 0; j < psb->m_nodes.size(); j++)
        {
            if((psb->m_nodes[j].m_x - cur_tm.getOrigin()).length() < radius)
            {
                if( (psb->m_nodes[j].m_x - top_tm.getOrigin()).length() < (psb->m_nodes[j].m_x - bottom_tm.getOrigin()).length() )
                    closest_body = 0;
                else
                    closest_body = 1;

                vattached_node_inds.push_back(j);
                appendAnchor(psb, &psb->m_nodes[j], children[closest_body]->rigidBody.get());
                cout << "\tappending anchor, closest ind: "<< j << "\n";

            }
        }
#else
        std::vector<btVector3> nodeposvec;
        nodeArrayToNodePosVector(psb->m_nodes, nodeposvec);

        for(int k = 0; k < 2; k++)
        {
            BoxObject::Ptr part = children[k];

            btRigidBody* rigidBody = part->rigidBody.get();
            btSoftBody::tRContactArray rcontacts;
            getContactPointsWith(psb, rigidBody, rcontacts);
            cout << "got " << rcontacts.size() << " contacts\n";

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
                    int closest_ind = -1;
                    for(int n = 0; n < nodeposvec.size(); n++)
                    {
                        if((contactPt - nodeposvec[n]).length() < 0.0001)
                        {
                            closest_ind = n;
                            break;
                        }
                    }
                    assert(closest_ind!=-1);

                    vattached_node_inds.push_back(closest_ind);
                    appendAnchor(psb, node, rigidBody);
                    cout << "\tappending anchor, closest ind: "<< closest_ind << "\n";
            }
        }
#endif
    }

    bAttached = !bAttached;
}

// Fills in the rcontacs array with contact information between psb and pco
void GripperKinematicObject::getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts) {
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
    pco->getCollisionShape()->getAabb(      pco->getWorldTransform(),
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
void GripperKinematicObject::appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence) {
    btSoftBody::Anchor a;
    a.m_node = node;
    a.m_body = body;
    a.m_local = body->getWorldTransform().inverse()*a.m_node->m_x;
    a.m_node->m_battach = 1;
    a.m_influence = influence;
    psb->m_anchors.push_back(a);
}




GripperKinematicObject::GripperKinematicObject(btVector4 color)
{
    bAttached = false;
    closed_gap = 0.1;
#ifdef ROPE
    apperture = 0.5;
#else
    apperture = 2;
#endif

    halfextents = btVector3(.3,.3,0.1);
    BoxObject::Ptr top_jaw(new BoxObject(0, halfextents, btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, apperture/2)),true));
    top_jaw->setColor(color[0],color[1],color[2],color[3]);
    BoxObject::Ptr bottom_jaw(new BoxObject(0, halfextents, btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,0,-apperture/2)),true));
    bottom_jaw->setColor(color[0],color[1],color[2],color[3]);
    top_jaw->motionState->getWorldTransform(cur_tm);
    cur_tm.setOrigin(cur_tm.getOrigin() - btVector3(0,0,-apperture/2));
    bOpen = true;
    children.push_back(top_jaw);
    children.push_back(bottom_jaw);
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

void GripperKinematicObject::toggle()
{
    btTransform top_tm;    btTransform bottom_tm;
    children[0]->motionState->getWorldTransform(top_tm);
    children[1]->motionState->getWorldTransform(bottom_tm);

    if(bOpen)
    {
        //btTransform top_offset = cur_tm.inverse()*top_tm;
        //float close_length = (1+closed_gap)*top_offset.getOrigin()[2] - children[0]->halfExtents[2];
        //float close_length = (apperture/2 - children[0]->halfExtents[2] + closed_gap/2);
        top_tm.setOrigin(cur_tm.getOrigin() + cur_tm.getBasis().getColumn(2)*(children[0]->halfExtents[2] + closed_gap/2));
        bottom_tm.setOrigin(cur_tm.getOrigin() - cur_tm.getBasis().getColumn(2)*(children[1]->halfExtents[2] + closed_gap/2));
    }
    else
    {
        top_tm.setOrigin(cur_tm.getOrigin() - cur_tm.getBasis().getColumn(2)*(apperture/2));
        bottom_tm.setOrigin(cur_tm.getOrigin() + cur_tm.getBasis().getColumn(2)*(apperture/2));
    }

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);

    bOpen = !bOpen;
}





void CustomScene::simulateInNewFork(StepState& innerstate, float sim_time, btTransform& left_gripper1_tm, btTransform& left_gripper2_tm)
{

    innerstate.bullet.reset(new BulletInstance);
    innerstate.bullet->setGravity(BulletConfig::gravity);
    innerstate.osg.reset(new OSGInstance);
    innerstate.fork.reset(new Fork(env, innerstate.bullet, innerstate.osg));
    innerstate.cloth = boost::static_pointer_cast<BulletSoftObject>(innerstate.fork->forkOf(clothptr));
    innerstate.left_gripper1 = boost::static_pointer_cast<GripperKinematicObject>(innerstate.fork->forkOf(left_gripper1));
    innerstate.left_gripper2 = boost::static_pointer_cast<GripperKinematicObject>(innerstate.fork->forkOf(left_gripper2));

    innerstate.left_gripper1->applyTransform(left_gripper1_tm);
    innerstate.left_gripper2->applyTransform(left_gripper2_tm);

    float time = sim_time;
    while (time > 0) {
        // run pre-step callbacks
        //for (int j = 0; j < prestepCallbacks.size(); ++j)
        //    prestepCallbacks[j]();

        innerstate.fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
        time -= BulletConfig::dt;
    }

}


double CustomScene::getDistfromNodeToClosestAttachedNodeInGripper(GripperKinematicObject::Ptr gripper, int input_ind, int &closest_ind)
{
    double min_dist = 1000000;
    closest_ind = -1;
    for(int i =0; i < gripper->vattached_node_inds.size(); i++)
    {
        double new_dist = deformableobject_distance_matrix(gripper->vattached_node_inds[i],input_ind);
        if(new_dist < min_dist)
        {
            min_dist = new_dist;
            closest_ind = gripper->vattached_node_inds[i];
        }
    }

    return min_dist;


}


void printMatrixXf(Eigen::MatrixXf mat)
{
    cout << endl;
    for(int i = 0; i < mat.rows(); i++)
    {
        for(int j = 0; j < mat.cols();j++)
        {
            cout << mat(i,j) << " ";

        }
        cout << endl;
    }
}

Eigen::MatrixXf CustomScene::computePointsOnGripperJacobian(std::vector<btVector3>& points_in_world_frame,std::vector<int>& autogripper_indices_per_point)
{
   int num_Jcols_per_gripper = 3;
#ifdef DO_ROTATION
    #ifdef USE_QUATERNION
        ///NOT IMPLEMENTED!!!!
        num_Jcols_per_gripper += 4;
    #else
        num_Jcols_per_gripper += 3;
    #endif
#endif


    Eigen::MatrixXf J(points_in_world_frame.size()*3,num_Jcols_per_gripper*num_auto_grippers);
    GripperKinematicObject::Ptr gripper;
    Eigen::VectorXf  V_pos(points_in_world_frame.size()*3);

    for(int g = 0; g < num_auto_grippers; g++)
    {
        if(g == 0)
            gripper = left_gripper1;
        else if(g == 1)
            gripper = left_gripper2;

        btVector3 transvec;
        for(int i = 0; i < num_Jcols_per_gripper; i++)
        {

            for(int k = 0; k < points_in_world_frame.size(); k++)
            {
                //no effect if this point doesn't correspond to this gripper
                if(autogripper_indices_per_point[k] != g)
                {
                    for(int j = 0; j < 3; j++)
                        V_pos(3*k + j) = 0;
                    //cout << V_pos[0] << " " << V_pos[1] << " " << V_pos[2] << endl;
                    continue;
                }


                if(i == 0)
                    transvec = btVector3(1,0,0);
                else if(i == 1)
                    transvec = btVector3(0,1,0);
                else if(i == 2)
                    transvec = btVector3(0,0,1);
                else if(i == 3)
                    transvec =  (gripper->getWorldTransform()*btVector4(1,0,0,0)).cross(points_in_world_frame[k] - gripper->getWorldTransform().getOrigin());
                else if(i == 4)
                    transvec =  (gripper->getWorldTransform()*btVector4(0,1,0,0)).cross(points_in_world_frame[k] - gripper->getWorldTransform().getOrigin());
                else if(i == 5)
                    transvec =  (gripper->getWorldTransform()*btVector4(0,0,1,0)).cross(points_in_world_frame[k] - gripper->getWorldTransform().getOrigin());

                for(int j = 0; j < 3; j++)
                    V_pos(3*k + j) = transvec[j];

                //cout << V_pos[0] << " " << V_pos[1] << " " << V_pos[2] << endl;

            }

            J.col(num_Jcols_per_gripper*g + i) = V_pos;
            //cout << endl << J << endl; //printmatrix

        }

    }
    return J;
}


Eigen::MatrixXf CustomScene::computeJacobian_approx()
{
#ifdef ROPE
    double dropoff_const = 0.5;//0.5;
#else
    double dropoff_const = 0.7;//0.7 for colab folding;//;
#endif
    int numnodes = getNumDeformableObjectNodes();//clothptr->softBody->m_nodes.size();

    std::vector<btTransform> perts;
    float step_length = 0.2;
    float rot_angle = 0.2;
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(step_length,0,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,step_length,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,step_length)));
#ifdef DO_ROTATION
    #ifdef USE_QUATERNION
        ///NOT IMPLEMENTED!!!!
        perts.push_back(btTransform(btQuaternion(btVector3(1,0,0),rot_angle),btVector3(0,0,0)));
        perts.push_back(btTransform(btQuaternion(btVector3(0,1,0),rot_angle),btVector3(0,0,0)));
        perts.push_back(btTransform(btQuaternion(btVector3(0,0,1),rot_angle),btVector3(0,0,0)));
    #else
        perts.push_back(btTransform(btQuaternion(btVector3(1,0,0),rot_angle),btVector3(0,0,0)));
        perts.push_back(btTransform(btQuaternion(btVector3(0,1,0),rot_angle),btVector3(0,0,0)));
        perts.push_back(btTransform(btQuaternion(btVector3(0,0,1),rot_angle),btVector3(0,0,0)));
    #endif
#endif


    Eigen::MatrixXf J(numnodes*3,perts.size()*num_auto_grippers);
    GripperKinematicObject::Ptr gripper;

    std::vector<btVector3> rot_line_pnts;
    std::vector<btVector4> plot_cols;


//    Eigen::VectorXf  V_before(numnodes*3);
//    for(int k = 0; k < numnodes; k++)
//    {
//        for(int j = 0; j < 3; j++)
//            V_before(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
//    }
    omp_set_num_threads(4);

    std::vector<btVector3> node_pos;
    getDeformableObjectNodes(node_pos);

    for(int g = 0; g < num_auto_grippers; g++)
    {
        if(g == 0)
            gripper = left_gripper1;
        if(g == 1)
            gripper = left_gripper2;

        #pragma omp parallel shared(J)
        {

        #pragma omp for
        for(int i = 0; i < perts.size(); i++)
        {
            Eigen::VectorXf  V_pos(numnodes*3);

//                    btTransform dummy_tm(btQuaternion(0,0,0,1),btVector3(0,0,0));
//                    StepState innerstate;

//                    if( i >= 3)
//                    {
//                        if(g == 0)
//                            simulateInNewFork(innerstate, jacobian_sim_time, perts[i],dummy_tm);
//                        else
//                            simulateInNewFork(innerstate, jacobian_sim_time, dummy_tm,perts[i]);

//                        Eigen::VectorXf  V_after(V_before);
//                        for(int k = 0; k < numnodes; k++)
//                        {
//                            for(int j = 0; j < 3; j++)
//                                V_after(3*k + j) = innerstate.cloth->softBody->m_nodes[k].m_x[j];
//                        }
//
//                        V_pos = (V_after - V_before)/rot_angle;

//                    }

                for(int k = 0; k < numnodes; k++)
                {
                    int closest_ind;
                    double dist = getDistfromNodeToClosestAttachedNodeInGripper(gripper, k, closest_ind);
                    //if(k < 10) cout << "dist: " << dist << " node_map " << gripper_node_distance_map[g][k] << endl;

                    if(i < 3) //translation
                    {
                        btVector3 transvec = ((gripper->getWorldTransform()*perts[i]).getOrigin() - gripper->getWorldTransform().getOrigin())*exp(-dist*dropoff_const)/step_length;
                        //WRONG: btVector3 transvec = perts[i].getOrigin()*exp(-dist*dropoff_const)/step_length;
                        //btVector3 transvec = perts[i].getOrigin()*exp(-gripper_node_distance_map[g][k]*dropoff_const);
                        for(int j = 0; j < 3; j++)
                            V_pos(3*k + j) = transvec[j];




    //                    if(i == 2)
    //                    {
    //                        rot_line_pnts.push_back(clothptr->softBody->m_nodes[k].m_x);
    //                        rot_line_pnts.push_back(clothptr->softBody->m_nodes[k].m_x + transvec);
    //                        plot_cols.push_back(btVector4(1,0,0,1));
    //                        //cout <<"z " << k << ": transvec " << transvec[0] << " " << transvec[1] << " " << transvec[2] << " dist: " << dist << "(" << closest_ind << ")" << endl;
    //                    }
                    }
                    else // rotation
                    {


                        //TODO: Use cross product instead

                        //get the vector of translation induced at closest attached point by the rotation about the center of the gripper
                        //btTransform T0_attached = btTransform(btQuaternion(0,0,0,1),clothptr->softBody->m_nodes[closest_ind].m_x);
                        btTransform T0_attached = btTransform(btQuaternion(0,0,0,1),node_pos[closest_ind]);
                        btTransform T0_center = gripper->getWorldTransform();
                        //btTransform Tcenter_attached= btTransform(T0_center.getRotation(), V0_attached - T0_center.getOrigin());
                        btTransform Tcenter_attached = T0_center.inverse()*T0_attached;
                        btTransform T0_newattached =  T0_center*perts[i]*Tcenter_attached;
                        btVector3 transvec = (T0_attached.inverse()*T0_newattached).getOrigin()/rot_angle * exp(-dist*dropoff_const);



                        for(int j = 0; j < 3; j++)
                            V_pos(3*k + j) = transvec[j]*ROTATION_SCALING;

//                        if(i == 3)
//                        {

//                            btVector3 unitvec = transvec*step_length;///transvec.length();

//                            rot_line_pnts.push_back(clothptr->softBody->m_nodes[k].m_x);
//                            rot_line_pnts.push_back(clothptr->softBody->m_nodes[k].m_x + unitvec);
//                            plot_cols.push_back(btVector4(1,0,0,1));

//                            unitvec = innerstate.cloth->softBody->m_nodes[k].m_x - clothptr->softBody->m_nodes[k].m_x;
//                            unitvec = unitvec/unitvec.length()*transvec.length()*step_length;

//                            rot_line_pnts.push_back(clothptr->softBody->m_nodes[k].m_x);
//                            rot_line_pnts.push_back(clothptr->softBody->m_nodes[k].m_x + unitvec);
//                            plot_cols.push_back(btVector4(0,0,1,1));


//                        }

//                            if(i == 3)
//                            {

//                                rot_line_pnts.push_back(clothptr->softBody->m_nodes[k].m_x);
//                                rot_line_pnts.push_back(clothptr->softBody->m_nodes[k].m_x + transvec);
//                                plot_cols.push_back(btVector4(1,0,0,1));

//                                //cout << k << ": transvec " << transvec[0] << " " << transvec[1] << " " << transvec[2] << " dist: " << dist << endl;
//                            }

                    }


                }
                J.col(perts.size()*g + i) = V_pos;
            }
            }//end omp

    }


    //rot_lines->setPoints(rot_line_pnts,plot_cols);

    return J;
}

Eigen::MatrixXf CustomScene::computeJacobian_parallel()
{
    boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());

    //printf("starting jacobian computation\n");
    //stopLoop();
    bool bBackupLoopState = loopState.skip_step;
    loopState.skip_step = true;

    int numnodes = clothptr->softBody->m_nodes.size();
    Eigen::VectorXf  V_before(numnodes*3);


    for(int k = 0; k < numnodes; k++)
    {
        for(int j = 0; j < 3; j++)
            V_before(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
    }



    std::vector<btTransform> perts;
    float step_length = 0.2;
    float rot_angle = 0.2;
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(step_length,0,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,step_length,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,step_length)));
#ifdef DO_ROTATION
    perts.push_back(btTransform(btQuaternion(btVector3(1,0,0),rot_angle),btVector3(0,0,0)));
    perts.push_back(btTransform(btQuaternion(btVector3(0,1,0),rot_angle),btVector3(0,0,0)));
    perts.push_back(btTransform(btQuaternion(btVector3(0,0,1),rot_angle),btVector3(0,0,0)));
    omp_set_num_threads(7); //need to find a better way to do this
#else
    omp_set_num_threads(4);
#endif

    Eigen::MatrixXf J(numnodes*3,perts.size());
    #pragma omp parallel shared(J)
    {

        //schedule(static, 1)
        #pragma omp for nowait
        for(int i = 0; i < perts.size(); i++)
        {

            btTransform dummy_tm;
            StepState innerstate;
            simulateInNewFork(innerstate, jacobian_sim_time, perts[i],dummy_tm);

            Eigen::VectorXf  V_after(V_before);
            for(int k = 0; k < numnodes; k++)
            {
                for(int j = 0; j < 3; j++)
                    V_after(3*k + j) = innerstate.cloth->softBody->m_nodes[k].m_x[j];
            }
            float divider;
            if(i < 3)
                divider = step_length;
            else
                divider = rot_angle;

            J.col(i) = (V_after - V_before)/divider;
        }
    }

    //cout << J<< endl;
    boost::posix_time::ptime endTick(boost::posix_time::microsec_clock::local_time());
    //std::cout << "time: " << boost::posix_time::to_simple_string(endTick - begTick) << std::endl;

    loopState.skip_step = bBackupLoopState;
    //printf("done jacobian computation\n");
    return J;


}





Eigen::MatrixXf CustomScene::computeJacobian()
{
    boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());

    //printf("starting jacobian computation\n");
    //stopLoop();
    bool bBackupLoopState = loopState.skip_step;
    loopState.skip_step = true;

    int numnodes = clothptr->softBody->m_nodes.size();
    Eigen::VectorXf  V_before(numnodes*3);
    Eigen::VectorXf  V_after(V_before);

    for(int k = 0; k < numnodes; k++)
    {
        for(int j = 0; j < 3; j++)
            V_before(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
    }

    Eigen::MatrixXf J(numnodes*3,3);

    std::vector<btTransform> perts;
    float step_length = 0.2;
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(step_length,0,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,step_length,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,step_length)));
    float time;

    for(int i = 0; i < 3 ; i++)
    {
        createFork();
        swapFork(); //now pointers are set to the forked objects

        //apply perturbation
        left_gripper1->applyTransform(perts[i]);

        time = jacobian_sim_time;
        while (time > 0) {
            float startTime=viewer.getFrameStamp()->getSimulationTime(), endTime;
            if (syncTime && drawingOn)
                endTime = viewer.getFrameStamp()->getSimulationTime();

            // run pre-step callbacks
            //for (int j = 0; j < prestepCallbackssize(); ++j)
            //    prestepCallbacks[j]();

            fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            draw();

            if (syncTime && drawingOn) {
                float timeLeft = BulletConfig::dt - (endTime - startTime);
                idleFor(timeLeft);
                startTime = endTime + timeLeft;
            }


            time -= BulletConfig::dt;

        }

        for(int k = 0; k < numnodes; k++)
        {
            for(int j = 0; j < 3; j++)
                V_after(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
        }

        destroyFork();
        J.col(i) = (V_after - V_before)/step_length;
    }

    //cout << J<< endl;
    boost::posix_time::ptime endTick(boost::posix_time::microsec_clock::local_time());
    //std::cout << "time: " << boost::posix_time::to_simple_string(endTick - begTick) << std::endl;


    loopState.skip_step = bBackupLoopState;
    //printf("done jacobian computation\n");
    return J;
}

Eigen::MatrixXf pinv(const Eigen::MatrixXf &a)
{
    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method

    if ( a.rows()<a.cols() )
    {
        cout << "pinv error!" << endl;
        return Eigen::MatrixXf();
    }

    // SVD
    Eigen::JacobiSVD< Eigen::MatrixXf> svdA(a,Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXf vSingular = svdA.singularValues();

    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Eigen::MatrixXf vPseudoInvertedSingular(svdA.matrixV().cols(),1);

    for (int iRow =0; iRow<vSingular.rows(); iRow++)
    {
        if ( fabs(vSingular(iRow))<=1e-10 ) // Todo : Put epsilon in parameter
        {
            vPseudoInvertedSingular(iRow,0)=0.;
        }
        else
        {
            vPseudoInvertedSingular(iRow,0)=1./vSingular(iRow);
        }
    }

    // A little optimization here
    Eigen::MatrixXf mAdjointU = svdA.matrixU().adjoint().block(0,0,vSingular.rows(),svdA.matrixU().adjoint().cols());

    // Pseudo-Inversion : V * S * U'
    return (svdA.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * mAdjointU  ;

}

float box_muller(float m, float s)	/* normal random variate generator */
{				        /* mean m, standard deviation s */
        float x1, x2, w, y1;
        static float y2;
        static int use_last = 0;

        if (use_last)		        /* use value from previous call */
        {
                y1 = y2;
                use_last = 0;
        }
        else
        {
                do {
                        x1 = 2.0 * (float)rand()/(float)RAND_MAX - 1.0;
                        x2 = 2.0 * (float)rand()/(float)RAND_MAX - 1.0;
                        w = x1 * x1 + x2 * x2;
                } while ( w >= 1.0 );

                w = sqrt( (-2.0 * log( w ) ) / w );
                y1 = x1 * w;
                y2 = x2 * w;
                use_last = 1;
        }

        return( m + y1 * s );
}

void CustomScene::getDeformableObjectNodes(std::vector<btVector3>& vnodes)
{
#ifdef ROPE
    vnodes = ropePtr->getNodes();
#else
    nodeArrayToNodePosVector(clothptr->softBody->m_nodes, vnodes);
#endif
}

int CustomScene::getNumDeformableObjectNodes()
{
#ifdef ROPE
    return ropePtr->getNodes().size();
#else
    return clothptr->softBody->m_nodes.size();
#endif

}



//this is getting called before the step loop
void CustomScene::doJTracking()
{

    //if(!clothptr)
    //    return;

    //this loop is already being executed by someone else, abort
    if(bInTrackingLoop)
        return;
    else
        bInTrackingLoop = true;




    itrnumber++;
    if(itrnumber != 3)
    {
        loopState.skip_step = false;
        bInTrackingLoop = false;
        return;
    }
    else
    {
        itrnumber = 0;
    }


    loopState.skip_step = true;


    // what is this? try to turn it to True;
    bool bAvoidObstacle = false;


#ifdef DO_ROTATION
    int dof_per_gripper = 6;
#else
    int dof_per_gripper = 3;
#endif
    Eigen::MatrixXf Jcollision = Eigen::MatrixXf::Zero(num_auto_grippers * 3, num_auto_grippers*dof_per_gripper);
    Eigen::VectorXf V_step_collision =  Eigen::VectorXf::Zero(num_auto_grippers * 3);
///////////////

#ifdef ROPE
    BulletObject::Ptr obj = o;
    //BulletObject::Ptr obj = o;
#else
    BulletObject::Ptr obj = table;
#endif
    GripperKinematicObject::Ptr gripper;
    std::vector<float> vclosest_dist(num_auto_grippers);
    
    // originall this is obj:::
    // below this is obstacle avoidance

    /*
    The situation is, even when I comment out the collision avoidance part,
    sometimes the gripper still do not directly move towards the goal;
    I know the rope scale affected this, so that is one thing to modify;

    The second thing is to figure out how to move to the goal even when the objective
    may not decrease fastest;
    */
    
    if(obj)
    {
        
        std::vector<btVector3> plotpoints;
        std::vector<btVector4> plotcols;
        for(int g =0; g < num_auto_grippers; g++) {
            
            if(g == 0)
                gripper = left_gripper1;
            else if(g == 1)
                gripper = left_gripper2;

            vclosest_dist[g] = BT_LARGE_FLOAT;

            // btGjkEpaPenetrationDepthSolver epaSolver;
            // btPointCollector gjkOutput;

            // btGjkPairDetector convexConvex(dynamic_cast<btBoxShape*> (gripper->getChildren()[0]->collisionShape.get()),dynamic_cast<btConvexShape*> (obj->collisionShape.get()),&sGjkSimplexSolver,&epaSolver);

            // btGjkPairDetector::ClosestPointInput input;

            /*
            Manually calculate the closest points, and penetration distance;
            use the center of the gripper;
            and the center of the torus;
            Since the torus is just a set of triangle mesh, I can get the point closest on the object;
            maybe even make is rounder;
            */


            /*
            



            */
            // center of the current gripper top we are tracking;
            gripperX = gripper->getChildren()[0]->rigidBody->getCenterOfMassTransform().getOrigin()[0];
            gripperY = (gripper->getChildren()[0]->rigidBody->getCenterOfMassTransform().getOrigin()[1] + 
                gripper->getChildren()[1]->rigidBody->getCenterOfMassTransform().getOrigin()[1]) / 2;
            gripperZ = gripper->getChildren()[0]->rigidBody->getCenterOfMassTransform().getOrigin()[2];

            // center of the static torus;
            torusX = o->rigidBody->getCenterOfMassTransform().getOrigin()[0];
            torusY = o->rigidBody->getCenterOfMassTransform().getOrigin()[1];
            torusZ = o->rigidBody->getCenterOfMassTransform().getOrigin()[2];

            distanceGT = sqrt((torusX-gripperX)*(torusX-gripperX) + 
                (torusY-gripperY)*(torusY-gripperY) + (torusZ-gripperZ)*(torusZ-gripperZ));

            // Now finds the closest points

            // 
            //cout << "center distance: " << distanceGT << endl;
            distanceXZ = sqrt((torusX-gripperX)*(torusX-gripperX) + 
                (torusZ-gripperZ)*(torusZ-gripperZ));
            
            //cout << "distanceXZ: " << distanceXZ <<", " << torusX << ", " << torusZ << ", " << gripperX << ", " << gripperZ << endl;
            pointOnTorusX = (2 / distanceGT) * (gripperX - torusX) + torusX;
            pointOnTorusZ = (2 / distanceGT) * (gripperZ - torusZ) + torusZ;
            distanceToTorus = sqrt((pointOnTorusX-gripperX)*(pointOnTorusX-gripperX) + 
                (pointOnTorusZ-gripperZ)*(pointOnTorusZ-gripperZ) + 
                (torusY-gripperY)*(torusY-gripperY));
            px = (torusHeight/distanceToTorus) * (gripperX-pointOnTorusX) + pointOnTorusX;
            py = (torusHeight/distanceToTorus) * (gripperY-torusY) + torusY;
            pz = (torusHeight/distanceToTorus) * (gripperZ-pointOnTorusZ) + pointOnTorusZ;
            // case 1, outside the larger radius;
            // if (distanceXZ > (torusRadius + 2 * torusHeight)) {
            //     cout << g << " outside the large circle" << endl;
            //     // closest point should be outside
            //     cout << "closest point: " << px << ", " << py << ", " << pz << endl;
            // }


            // // case 2, inside the smaller radius;
            // else if (distanceXZ < (torusRadius - 2 * torusHeight)) {
            //     cout << g << " inside small circle" << endl;
            //     cout << "closest point: " << px << ", " << py << ", " << pz << endl;
            // }


            // // case 3, between the two radius;
            // else {
            //     cout << g << " between two circles;" << endl;
            //     cout << "closest point: " << px << ", " << py << ", " << pz << endl;
            // }

            // gripper->children[0]->motionState->getWorldTransform(input.m_transformA);
            // // m_transformA: world configuration of the gripper
            // obj->motionState->getWorldTransform(input.m_transformB);
            // // m_transformB: world configuration of the collision static Object
            // input.m_maximumDistanceSquared = BT_LARGE_FLOAT;
            // gjkOutput.m_distance = BT_LARGE_FLOAT;
            // convexConvex.getClosestPoints(input, gjkOutput, 0);
            // m_pointInWorld: closest point on the static object;
            // m_normalOnBInWorld: closestPoint normal, pointing from the point on the "gripper" to closest point on 
            // static object; 
            anotherRadius = 1.5;
            // if (gjkOutput.m_hasResult)
            // {
                   // cout << "has result" << endl;
                   // printf("distance: %10.4f\n", gjkOutput.m_distance);
                    //
                    // endPt: closest point on gripper
                   // btVector3 endPt = gjkOutput.m_pointInWorld + gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;
                   // new endPt, using manually calculated closest points;
                    btVector3 endPt = btVector3(
                        (anotherRadius/distanceToTorus)*(px-gripperX)+gripperX, 
                        (anotherRadius/distanceToTorus)*(py-gripperY)+gripperY, 
                        (anotherRadius/distanceToTorus)*(pz-gripperZ)+gripperZ); 
                    distanceToTorus -= 1.5;
                   //btVector3 startPt = (input.m_transformB*input.m_transformB.inverse())(gjkOutput.m_pointInWorld);
                    btVector3 startPt = btVector3(px, py, pz);
                    
                   // new startPt, using manually calculated closest points;
                    //btVector3 startPt = btVector3(px, py, pz);

                   // cout << "second: " << gjkOutput.m_pointInWorld[0] << ", " << gjkOutput.m_pointInWorld[1] << ", " << gjkOutput.m_pointInWorld[2] << ", " << gjkOutput.m_normalOnBInWorld[0] << ", " << gjkOutput.m_normalOnBInWorld[1] << ", " << gjkOutput.m_normalOnBInWorld[2] << endl;
                   plotpoints.push_back(startPt);
                   plotpoints.push_back(endPt);

                   // if(gjkOutput.m_distance < 0.5) {
                       bAvoidObstacle = true;
                       std::vector<btVector3> jacpoints;
                       std::vector<int> jacpoint_grippers;
                       jacpoints.push_back(endPt);
                       jacpoint_grippers.push_back(g);
                       Jcollision.block(g*3,0,3,Jcollision.cols()) = computePointsOnGripperJacobian(jacpoints,jacpoint_grippers);
                       Eigen::VectorXf V_coll_step(3);
                       V_coll_step[0] = endPt[0] - startPt[0];
                       V_coll_step[1] = endPt[1] - startPt[1];
                       V_coll_step[2] = endPt[2] - startPt[2];
                       V_coll_step = V_coll_step/V_coll_step.norm();

                       plotcols.push_back(btVector4(1,0,0,1));


                       // when to revert the direction
                       // why not working? 
                       // if(gjkOutput.m_distance < vclosest_dist[g])
                       // {
                       if (distanceToTorus < vclosest_dist[g]) {

                            vclosest_dist[g] = distanceToTorus;
                            // originally 0
                            if(vclosest_dist[g] < 0) {
                                V_coll_step = -V_coll_step;
                                
                            }

                       }

                       // end reverting the direction
                       V_step_collision[g*3 + 0] = V_coll_step[0];
                       V_step_collision[g*3 + 1] = V_coll_step[1];
                       V_step_collision[g*3 + 2] = V_coll_step[2];
                   //}
                   //else
                   //    plotcols.push_back(btVector4(0,0,1,1));
                 // }
                 //rot_lines->setPoints(plotpoints,plotcols);

            }

            //cout << "min dis: " << vclosest_dist[g] << ", " << V_step_collision[g*3 + 0] << ", " << V_step_collision[g*3 + 1] << ", " << V_step_collision[g*3 + 2] << end;

    }
    else
    {
        //cout << "No object found for collision avoidance!" << endl;
    }

//////////////////

    int numnodes = getNumDeformableObjectNodes();

    float approx_thresh = 5;

    float step_limit = 0.05;
    Eigen::VectorXf V_step(numnodes*3);
    Eigen::VectorXf V_delta(numnodes*3);

    Eigen::VectorXf V_trans;
    //btVector3 transvec;
    btTransform transtm1,transtm2;
    Eigen::MatrixXf J;

    //currently this while loop only goes through once before exiting
    while(bTracking)
    {
        for(int i = 0; i < numnodes*3;i++)
        {
            V_step(i) = 0;
            V_delta(i) = 0;

        }
        float error = 0;
        std::vector<btVector3> plotpoints;
        std::vector<btVector4> plotcols;
        float node_change = 0;

        std::vector<btVector3> raw_new_nodes;
        std::vector<btVector3> clean_new_nodes;//only for ground-truth


        //low-pass filter
        float mu = 1.0; //(0 = no change, 1.0 = no filtering)
        getDeformableObjectNodes(raw_new_nodes);
        clean_new_nodes = raw_new_nodes;
#ifdef USE_NOISE
        float sigma = 0.025;//good
        //float sigma = 0.01875;//1,0.75,0.5,0.1,0.05,0.025; for rope //0.00625, 0.00125, 0.01875, 0.025 for cloth

        //Box-Mueller
        for(int i =0; i < raw_new_nodes.size(); i++)
        {
            raw_new_nodes[i] = raw_new_nodes[i] + btVector3(box_muller(0,sigma),box_muller(0,sigma),box_muller(0,sigma));
        }

        //printf("%f\n",box_muller(0,sigma));
#endif

        if(bFirstTrackingIteration)
        {
            filtered_new_nodes = raw_new_nodes;
        }
        else
        {
            for(int i =0; i< raw_new_nodes.size(); i++)
            {

                filtered_new_nodes[i] = mu*(raw_new_nodes[i] - filtered_new_nodes[i]) + filtered_new_nodes[i];

            }
        }

        float true_error = 0;

#ifdef DO_COVERAGE
        std::vector<btVector3> rot_line_pnts;
        std::vector<btVector4> plot_cols;

        for(int i = 0; i < cover_points.size(); i++)
        {
            int closest_ind = -1;
            float closest_dist = 1000000;
            for(int j = 0; j < filtered_new_nodes.size(); j++)
            {
                float dist = (cover_points[i] - filtered_new_nodes[j]).length();
                if(dist < closest_dist)
                {
                    closest_dist = dist;
                    closest_ind = j;
                }
            }

            btVector3 targvec = cover_points[i] - filtered_new_nodes[closest_ind];
            error = error + targvec.length();

            ////test code
            int true_closest_ind = -1;
            float true_closest_dist = 1000000;
            for(int j = 0; j < clean_new_nodes.size(); j++)
            {
                float dist = (cover_points[i] - clean_new_nodes[j]).length();
                if(dist < true_closest_dist)
                {
                    true_closest_dist = dist;
                    true_closest_ind = j;
                }
            }
            true_error += (cover_points[i] - clean_new_nodes[true_closest_ind]).length();
            ////

            if(closest_dist < 0.2)
                continue;

//            for(int j = 0; j < filtered_new_nodes.size(); j++)
//            {
//                btVector3 targvec = cover_points[i] - filtered_new_nodes[j];
//                targvec = exp(-0.1*targvec.length())*targvec;
//                for(int k = 0; k < 3; k++)
//                    V_step(3*closest_ind + k) += targvec[k]; //accumulate targvecs
//            }


            for(int j = 0; j < 3; j++)
                V_step(3*closest_ind + j) += targvec[j]; //accumulate targvecs

            plotpoints.push_back(filtered_new_nodes[closest_ind]);
            //plotpoints.push_back(cover_points[i]);
            plotcols.push_back(btVector4(targvec.length(),0,0,1));

    #ifdef ROPE
            plotpoints.push_back(cover_points[i]);
            plotcols.push_back(btVector4(targvec.length()/2,0,0,1));
    #else
            rot_line_pnts.push_back(filtered_new_nodes[closest_ind]);
            rot_line_pnts.push_back(cover_points[i]);
            plot_cols.push_back(btVector4(0.8,0,0.8,1));
    #endif

        }
    #ifndef ROPE
        rot_lines->setPoints(rot_line_pnts,plot_cols);
    #endif

#else
        for( map<int,int>::iterator ii=node_mirror_map.begin(); ii!=node_mirror_map.end(); ++ii)
        {
            //btVector3 targpoint = point_reflector->reflect(clothptr->softBody->m_nodes[(*ii).second].m_x);
            btVector3 targpoint = point_reflector->reflect(filtered_new_nodes[(*ii).second]);

            //btVector3 targvec = targpoint - clothptr->softBody->m_nodes[(*ii).first].m_x;
            btVector3 targvec = targpoint - filtered_new_nodes[(*ii).first];
            error = error + targvec.length();
            for(int j = 0; j < 3; j++)
                V_step(3*(*ii).first + j) = targvec[j];

            //btVector3 node_delta = clothptr->softBody->m_nodes[(*ii).first].m_x - prev_node_pos[(*ii).first];
            btVector3 node_delta = filtered_new_nodes[(*ii).first] - prev_node_pos[(*ii).first];
            for(int j = 0; j < 3; j++)
                V_delta(3*(*ii).first + j) = node_delta[j]; // used for adaptive Jacobian


            //plotpoints.push_back(clothptr->softBody->m_nodes[(*ii).first].m_x);
            plotpoints.push_back(filtered_new_nodes[(*ii).first]);
            plotcols.push_back(btVector4(targvec.length(),0,0,1));


            node_change = node_change + node_delta.length();

        }
#endif

#ifdef PRESERVE_LENGTH
        // figure out why the tolerance and rope scale affect the result of 
        // manipulation
        // Second, try to turn on the collision detection for the gripper;
        // so far, no luck;

        float tolerance = 0.001;
        Eigen::MatrixXf new_distance_matrix;

        computeDeformableObjectDistanceMatrix(filtered_new_nodes,new_distance_matrix);

        Eigen::MatrixXf node_distance_difference = new_distance_matrix - deformableobject_distance_matrix;

        int ropeScale = 100;
        for(int i = 0; i < node_distance_difference.rows(); i++)
        {
            for(int j = i; j < node_distance_difference.cols(); j++)
            {
                if(node_distance_difference(i,j) - tolerance> 0)
                {
                    //printf("Distance exceeded between nodes %d and %d\n",i,j);
                    btVector3 targvec = 0.5*node_distance_difference(i,j)*(filtered_new_nodes[j] - filtered_new_nodes[i]);

                    for(int k = 0; k < 3; k++)
                        V_step(3*i + k) += ropeScale*targvec[k];

                    for(int k = 0; k < 3; k++)
                        V_step(3*j + k) += -ropeScale*targvec[k]; //accumulate targvecs
                }
            }
        }


#endif


        plot_points->setPoints(plotpoints,plotcols);

        //cout << "Error: " << error << " ";
        cout << error;
        cout << " " << true_error;

        //cout << "(Node Change " << node_change <<")";

#ifdef USE_ADAPTIVE_JACOBIAN
        if(bFirstTrackingIteration)
        {
            cout << " Normal Jacobian ";
            J = computeJacobian_approx();
        }
        else
        {
            cout << " Adaptive Jacobian ";

            nodeArrayToNodePosVector(clothptr->softBody->m_nodes, prev_node_pos);
            J = last_jacobian + 0.1*(V_delta - last_jacobian*last_movement)*last_movement.transpose()/(last_movement.transpose()*last_movement);
        }
#else

        J = computeJacobian_approx();
#endif

//        //low-pass filter
//        float mu2 = 0.05; //(0 = no change, 1.0 = no filtering)
//        if(!bFirstTrackingIteration)
//        {
//            V_step = mu2*(V_step - last_V_step) + last_V_step;
//        }




        //Eigen::MatrixXf Jt(J.transpose());
        //V_trans = Jt*V_step;

        Eigen::MatrixXf Jpinv_collision= pinv(Jcollision.transpose()*Jcollision)*Jcollision.transpose();
        Eigen::MatrixXf Jpinv= pinv(J.transpose()*J)*J.transpose();

#ifdef ROPE
        float k2 = 10;
#else
        float k2 = 100;
#endif
        //V_trans = Jpinv*V_step;
        Eigen::VectorXf q_desired = Jpinv*V_step;
        Eigen::VectorXf q_desired_nullspace = (Eigen::MatrixXf::Identity(Jcollision.cols(), Jcollision.cols())  - Jpinv_collision*Jcollision)*q_desired;
        Eigen::VectorXf q_collision = Jpinv_collision*V_step_collision;

        Eigen::VectorXf term1 = q_collision + q_desired_nullspace;
        Eigen::VectorXf term2 = q_desired;
        std::vector<float> vK(num_auto_grippers);


        for(int g = 0; g < num_auto_grippers; g++)
        {
            //cout << " dist" << g << ": " << vclosest_dist[g];
            vK[g] = exp(-k2*vclosest_dist[g]);
            if(vK[g] > 1)
                vK[g] = 1;
            //cout << "term1: " << term1.segment(g*dof_per_gripper,dof_per_gripper) << ", term2: " << term2.segment(g*dof_per_gripper,dof_per_gripper) << endl;
            //cout << " vK" << g << ": " << vK[g] <<" "<< dof_per_gripper << " " << term1.rows();
            term1.segment(g*dof_per_gripper,dof_per_gripper) = vK[g]*term1.segment(g*dof_per_gripper,dof_per_gripper);
            term2.segment(g*dof_per_gripper,dof_per_gripper) = (1 - vK[g])*term2.segment(g*dof_per_gripper,dof_per_gripper);
            //cout << "term1: " << term1.segment(g*dof_per_gripper,dof_per_gripper) << ", term2: " << term2.segment(g*dof_per_gripper,dof_per_gripper) << endl;
            
        }


        //Eigen::VectorXf term1 = K*(q_collision);// + q_desired_nullspace);
        //Eigen::VectorXf term2 = (1 - K)*q_desired;

        V_trans = term1 + term2;
        //V_trans = term2;
        //cout << " collision step: " << (Jpinv_collision*V_step_collision).norm() << " term1: " <<  term1.norm() << " desired: " << term2.norm();





        if(V_trans.norm() > step_limit)
            V_trans = V_trans/V_trans.norm()*step_limit;


        // if(!bAvoidObstacle) {
        //    //cout << "J: " << J.rows() << " " << J.cols() << endl;
        //    //cout << "V_step: " << V_step.rows() << " " << V_step.cols() << endl;
        //    //cout << "V_trans: " << V_trans.rows() << " " << V_trans.cols() << endl;
        //    V_trans = Jpinv*V_step;
        //    //cout << " (Command norm " << V_trans.norm()<<")";

        //    if(V_trans.norm() > step_limit)
        //        V_trans = V_trans/V_trans.norm()*step_limit;

        // }
        // else {
        //    Eigen::MatrixXf Jpinv_collision= pinv(Jcollision.transpose()*Jcollision)*Jcollision.transpose();
        //    //Eigen::VectorXf V_trans_collision = Jpinv_collision*V_step_collision;
        //    //if(V_trans_collision.norm() > step_limit)
        //    //    V_trans_collision = V_trans_collision/V_trans_collision.norm()*step_limit;

        //    V_trans = Jpinv_collision*V_step_collision + 0.1*(Eigen::MatrixXf::Identity(Jcollision.cols(), Jcollision.cols()) - Jpinv_collision*Jcollision)*Jpinv*V_step;

        //    if(V_trans.norm() > step_limit)
        //        V_trans = V_trans/V_trans.norm()*step_limit;


        //    //cout << endl << V_trans_collision.transpose() << endl;
        //    //V_trans = V_trans_collision;
        // }

//        //gaussian filter
//        float std_dev = 0.25;
//        float magsqr = V_trans.transpose()*V_trans;
//        V_trans = V_trans*exp(-magsqr/(2*std_dev*std_dev));
//        cout << " (Filtered command norm " << V_trans.norm()<<")";

//        //low-pass filter
//        float mu2 = .01; //(0 = no change, 1.0 = no filtering)
//        if(!bFirstTrackingIteration)
//        {
//            V_trans = mu2*(V_trans - last_movement) + last_movement;
//        }

        getDeformableObjectNodes(prev_node_pos);



        last_jacobian = J;
        last_movement = V_trans;
        bFirstTrackingIteration = false;


//        //dead band
//        if(abs(V_step.norm() - last_V_step.norm()) < 2)
//        {
//            last_V_step = V_step;
//            cout << endl;
//            break;
//        }
//        else
//        {
//            last_V_step = V_step;
//        }

        //cout << "trans vec: " << V_trans.transpose() << endl;
        //transvec = btVector3(V_trans(0),V_trans(1),V_trans(2));
#ifdef DO_ROTATION
    #ifdef USE_QUATERNION
        ///NOT IMPLEMENTED!
        btVector4 dquat1(V_trans(3),V_trans(4),V_trans(5),V_trans(6));
        btVector4 dquat2(V_trans(10),V_trans(11),V_trans(12),V_trans(13));
        dquat1 = dquat1*1/dquat1.length();
        dquat2 = dquat2*1/dquat2.length();

        transtm1 = btTransform(btQuaternion(dquat1),
                              btVector3(V_trans(0),V_trans(1),V_trans(2)));
        transtm2 = btTransform(btQuaternion(dquat2),
                                  btVector3(V_trans(7),V_trans(8),V_trans(9)));


    #else

        transtm1 = btTransform(btQuaternion(btVector3(0,0,1),V_trans(5))*
                              btQuaternion(btVector3(0,1,0),V_trans(4))*
                              btQuaternion(btVector3(1,0,0),V_trans(3)),
                              btVector3(V_trans(0),V_trans(1),V_trans(2)));

        if(num_auto_grippers > 1)
        {
            transtm2 = btTransform(btQuaternion(btVector3(0,0,1),V_trans(11))*
                                      btQuaternion(btVector3(0,1,0),V_trans(10))*
                                      btQuaternion(btVector3(1,0,0),V_trans(9)),
                                      btVector3(V_trans(6),V_trans(7),V_trans(8)));
        }
        else
            transtm2 = btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));

    #endif
#else
        transtm1 = btTransform(btQuaternion(0,0,0,1), btVector3(V_trans(0),V_trans(1),V_trans(2)));
        if(num_auto_grippers > 1)
            transtm2 = btTransform(btQuaternion(0,0,0,1), btVector3(V_trans(3),V_trans(4),V_trans(5)));
        else
            transtm2 = btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
#endif
        //check is it more semetric than it would have been had you done nothing
        //simulateInNewFork(innerstate, BulletConfig::dt, transvec);

        float errors[2];
//        omp_set_num_threads(2);
//        #pragma omp parallel
//        {
//            #pragma omp for nowait
//            for(int i = 0; i < 2 ; i++)
//            {
//                StepState innerstate;
//                //btTransform simtm(btQuaternion(0,0,0,1),transvec);
//                btTransform simtm1 = transtm1;
//                btTransform simtm2 = transtm2;
//                if(i == 1)
//                {
//                    simtm1 = btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
//                    simtm2 = btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
//                }
//                simulateInNewFork(innerstate, jacobian_sim_time, simtm1, simtm2);
//                errors[i] = 0;
//                for( map<int,int>::iterator ii=node_mirror_map.begin(); ii!=node_mirror_map.end(); ++ii)
//                {
//                    btVector3 targpoint = point_reflector->reflect(innerstate.cloth->softBody->m_nodes[(*ii).second].m_x);

//                    btVector3 targvec = targpoint - innerstate.cloth->softBody->m_nodes[(*ii).first].m_x;
//                    errors[i] = errors[i] + targvec.length();
//                }
//            }
//        }



        if(0 && errors[0] >= errors[1])
        {
            cout << "Error increase, not moving (new error: " <<  error << ")" << endl;
            //idleFor(0.2);
            //left_gripper1->translate(-transvec);
            //stepFor(BulletConfig::dt, 0.2);
            //bTracking = false;
            //stepFor(BulletConfig::dt, jacobian_sim_time);
            break;
        }
        else
        {
            cout << endl;
            left_gripper1->applyTransform(transtm1);
            left_gripper2->applyTransform(transtm2);

#ifdef USE_PR2
            btTransform left1(left_gripper1->getWorldTransform());
            btTransform TOR_newtrans = left1*TBullet_PR2GripperRight;
            TOR_newtrans.setOrigin(left1.getOrigin());
            pr2m.pr2Right->moveByIK(TOR_newtrans,SceneConfig::enableRobotCollision, true);

            btTransform left2(left_gripper2->getWorldTransform());
            TOR_newtrans = left2*TBullet_PR2GripperLeft;
            TOR_newtrans.setOrigin(left2.getOrigin());
            pr2m.pr2Left->moveByIK(TOR_newtrans,SceneConfig::enableRobotCollision, true);
#endif
            //stepFor(BulletConfig::dt, jacobian_sim_time);
        }
        break;

    }

    loopState.skip_step = false;
    bInTrackingLoop = false;

}


void CustomScene::regraspWithOneGripper(GripperKinematicObject::Ptr gripper_to_attach, GripperKinematicObject::Ptr  gripper_to_detach)
{
    gripper_to_attach->toggleattach(clothptr->softBody.get());
    gripper_to_detach->toggleattach(clothptr->softBody.get());
    gripper_to_detach->toggle();

    float apperture = gripper_to_attach->apperture;
    gripper_to_attach->apperture = 0.1;
    gripper_to_attach->toggle();
    gripper_to_attach->apperture = apperture;

    gripper_to_attach->toggleattach(clothptr->softBody.get());
    gripper_to_attach->toggle();

    //gripper_to_detach->setWorldTransform(btTransform(btQuaternion(0,0,0,1),btVector3(100,5,0)));
}

class CustomKeyHandler : public osgGA::GUIEventHandler {
    CustomScene &scene;
public:
    CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,osgGA::GUIActionAdapter & aa) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {


//        case 'f':
//            scene.createFork();
//            break;
//        case 'g':
//            {
//            scene.swapFork();
//            }
//            break;

//        case 'h':
//            scene.destroyFork();
//            break;


        //'1', '2', 'q', 'w' reservered for PR2!

        case '3':
            scene.inputState.transGrabber0 = true; break;
        case 'a':
            scene.inputState.rotateGrabber0 = true; break;

        case '4':
            scene.inputState.transGrabber1 = true; break;
        case 's':
            scene.inputState.rotateGrabber1 = true; break;

        case '5':
            scene.inputState.transGrabber2 = true; break;
        case 'e':
            scene.inputState.rotateGrabber2 = true; break;

        case '6':
            scene.inputState.transGrabber3 = true; break;
        case 'r':
            scene.inputState.rotateGrabber3 = true; break;

#ifdef USE_PR2
        case '9':
            scene.leftAction->reset();
            scene.leftAction->toggleAction();
            scene.runAction(scene.leftAction, BulletConfig::dt);

            break;
        case '0':

            scene.rightAction->reset();
            scene.rightAction->toggleAction();
            scene.runAction(scene.rightAction, BulletConfig::dt);

            break;
#endif
        case '[':
            scene.left_gripper1->toggle();
            scene.left_gripper1->toggleattach(scene.clothptr->softBody.get());
            if(scene.num_auto_grippers > 1)
            {
                scene.left_gripper2->toggle();
                scene.left_gripper2->toggleattach(scene.clothptr->softBody.get());
            }
            break;
//        case ']':
//            scene.corner_number++;
//            if(scene.corner_number > 3)
//                scene.corner_number = 0;

//            scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), scene.clothptr->softBody->m_nodes[scene.corner_grasp_point_inds[scene.corner_number]].m_x));

//            break;

        case ']':
            if(scene.num_auto_grippers > 1)
                scene.corner_number += 2;
            else
                scene.corner_number += 1;
            if(scene.corner_number > 3)
                scene.corner_number = 0;
            scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), scene.clothptr->softBody->m_nodes[scene.corner_grasp_point_inds[scene.corner_number]].m_x));
            if(scene.num_auto_grippers > 1)
                scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), scene.clothptr->softBody->m_nodes[scene.corner_grasp_point_inds[scene.corner_number+1]].m_x));
            break;


//        case 's':
//            scene.left_gripper2->toggle();
//            break;

//        case 'z':
//            scene.left_gripper1->toggleattach(scene.clothptr->softBody.get());
//            break;

//        case 'x':
//            scene.left_gripper2->toggleattach(scene.clothptr->softBody.get());
//            break;

        case 'c':
        {
            scene.regraspWithOneGripper(scene.left_gripper1,scene.left_gripper2);
            break;
        }

        case 'v':
        {
            scene.regraspWithOneGripper(scene.right_gripper1,scene.right_gripper2);
            break;
        }

        case 'f':
        {
            scene.regraspWithOneGripper(scene.right_gripper1,scene.left_gripper1);
            scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
            break;
        }

        case 'g':
        {
            scene.regraspWithOneGripper(scene.right_gripper2,scene.left_gripper2);
            scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,110)));
            break;
        }


        case 'k':
            scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*scene.right_gripper1->getWorldTransform().getRotation(), scene.right_gripper1->getWorldTransform().getOrigin()));
            break;

        case ',':
            scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*scene.right_gripper1->getWorldTransform().getRotation(), scene.right_gripper1->getWorldTransform().getOrigin()));
            break;


        case 'l':
            scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*scene.right_gripper2->getWorldTransform().getRotation(), scene.right_gripper2->getWorldTransform().getOrigin()));
            break;

        case '.':
            scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*scene.right_gripper2->getWorldTransform().getRotation(), scene.right_gripper2->getWorldTransform().getOrigin()));
            break;


        case 'y':
            scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*scene.right_gripper1->getWorldTransform().getRotation(), scene.right_gripper1->getWorldTransform().getOrigin()));
            break;


        case 'u':
            scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*scene.right_gripper2->getWorldTransform().getRotation(), scene.right_gripper2->getWorldTransform().getOrigin()));
            break;


        case 'i':
            scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), scene.clothptr->softBody->m_nodes[scene.robot_mid_point_ind].m_x));
            break;

        case 'o':
            scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), scene.clothptr->softBody->m_nodes[scene.user_mid_point_ind].m_x));
            break;


        case 'b':
            if(scene.right_gripper2->bOpen)
                scene.right_gripper2->state = GripperState_CLOSING;
            else
                scene.right_gripper2->state = GripperState_OPENING;

            break;

        case 'n':
            if(scene.left_gripper2->bOpen)
                scene.left_gripper2->state = GripperState_CLOSING;
            else
                scene.left_gripper2->state = GripperState_OPENING;

            break;

        case 'j':
            {
#ifdef PROFILER
                if(!scene.bTracking)
                    ProfilerStart("profile.txt");
                else
                    ProfilerStop();
#endif


               scene.getDeformableObjectNodes(scene.prev_node_pos);
               scene.bTracking = !scene.bTracking;
               if(scene.bTracking)
               {
                   scene.bFirstTrackingIteration = true;
                   scene.itrnumber = 0;
               }
               if(!scene.bTracking)
                   scene.plot_points->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());

                break;
            }

//        case 'b':
//            scene.stopLoop();
//            break;
        }
        break;

    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '3':
            scene.inputState.transGrabber0 = false; break;
            break;
        case 'a':
            scene.inputState.rotateGrabber0 = false; break;
        case '4':
            scene.inputState.transGrabber1 = false; break;
        case 's':
            scene.inputState.rotateGrabber1 = false; break;
        case '5':
            scene.inputState.transGrabber2 = false; break;
        case 'e':
            scene.inputState.rotateGrabber2 = false; break;
        case '6':
            scene.inputState.transGrabber3 = false; break;
        case 'r':
            scene.inputState.rotateGrabber3 = false; break;


        }
        break;

    case osgGA::GUIEventAdapter::PUSH:
        scene.inputState.startDragging = true;
        break;

    case osgGA::GUIEventAdapter::DRAG:
        if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG){
            // drag the active manipulator in the plane of view
            if ( (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
                  (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0 ||
                   scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1 ||
                   scene.inputState.transGrabber2 || scene.inputState.rotateGrabber2 ||
                   scene.inputState.transGrabber3 || scene.inputState.rotateGrabber3)) {
                if (scene.inputState.startDragging) {
                    scene.inputState.dx = scene.inputState.dy = 0;
                } else {
                    scene.inputState.dx = scene.inputState.lastX - ea.getXnormalized();
                    scene.inputState.dy = ea.getYnormalized() - scene.inputState.lastY;
                }
                scene.inputState.lastX = ea.getXnormalized(); scene.inputState.lastY = ea.getYnormalized();
                scene.inputState.startDragging = false;

                // get our current view
                osg::Vec3d osgCenter, osgEye, osgUp;
                scene.manip->getTransformation(osgCenter, osgEye, osgUp);
                btVector3 from(util::toBtVector(osgEye));
                btVector3 to(util::toBtVector(osgCenter));
                btVector3 up(util::toBtVector(osgUp)); up.normalize();

                // compute basis vectors for the plane of view
                // (the plane normal to the ray from the camera to the center of the scene)
                btVector3 normal = (to - from).normalized();
                btVector3 yVec = (up - (up.dot(normal))*normal).normalized(); //FIXME: is this necessary with osg?
                btVector3 xVec = normal.cross(yVec);
                btVector3 dragVec = SceneConfig::mouseDragScale*10 * (scene.inputState.dx*xVec + scene.inputState.dy*yVec);
                //printf("dx: %f dy: %f\n",scene.inputState.dx,scene.inputState.dy);

                btTransform origTrans;
                if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0)
                {
                    scene.left_gripper1->getWorldTransform(origTrans);
                }
                else if(scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1)
                {
                    scene.left_gripper2->getWorldTransform(origTrans);
                }
                else if(scene.inputState.transGrabber2 || scene.inputState.rotateGrabber2)
                {
                    scene.right_gripper1->getWorldTransform(origTrans);
                }
                else if(scene.inputState.transGrabber3 || scene.inputState.rotateGrabber3)
                {
                    scene.right_gripper2->getWorldTransform(origTrans);
                }

                //printf("origin: %f %f %f\n",origTrans.getOrigin()[0],origTrans.getOrigin()[1],origTrans.getOrigin()[2]);

                btTransform newTrans(origTrans);

                if (scene.inputState.transGrabber0 || scene.inputState.transGrabber1  ||
                        scene.inputState.transGrabber2  || scene.inputState.transGrabber3)
                    // if moving the manip, just set the origin appropriately
                    newTrans.setOrigin(dragVec + origTrans.getOrigin());
                else if (scene.inputState.rotateGrabber0 || scene.inputState.rotateGrabber1 ||
                         scene.inputState.rotateGrabber2 || scene.inputState.rotateGrabber3) {
                    // if we're rotating, the axis is perpendicular to the
                    // direction the mouse is dragging
                    btVector3 axis = normal.cross(dragVec);
                    btScalar angle = dragVec.length();
                    btQuaternion rot(axis, angle);
                    // we must ensure that we never get a bad rotation quaternion
                    // due to really small (effectively zero) mouse movements
                    // this is the easiest way to do this:
                    if (rot.length() > 0.99f && rot.length() < 1.01f)
                        newTrans.setRotation(rot * origTrans.getRotation());
                }
                //printf("newtrans: %f %f %f\n",newTrans.getOrigin()[0],newTrans.getOrigin()[1],newTrans.getOrigin()[2]);
                //softbody ->addForce(const btVector3& forceVector,int node)

//                std::vector<btVector3> plot_line;
//                std::vector<btVector4> plot_color;
//                plot_line.push_back(origTrans.getOrigin());
//                plot_line.push_back(origTrans.getOrigin() + 100*(newTrans.getOrigin()- origTrans.getOrigin()));
//                plot_color.push_back(btVector4(1,0,0,1));
//                scene.drag_line->setPoints(plot_line,plot_color);
                //btTransform TBullet_PR2Gripper = btTransform(btQuaternion(btVector3(0,1,0),3.14159265/2),btVector3(0,0,0));
                //btTransform TOR_newtrans = TBullet_PR2Gripper*newTrans;
                //TOR_newtrans.setOrigin(newTrans.getOrigin());
                if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0)
                {
                    scene.left_gripper1->setWorldTransform(newTrans);
#ifdef USE_PR2
                    btTransform TOR_newtrans = newTrans*TBullet_PR2GripperRight;
                    TOR_newtrans.setOrigin(newTrans.getOrigin());
                    scene.pr2m.pr2Right->moveByIK(TOR_newtrans,SceneConfig::enableRobotCollision, true);
#endif
                }
                else if(scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1)
                {
                    scene.left_gripper2->setWorldTransform(newTrans);
#ifdef USE_PR2
                    btTransform TOR_newtrans = newTrans*TBullet_PR2GripperLeft;
                    TOR_newtrans.setOrigin(newTrans.getOrigin());
                    scene.pr2m.pr2Left->moveByIK(TOR_newtrans,SceneConfig::enableRobotCollision, true);
#endif
                }
                else if(scene.inputState.transGrabber2 || scene.inputState.rotateGrabber2)
                {
                    scene.right_gripper1->setWorldTransform(newTrans);
                }
                else if(scene.inputState.transGrabber3 || scene.inputState.rotateGrabber3)
                {
                    scene.right_gripper2->setWorldTransform(newTrans);
                }
                return true;
            }
        }
        break;
    }
    return false;
}

BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, const btVector3 &center) {
    const int divs = 45;

    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        divs, divs,
        0, true);

    psb->m_cfg.piterations = 10;//2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS      ;//  | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    //pm->m_kLST = 0.2;//0.1; //makes it rubbery (handles self collisions better)
    psb->m_cfg.kDP = 0.05;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);
    //psb->generateClusters(500);

/*    for (int i = 0; i < psb->m_clusters.size(); ++i) {
        psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.1;
    }*/

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

void CustomScene::createFork() {
    if(fork)
    {
        destroyFork();
    }
    bullet2.reset(new BulletInstance);
    bullet2->setGravity(BulletConfig::gravity);
    osg2.reset(new OSGInstance);
    osg->root->addChild(osg2->root.get());

    fork.reset(new Fork(env, bullet2, osg2));
    registerFork(fork);

    //cout << "forked!" << endl;

#ifdef USE_PR2
    origRobot = pr2m.pr2;
    EnvironmentObject::Ptr p = fork->forkOf(pr2m.pr2);
    if (!p) {
        cout << "failed to get forked version of robot!" << endl;
        return;
    }
    tmpRobot = boost::static_pointer_cast<RaveRobotObject>(p);
    cout << (tmpRobot->getEnvironment() == env.get()) << endl;
    cout << (tmpRobot->getEnvironment() == fork->env.get()) << endl;
#endif
    left_gripper1_fork = boost::static_pointer_cast<GripperKinematicObject> (fork->forkOf(left_gripper1));
    right_gripper1_fork = boost::static_pointer_cast<GripperKinematicObject> (fork->forkOf(right_gripper1));
    clothptr_fork = boost::static_pointer_cast<BulletSoftObject> (fork->forkOf(clothptr));

}

void CustomScene::destroyFork() {
    if(left_gripper1.get() == left_gripper1_fork.get())
    {
        left_gripper1 = left_gripper1_orig;
        right_gripper1 = right_gripper1_orig;
        clothptr = clothptr_orig;
    }

    unregisterFork(fork);
    osg->root->removeChild(osg2->root.get());
    fork.reset();
    left_gripper1_fork.reset();
    right_gripper1_fork.reset();
    clothptr_fork.reset();
}

void CustomScene::swapFork() {
#ifdef USE_PR2
    // swaps the forked robot with the real one
    cout << "swapping!" << endl;
    int leftidx = pr2m.pr2Left->index;
    int rightidx = pr2m.pr2Right->index;
    origRobot.swap(tmpRobot);
    pr2m.pr2 = origRobot;
    pr2m.pr2Left = pr2m.pr2->getManipByIndex(leftidx);
    pr2m.pr2Right = pr2m.pr2->getManipByIndex(rightidx);
#endif
    if(left_gripper1.get() == left_gripper1_orig.get())
    {
        left_gripper1 = left_gripper1_fork;
        right_gripper1 = right_gripper1_fork;
        clothptr = clothptr_fork;
    }
    else
    {
        left_gripper1 = left_gripper1_orig;
        right_gripper1 = right_gripper1_orig;
        clothptr = clothptr_orig;
    }



/*    vector<int> indices; vector<dReal> vals;
    for (int i = 0; i < tmpRobot->robot->GetDOF(); ++i) {
        indices.push_back(i);
        vals.push_back(0);
    }
    tmpRobot->setDOFValues(indices, vals);*/
}


int getExtremalPoint(std::vector<btVector3> &pnt_vec, btMatrix3x3 projection_matrix, int first_dim_minmax, int second_dim_minmax, int third_dim_minmax)
{
    //first_dim_maxmin = 0 if min, first_dim_maxmin = 1 if max, first_dim_maxmin = -1 don't care
    btVector3 extremal_pnt = btVector3(pow(-1,first_dim_minmax)*1000,pow(-1,second_dim_minmax)*1000,pow(-1,third_dim_minmax)*1000);
    btVector3 projected_pnt;
    int extremal_ind = -1;
    bool bDimOK_1,bDimOK_2,bDimOK_3;
    for(int i = 0; i < pnt_vec.size();i++)
    {
        projected_pnt = projection_matrix * pnt_vec[i];

        bDimOK_1 = bDimOK_2 = bDimOK_3 = false;

        if(first_dim_minmax == -1)
            bDimOK_1 = true;
        if(second_dim_minmax == -1)
            bDimOK_2 = true;
        if(third_dim_minmax == -1)
            bDimOK_3 = true;

        if(projected_pnt[0] >= extremal_pnt[0] && first_dim_minmax == 1)
            bDimOK_1 = true;

        if(projected_pnt[0] <= extremal_pnt[0] && !first_dim_minmax)
            bDimOK_1 = true;

        if(projected_pnt[1] >= extremal_pnt[1] && second_dim_minmax == 1)
            bDimOK_2 = true;

        if(projected_pnt[1] <= extremal_pnt[1] && !second_dim_minmax )
            bDimOK_2 = true;

        if(projected_pnt[2] >= extremal_pnt[2] && third_dim_minmax == 1)
            bDimOK_3 = true;

        if(projected_pnt[2] <= extremal_pnt[2] && !third_dim_minmax)
            bDimOK_3 = true;


        if(bDimOK_1 && bDimOK_2 && bDimOK_3)
        {
            extremal_pnt = projected_pnt;
            extremal_ind = i;
        }
    }
    return extremal_ind;

}

void CustomScene::drawAxes()
{
    if(!!left_axes1)
        left_axes1->setup(left_gripper1->getWorldTransform(),1);
    if(!!left_axes2)
        left_axes2->setup(left_gripper2->getWorldTransform(),1);
}

void CustomScene::drawClosestPoints()
{


}



void GripperKinematicObject::rigidGrab(btRigidBody* prb, int objectnodeind, Environment::Ptr env_ptr)
{
    btTransform top_tm;
    children[0]->motionState->getWorldTransform(top_tm);

    cnt.reset(new btGeneric6DofConstraint(*(children[0]->rigidBody.get()),*prb,top_tm.inverse()*cur_tm,btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)),true));
    cnt->setLinearLowerLimit(btVector3(0,0,0));
    cnt->setLinearUpperLimit(btVector3(0,0,0));
    cnt->setAngularLowerLimit(btVector3(0,0,0));
    cnt->setAngularUpperLimit(btVector3(0,0,0));
    env_ptr->bullet->dynamicsWorld->addConstraint(cnt.get());

    vattached_node_inds.clear();
    vattached_node_inds.push_back(objectnodeind);

}

void CustomScene::makeRopeWorld()
{
    float rope_radius = .01;
    float segment_len = .025;
    const float table_height = .7;
    const float table_thickness = .05;

    // originall was 50 links
    int nLinks = 30;

    vector<btVector3> ctrlPts;
    for (int i=0; i< nLinks; i++) {
        //changed the initial position of the rope
        // originally, it was 0.5*segment_len*i, 0, table_height+5*rope_radius
      ctrlPts.push_back(METERS*btVector3(1.25,0.1-segment_len*i,table_height+5*rope_radius));
    }

    table = BoxObject::Ptr(new BoxObject(0,METERS*btVector3(.75,.75,table_thickness/2),
                btTransform(btQuaternion(0, 0, 0, 1), METERS*btVector3(1,0,table_height-table_thickness/2))));

    ropePtr.reset(new CapsuleRope(ctrlPts,.01*METERS));

    env->add(ropePtr);
    env->add(table);

    vector<BulletObject::Ptr> children =  ropePtr->getChildren();
    for (int j=0; j<children.size(); j++) {
      //children[j]->setColor(1,0,0,1);
        children[j]->setColor(0.15,0.65,0.15,1.0);
    }



#ifdef DO_COVERAGE

//    //translation offset
//    for(int i =0; i < children.size(); i++)
//    {
//        cover_points.push_back(children[i]->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(0,2,0));
//    }

//    //circle
//    for(float theta = 0; theta < 2*3.1415; theta += 0.1)
//    {
//        cover_points.push_back(table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(5*cos(theta),5*sin(theta)+5,table->halfExtents[2]+.01*METERS/2));

//    }

    //cylinder
    float radius = 3;
    float height = 6;
    num_auto_grippers = 2;


    // cylinder = CylinderStaticObject::Ptr(new CylinderStaticObject(0, radius, height, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(0,5,height/2))));
    // env->add(cylinder);
    // cylinder->setColor(179.0/255.0,176.0/255.0,160.0/255.0,1);


    // Part of codes that set the points of covering;

    // Stop the covering the old points;
    // for(float theta = 0; theta < 2*3.1415; theta += 0.3)
    // {
    //     for(float h = 0; h < height; h += 0.2)
    //         cover_points.push_back(cylinder->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(radius*cos(theta)+.01*METERS/2,radius*sin(theta)+.01*METERS/2,h-height/2));

    // }

    // First add additional points, without deleting old points;

    
    for (float pos = 3; pos <= 12; pos += 0.1) {
        cover_points.push_back(table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(5, pos, 7));
    }

    // end of creating the points to cover;
    cout << "num cover points " << cover_points.size() << endl;
    left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
    right_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
    right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));

    // Adding a second cylinder;
    // This cylinder serves as the base of the needle;

    // rename is cylinder?? 
    // anotherCylinder = CylinderStaticObject::Ptr(new CylinderStaticObject(0, 0.5, 5, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(5,5,2.5))));
    // env->add(anotherCylinder);
    
    // anotherCylinder->setColor(0.9,9.9,0.0,1.0);
    // End of adding the base of the needle

    // Creating a sequence of capsules, serve as the torus;
    // Used a diamond shape to approximate the shape of a torus;
    // Need to form this into a loop based on parameters;
    
    // Starting the formulation of the loop
    // 
    
    
    
    
    

    // numOfCapsules = torusRadius / torusStep + 1;
    // for (int i = 0; i < numOfCapsules; i++) {
    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX+i*torusStep,torusCenterY,torusCenterZ - torusRadius + i * torusStep)))));
    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX-i*torusStep,torusCenterY,torusCenterZ - torusRadius + i * torusStep)))));

    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX+i*torusStep,torusCenterY,torusCenterZ + torusRadius - i * torusStep)))));
    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX-i*torusStep,torusCenterY,torusCenterZ + torusRadius - i * torusStep)))));
    // }
    // torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX-torusRadius,torusCenterY,torusCenterZ)))));
    // torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX+torusRadius,torusCenterY,torusCenterZ)))));
    
    // // Make more thick walls
    // numOfCapsules += 3;
    // for (int i = 0; i < numOfCapsules; i++) {
    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX+i*torusStep,torusCenterY,torusCenterZ - torusRadius - torusStep + i * torusStep)))));
    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX-i*torusStep,torusCenterY,torusCenterZ - torusRadius - torusStep + i * torusStep)))));

    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX+i*torusStep,torusCenterY,torusCenterZ + torusRadius + torusStep - i * torusStep)))));
    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX-i*torusStep,torusCenterY,torusCenterZ + torusRadius + torusStep - i * torusStep)))));
    // }
    // torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX-torusRadius-torusStep,torusCenterY,torusCenterZ)))));
    // torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX+torusRadius+torusStep,torusCenterY,torusCenterZ)))));
    
    // numOfCapsules += 3;
    // for (int i = 0; i < numOfCapsules; i++) {
    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX+i*torusStep,torusCenterY,torusCenterZ - torusRadius - 2*torusStep + i * torusStep)))));
    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX-i*torusStep,torusCenterY,torusCenterZ - torusRadius - 2*torusStep + i * torusStep)))));

    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX+i*torusStep,torusCenterY,torusCenterZ + torusRadius + 2*torusStep - i * torusStep)))));
    //     torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX-i*torusStep,torusCenterY,torusCenterZ + torusRadius + 2*torusStep - i * torusStep)))));
    // }
    // torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX-torusRadius-2*torusStep,torusCenterY,torusCenterZ)))));
    // torus.push_back(CapsuleObject::Ptr(new CapsuleObject(0, 0.4, 0.4, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(torusCenterX+torusRadius+2*torusStep,torusCenterY,torusCenterZ)))));

    // for (int i = 0; i < torus.size(); i++) {
    //     torus[i]->setColor(0.9, 9.9, 0.0, 1.0);
    //     env->add(torus[i]);
    // }

    // start making a triangle mesh;
    // use the idea from http://www.ecse.rpi.edu/~wrf/wiki/ComputerGraphicsFall2012/guha/Code/torus.cpp

    int numOfColumn = 20;
    int numOfRow = 4; 
    torusRadius = 2;
    torusHeight = 0.5;
    

    // making a triangle mesh of the torus;
    // Idea is, to create 4 rows of points on torus;
    // one row on top, one row on bottom;

    // the other two is on the center;

    // make triangles from the top row with two rows on the center

    // make triangles from the bottom row with two rows on the center

    centerX = 5;
    centerY = 5;
    centerZ = 7;

    btTriangleMesh* mesh = new btTriangleMesh();
    vector<btVector3> row1;
    vector<btVector3> row2;
    vector<btVector3> row3;
    vector<btVector3> row4;
    double xc, yc, zc;
    for (int i = 0; i < numOfRow; i++) {
        for (int j = 0; j < numOfColumn; j++) {
            zc = ( torusRadius + 2* torusHeight * cos( (-1 + 2*(float)i/numOfRow) * M_PI ) ) * cos( (-1 + 2*(float)j/numOfColumn) * M_PI );
            xc = ( torusRadius + 2* torusHeight * cos( (-1 + 2*(float)i/numOfRow) * M_PI ) ) * sin( (-1 + 2*(float)j/numOfColumn) * M_PI );
            yc = torusHeight * sin( (-1 + 2*(float)i/numOfRow) * M_PI );
            
            if (i == 0) {
                //column 1, in the middle
                row1.push_back(btVector3(xc, yc, zc));
            }
            if (i == 1) {
                // column 2, in the middle
                row2.push_back(btVector3(xc, yc, zc));
            }
            if (i == 2) {
                // column 3, on top;
                row3.push_back(btVector3(xc, yc, zc));
            }
            if (i == 3) {
                // column 4, on bottom;
                row4.push_back(btVector3(xc, yc, zc));
            }
        }
    }

    // adding triangles between row3 and row1/row2
    for (int i = 0; i < row3.size()-1; i++) {
        mesh->addTriangle(row1[i], row2[i], row1[i+1], false);
        mesh->addTriangle(row2[i], row1[i+1], row2[i+1], false);

        mesh->addTriangle(row3[i], row2[i], row3[i+1], false);
        mesh->addTriangle(row2[i], row3[i+1], row2[i+1], false);
    }
    mesh->addTriangle(row3[row2.size()-1], row2[row2.size()-1], row3[0], false);
    mesh->addTriangle(row2[row2.size()-1], row3[0], row2[0], false);

    mesh->addTriangle(row1[row2.size()-1], row2[row2.size()-1], row1[0], false);
    mesh->addTriangle(row2[row2.size()-1], row1[0], row2[0], false);

    // adding triangles between row4 and row1/row2

    for (int i = 0; i < row3.size()-1; i++) {
        mesh->addTriangle(row1[i], row4[i], row1[i+1], false);
        mesh->addTriangle(row4[i], row1[i+1], row4[i+1], false);

        mesh->addTriangle(row3[i], row4[i], row3[i+1], false);
        mesh->addTriangle(row4[i], row3[i+1], row4[i+1], false);
    }
    mesh->addTriangle(row3[row3.size()-1], row4[row2.size()-1], row3[0], false);
    mesh->addTriangle(row4[row2.size()-1], row3[0], row4[0], false);

    mesh->addTriangle(row1[row2.size()-1], row4[row2.size()-1], row1[0], false);
    mesh->addTriangle(row4[row2.size()-1], row1[0], row4[0], false);

    



    btBvhTriangleMeshShape* shape=new btBvhTriangleMeshShape(mesh,true,true);
    BulletObject* test = new BulletObject(0, shape, btTransform(btQuaternion(0, 0, 0, 1), table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(centerX,centerY,centerZ)));
    o = BulletObject::Ptr(test);
    //o->collisionShape.reset(shape);
    o->setColor(0.9, 9.9, 0.0, 1.0);
    env->add(o);

    // end creating torus mesh


    

    // End of creating a torus;

#endif

    int gripper_closestobjectnodeind[num_auto_grippers];

    std::vector<btVector3> node_pos(ropePtr->getNodes());

    gripper_node_distance_map.resize(num_auto_grippers);
    GripperKinematicObject::Ptr cur_gripper;
    int childindex;
    for(int i = 0; i < num_auto_grippers; i++)
    {

        if(i == 0)
        {
            childindex = 0;
            gripper_closestobjectnodeind[i] = 0;
            
            cur_gripper = left_gripper1;
        }
        if(i == 1)
        {
            
            childindex = 7;
            gripper_closestobjectnodeind[i] = 7;
            
            cur_gripper = left_gripper2;
        }

        cur_gripper->setWorldTransform(ropePtr->children[childindex]->rigidBody->getCenterOfMassTransform());
        cur_gripper->rigidGrab(ropePtr->children[childindex]->rigidBody.get(),gripper_closestobjectnodeind[i],env);



        gripper_node_distance_map[i].resize(node_pos.size());

        for(int j = 0; j < node_pos.size(); j++)
        {
            gripper_node_distance_map[i][j] = (node_pos[gripper_closestobjectnodeind[i]]-node_pos[j]).length();
        }
    }

    computeDeformableObjectDistanceMatrix(node_pos,deformableobject_distance_matrix);

    cout << "rope node length " << (node_pos[0]-node_pos[node_pos.size()-1]).length() << endl;


    left_axes1.reset(new PlotAxes());
    env->add(left_axes1);

}

void CustomScene::computeDeformableObjectDistanceMatrix( const std::vector<btVector3>& node_pos, Eigen::MatrixXf& distance_matrix)
{
    distance_matrix = Eigen::MatrixXf( node_pos.size(), node_pos.size());
    for(int i = 0; i < node_pos.size(); i++)
    {
        for(int j = i; j < node_pos.size(); j++)
        {
            distance_matrix(i,j) = (node_pos[i]-node_pos[j]).length();
            distance_matrix(j,i) = distance_matrix(i,j);
        }
    }

}

void CustomScene::initializePloting()
{
    plot_points.reset(new PlotPoints(5));
    env->add(plot_points);

    rot_lines.reset(new PlotLines(2));
    rot_lines->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());
    env->add(rot_lines);


}

void CustomScene::makeClothWorld()
{
    const float table_height = .7;
#ifdef USE_TABLE
    const float table_thickness = .05;
    table = BoxObject::Ptr(
        new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
            btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.0, 0, table_height-table_thickness/2))));
    table->rigidBody->setFriction(1);
    env->add(table);
#endif

#ifdef DO_COVERAGE
    num_auto_grippers = 2;
    const float table_thickness = .05;
    //btTransform Tm_table(btQuaternion(0, 0, 0.3827,  0.9239), GeneralConfig::scale * btVector3(0.5-0.3, 0, table_height-table_thickness/2-0.1));
    btTransform Tm_table(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(0.5, 0, table_height-table_thickness/2));
    table = BoxObject::Ptr(new BoxObject(0, GeneralConfig::scale * btVector3(.2,.2,table_thickness/2),Tm_table));
    table->rigidBody->setFriction(1);
    //table->setColor(0.8,0.2,0.2,1.0);
    env->add(table);
    float stepsize = 0.25;
    for(float x = -table->halfExtents[0]; x < table->halfExtents[0]; x+=stepsize)
        for(float y = -table->halfExtents[1]; y < table->halfExtents[1]; y+=stepsize)
        {
            cover_points.push_back(Tm_table*btVector3(x,y,table->halfExtents[2]));
        }
     cout << "num cover points " << cover_points.size() << endl;
//    float radius = 3;
//    float height = 7;
//    cylinder = CylinderStaticObject::Ptr(new CylinderStaticObject(0, radius, height, btTransform(btQuaternion(0, 0, 0, 1), btVector3(4,0,6)+btVector3(0,0,height/2))));
//    cylinder->rigidBody->setFriction(1);
//    env->add(cylinder);
//    for(float theta = 0; theta < 2*3.1415; theta += 0.3)
//    {
//        for(float r = 0; r < radius; r += 0.3)
//            cover_points.push_back(cylinder->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(r*cos(theta)+.01*METERS/2,r*sin(theta)+.01*METERS/2,height/2));
//    }






    corner_number = 0;

#endif

//    float cylradius = 3;
//    float cylheight = 6;
//    cylinder = CylinderStaticObject::Ptr(new CylinderStaticObject(0, cylradius, cylheight, btTransform(btQuaternion(0, 0, 0, 1), btVector3(10,0,10+cylheight/2))));
//    env->add(cylinder);

    BulletSoftObject::Ptr cloth(
            createCloth(GeneralConfig::scale * 0.25, GeneralConfig::scale * btVector3(0.7, 0, table_height+0.01)));
    //printf("scale: \%f\n meters: %f\n",GeneralConfig::scale, METERS);

    btSoftBody* psb = cloth->softBody.get();
    clothptr = clothptr_orig = cloth;
    psb->setTotalMass(0.1);

    addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, this->right_gripper2,psb));
    addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, this->left_gripper2,psb));

    //table->setColor(0.8,0.2,0.2,1.0);
#ifdef USE_PR2
    pr2m.pr2->ignoreCollisionWith(psb);
    pr2m.pr2->ignoreCollisionWith(left_gripper1->getChildren()[0]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(left_gripper1->getChildren()[1]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(left_gripper2->getChildren()[0]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(left_gripper2->getChildren()[1]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(right_gripper1->getChildren()[0]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(right_gripper1->getChildren()[1]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(right_gripper2->getChildren()[0]->rigidBody.get());
    pr2m.pr2->ignoreCollisionWith(right_gripper2->getChildren()[1]->rigidBody.get());
#endif


    env->add(cloth);
    cloth->setColor(0.15,0.65,0.15,1.0);
    //left_mover.reset(new RigidMover(table, table->rigidBody->getCenterOfMassPosition(), env->bullet->dynamicsWorld));

#ifdef USE_PR2
    leftAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Left, "l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link", 1));
    leftAction->setTarget(psb);
    rightAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Right, "r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", 1));
    rightAction->setTarget(psb);
#endif


    //btVector3 pos(0,0,0);
    //grab_left.reset(new Grab(psb, &psb->m_nodes[0], left_grabber->rigidBody.get()));
    //grab_left.reset(new Grab(psb, 0, left_grabber->rigidBody.get()));

    //psb->m_cfg.kAHR = 1;
    //psb->appendAnchor(0,left_grabber->rigidBody.get());
    //psb->appendAnchor(1,left_grabber->rigidBody.get());
    //psb->appendAnchor(2,left_grabber->rigidBody.get());


    int min_x_ind = -1;
    int max_x_ind = -1;
    int min_y_ind = -1;
    int max_y_ind = -1;
    double min_x = 100;
    double max_x = -100;
    double min_y = 100;
    double max_y = -100;

    //std::vector<float> node_x(psb->m_nodes.size());
    //std::vector<float> node_y(psb->m_nodes.size());
    std::vector<btVector3> node_pos(psb->m_nodes.size());
    corner_ind = std::vector<int>(4,-1);
    std::vector<btVector3> corner_pnts(4);

    corner_pnts[0] = btVector3(100,100,0);
    corner_pnts[1] = btVector3(100,-100,0);
    corner_pnts[2] = btVector3(-100,100,0);
    corner_pnts[3] = btVector3(-100,-100,0);

    for(int i = 0; i < psb->m_nodes.size();i++)
    {
        //printf("%f\n", psb->m_nodes[i].m_x[0]);
//        double new_x = psb->m_nodes[i].m_x[0];
//        double new_y = psb->m_nodes[i].m_x[1];
        node_pos[i] = psb->m_nodes[i].m_x;

        if(node_pos[i][0] <= corner_pnts[0][0] && node_pos[i][1] <= corner_pnts[0][1])
        {
            corner_ind[0] = i;
            corner_pnts[0] = node_pos[i];
        }

        if(node_pos[i][0] <= corner_pnts[1][0] && node_pos[i][1] >= corner_pnts[1][1])
        {
            corner_ind[1] = i;
            corner_pnts[1] = node_pos[i];
        }

        if(node_pos[i][0] >= corner_pnts[2][0] && node_pos[i][1] <= corner_pnts[2][1])
        {
            corner_ind[2] = i;
            corner_pnts[2] = node_pos[i];
        }

        if(node_pos[i][0] >= corner_pnts[3][0] && node_pos[i][1] >= corner_pnts[3][1])
        {
            corner_ind[3] = i;
            corner_pnts[3] = node_pos[i];
        }

    }

    max_x = corner_pnts[3][0];
    max_y = corner_pnts[3][1];
    min_x = corner_pnts[0][0];
    min_y = corner_pnts[0][1];


    //btTransform tm_left = btTransform(btQuaternion( 0,    0.9877,    0.1564 ,   0), psb->m_nodes[min_x_ind].m_x+btVector3(0,0,2));
    //btTransform tm_right = btTransform(btQuaternion( 0,    0.9877,    0.1564 ,   0), psb->m_nodes[max_x_ind].m_x+btVector3(0,0,2));
    //left_grabber->motionState->setKinematicPos(tm_left);
    //right_grabber->motionState->setKinematicPos(tm_right);

    //psb->appendAnchor(min_x_ind,left_grabber->rigidBody.get());
    //psb->appendAnchor(max_x_ind,right_grabber->rigidBody.get());


    btTransform tm_left1 = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[0] + btVector3(left_gripper1->children[0]->halfExtents[0],left_gripper1->children[0]->halfExtents[1],0));
    left_gripper1->setWorldTransform(tm_left1);
    //left_gripper1->toggle();


    btTransform tm_right1 = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[2] + btVector3(-right_gripper1->children[0]->halfExtents[0],right_gripper1->children[0]->halfExtents[1],0));
    right_gripper1->setWorldTransform(tm_right1);

    btTransform tm_left2 = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[1] + btVector3(left_gripper2->children[0]->halfExtents[0],-left_gripper2->children[0]->halfExtents[1],0));
    left_gripper2->setWorldTransform(tm_left2);
    //left_gripper1->toggle();


    btTransform tm_right2 = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[3] + btVector3(-right_gripper2->children[0]->halfExtents[0],-right_gripper2->children[0]->halfExtents[1],0));
    right_gripper2->setWorldTransform(tm_right2);

    gripper_node_distance_map.resize(num_auto_grippers);

    for(int i = 0; i < num_auto_grippers; i++)
    {
        gripper_node_distance_map[i].resize(node_pos.size());

        for(int j = 0; j < node_pos.size(); j++)
        {
            gripper_node_distance_map[i][j] = (corner_pnts[i]-node_pos[j]).length();
        }
    }

#ifdef DO_COVERAGE
//    right_gripper1->setWorldTransform(tm_left2);
//    left_gripper2->setWorldTransform(tm_right1);

    corner_grasp_point_inds.resize(4);

    for(int i = 0; i < 4; i++)
    {

        btVector3 tm_origin;
        float closest_dist = 100000;
        corner_grasp_point_inds[i] = -1;
        if(i == 0)
            tm_origin = tm_left1.getOrigin();
        else if(i ==1 )
            tm_origin = tm_right1.getOrigin();
        else if(i ==2 )
            tm_origin = tm_left2.getOrigin();
        else
            tm_origin = tm_right2.getOrigin();

        for(int j = 0; j < node_pos.size(); j++)
        {
            float dist = (node_pos[j]-tm_origin).length();
            if(dist < closest_dist)
            {
                closest_dist = dist;
                corner_grasp_point_inds[i] = j;
            }
        }
    }
    //make an intuitive order
    int temp = corner_grasp_point_inds[1];
    corner_grasp_point_inds[1] = corner_grasp_point_inds[2];
    corner_grasp_point_inds[2] = temp;
#endif


    computeDeformableObjectDistanceMatrix(node_pos,deformableobject_distance_matrix);
    //cout << deformableobject_distance_matrix<< endl;

    //mirror about centerline along y direction;
    //centerline defined by 2 points
    float mid_x = (max_x + min_x)/2;

    point_reflector.reset(new PointReflector(mid_x, min_y, max_y));
    //find node that most closely matches reflection of point
    for(int i = 0; i < node_pos.size(); i++)
    {
        if(node_pos[i][0] < mid_x) //look at points in left half
        //if(node_pos[i][0] < mid_x && (abs(node_pos[i][0] - min_x) < 0.01 || abs(node_pos[i][1] - min_y) < 0.01 || abs(node_pos[i][1] - max_y) < 0.01))
        {
            //float reflected_x = node_pos[i][0] + 2*(mid_x - node_pos[i][0]);
            btVector3 new_vec = point_reflector->reflect(node_pos[i]);
            float closest_dist = 100000;
            int closest_ind = -1;
            for(int j = 0; j < node_pos.size(); j++)
            {
                float dist = (node_pos[j]-new_vec).length();//(node_pos[j][0]-reflected_x)*(node_pos[j][0]-reflected_x) + (node_pos[j][1]-node_pos[i][1])*(node_pos[j][1]-node_pos[i][1]);
                if(dist < closest_dist)
                {
                    closest_dist = dist;
                    closest_ind = j;
                }
            }
            node_mirror_map[i] = closest_ind;
        }


    }

    //get boundary points

    btVector3 user_target_mid_point(max_x,(max_y + min_y)/2,corner_pnts[0][2]);
    btVector3 robot_target_mid_point(min_x,(max_y + min_y)/2,corner_pnts[0][2]);
    btVector3 user_mid_point(100,100,100);
    btVector3 robot_mid_point(100,100,100);

    double user_length= 1000;
    double robot_length= 1000;
    for(int i = 0; i < node_pos.size(); i++)
    {
        double this_user_length = (node_pos[i]-user_target_mid_point).length();
        if( this_user_length < user_length)
        {
            user_length = this_user_length;
            user_mid_point = node_pos[i];
            user_mid_point_ind = i;
        }

        double this_robot_length = (node_pos[i]-robot_target_mid_point).length();
        if( this_robot_length < robot_length)
        {
            robot_length = this_robot_length;
            robot_mid_point = node_pos[i];
            robot_mid_point_ind = i;
        }
    }
//    cout << "robot side midpoint: " << robot_mid_point[0] << " " << robot_mid_point[1] << " " << robot_mid_point[2] << endl;
//    cout << "user side midpoint: " << user_mid_point[0] << " " << user_mid_point[1] << " " << user_mid_point[2] << endl;


//    tm_left1 = btTransform(btQuaternion( 0,    0,    0 ,   1), robot_mid_point);
//    left_gripper1->setWorldTransform(tm_left1);

//    tm_right2 = btTransform(btQuaternion( 0,    0,    0 ,   1), user_mid_point);
//    right_gripper2->setWorldTransform(tm_right2);


    //plotting

    std::vector<btVector3> plotpoints;
    std::vector<btVector4> plotcols;
    plotpoints.push_back(btVector3(mid_x,min_y,node_pos[0][2]));
    plotpoints.push_back(btVector3(mid_x,max_y,node_pos[0][2]));
    plotcols.push_back(btVector4(1,0,0,1));

//    for( map<int,int>::iterator ii=node_mirror_map.begin(); ii!=node_mirror_map.end(); ++ii)
//    {

//        cout << (*ii).first << ": " << (*ii).second << endl;
//        float r = (float)rand()/(float)RAND_MAX;
//        if(r < 0.5)
//        {
//            plotpoints.push_back(node_pos[(*ii).first]);
//            plotpoints.push_back(node_pos[(*ii).second]);
//            plotcols.push_back(btVector4(0,1,0,1));
//        }

//    }

#ifndef DO_COVERAGE
    PlotLines::Ptr lines;
    lines.reset(new PlotLines(2));
    lines->setPoints(plotpoints,plotcols);
    env->add(lines);
#endif

    left_center_point.reset(new PlotPoints(10));

    btTransform left_tm = left_gripper1->getWorldTransform();
    // cout << left_tm.getOrigin()[0] << " " << left_tm.getOrigin()[1] << " " << left_tm.getOrigin()[2] << " " <<endl;
    // cout << mid_x << " " << min_y << " " << node_pos[0][2] <<endl;
    std::vector<btVector3> poinsfsefts2;
    //points2.push_back(left_tm.getOrigin());
    //points2.push_back(left_tm.getOrigin());
    std::vector<btVector4> plotcols2;
    plotcols2.push_back(btVector4(1,0,0,1));
    //plotcols2.push_back(btVector4(1,0,0,1));

    poinsfsefts2.push_back(btVector3(mid_x,min_y,node_pos[0][2]));
    //poinsfsefts2[0] = left_tm.getOrigin();
    //plotcols.push_back(btVector4(1,0,0,1));

    std::vector<btVector3> plotpoints2;
    plotpoints2.push_back( left_tm.getOrigin());
    //plotpoints2.push_back(btVector3(mid_x,max_y,node_pos[0][2]));


    env->add(left_center_point);
    //left_center_point->setPoints(plotpoints2);


    left_axes1.reset(new PlotAxes());
    left_axes1->setup(tm_left1,1);
    env->add(left_axes1);

    left_axes2.reset(new PlotAxes());
    left_axes2->setup(tm_left2,1);
    env->add(left_axes2);

//    drag_line.reset(new PlotLines(2));
//    env->add(drag_line);

    left_gripper1->toggle();
    left_gripper1->toggleattach(clothptr->softBody.get());

    if(num_auto_grippers == 2)
    {
        left_gripper2->toggle();
        left_gripper2->toggleattach(clothptr->softBody.get());
    }
#ifndef DO_COVERAGE
    right_gripper1->toggle();
    right_gripper1->toggleattach(clothptr->softBody.get());

    right_gripper2->toggle();
    right_gripper2->toggleattach(clothptr->softBody.get());
#else
    right_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
    right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));

#endif

//    btMatrix3x3 proj_mat;
//    proj_mat[0][0] =   ; proj_mat[0][1] =  ;  proj_mat[0][2] = ;
//    proj_mat[1][0] =   ; proj_mat[1][1] =  ;  proj_mat[1][2] = ;
//    proj_mat[2][0] =   ; proj_mat[2][1] =  ;  proj_mat[2][2] = ;

//    for(int i = 0; i<cloth_boundary_inds.size();i++)
//    {

//    }

}

void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    addPreStepCallback(boost::bind(&CustomScene::doJTracking, this));
    addPreStepCallback(boost::bind(&CustomScene::drawAxes, this));
    addPreStepCallback(boost::bind(&CustomScene::drawClosestPoints, this));


    const float dt = BulletConfig::dt;


#ifdef ROPE
    makeRopeWorld();
#else
    makeClothWorld();
#endif

    initializePloting();


#ifdef DO_COVERAGE
    std::vector<btVector3> vnodes;
    getDeformableObjectNodes(vnodes);
    cout << "Num Deformable Object Points: " <<  vnodes.size() << endl << "Num Points to Cover: " << cover_points.size() << endl;
#endif

    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);

    /*
    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);
    */
    //ProfilerStart("profile.txt");
    startFixedTimestepLoop(dt);
    //ProfilerStop();
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);


    CustomScene().run();
    return 0;
}
