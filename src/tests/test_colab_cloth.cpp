#include "colab_cloth.h"
#include "simulation/util.h"
#include <string>
#include <boost/bind.hpp>

#include "internal_utils.hpp"

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
            scene.left_gripper1->toggleOpen();
            scene.left_gripper1->toggleAttach(scene.clothptr->softBody.get());
            if(scene.num_auto_grippers > 1)
            {
                scene.left_gripper2->toggleOpen();
                scene.left_gripper2->toggleAttach(scene.clothptr->softBody.get());
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

    default:
        break;
    }
    return false;
}


#ifdef USE_PR2
CustomScene::CustomScene() : pr2m(*this)
{
#else
CustomScene::CustomScene()
{
#endif
    bTracking = false;
    bFirstTrackingIteration = true;
    itrnumber = 0;

    bInTrackingLoop = false;
    inputState.transGrabber0 =  inputState.rotateGrabber0 =
            inputState.transGrabber1 =  inputState.rotateGrabber1 =
            inputState.transGrabber2 =  inputState.rotateGrabber2 =
            inputState.transGrabber3 =  inputState.rotateGrabber3 =
            inputState.startDragging = false;

    jacobian_sim_time = 0.05;
    btVector4 color2(0,0,1,1);

    left_gripper1_orig.reset(new GripperKinematicObject(color2));
    left_gripper1_orig->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,-10,0)));
    env->add(left_gripper1_orig);

    btVector4 color(0.6,0.6,0.6,1);//(1,0,0,0.0);

    right_gripper1_orig.reset(new GripperKinematicObject(color));
    right_gripper1_orig->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,10,0)));
    env->add(right_gripper1_orig);

    left_gripper1 = left_gripper1_orig;
    right_gripper1 = right_gripper1_orig;


    left_gripper2.reset(new GripperKinematicObject(color2));
    left_gripper2->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,20,0)));
    env->add(left_gripper2);

    right_gripper2.reset(new GripperKinematicObject(color));
    right_gripper2->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,-20,0)));
    env->add(right_gripper2);

    num_auto_grippers = 2;

    fork.reset();
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

double CustomScene::getDistfromNodeToClosestAttachedNodeInGripper(GripperKinematicObject::Ptr gripper, int input_ind, int &closest_ind)
{
    double min_dist = 1000000;
    closest_ind = -1;
    for(size_t i = 0; i < gripper->vattached_node_inds.size(); i++)
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



Eigen::MatrixXd CustomScene::computePointsOnGripperJacobian(std::vector<btVector3>& points_in_world_frame,std::vector<int>& autogripper_indices_per_point)
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


    Eigen::MatrixXd J(points_in_world_frame.size()*3,num_Jcols_per_gripper*num_auto_grippers);
    GripperKinematicObject::Ptr gripper;
    Eigen::VectorXd  V_pos(points_in_world_frame.size()*3);

    for(int g = 0; g < num_auto_grippers; g++)
    {
        if(g == 0)
            gripper = left_gripper1;
        else if(g == 1)
            gripper = left_gripper2;

        btVector3 transvec;
        for(int i = 0; i < num_Jcols_per_gripper; i++)
        {
            for(size_t k = 0; k < points_in_world_frame.size(); k++)
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

                ///////////// These 3 are not being used right now //////////////////////////////////
                else if(i == 3)
                    transvec =  (gripper->getWorldTransform()*btVector4(1,0,0,0)).cross(
                                points_in_world_frame[k] - gripper->getWorldTransform().getOrigin());
                else if(i == 4)
                    transvec =  (gripper->getWorldTransform()*btVector4(0,1,0,0)).cross(
                                points_in_world_frame[k] - gripper->getWorldTransform().getOrigin());
                else if(i == 5)
                    transvec =  (gripper->getWorldTransform()*btVector4(0,0,1,0)).cross(
                                points_in_world_frame[k] - gripper->getWorldTransform().getOrigin());

                for(int j = 0; j < 3; j++)
                    V_pos(3*k + j) = transvec[j];
            }

            J.col(num_Jcols_per_gripper*g + i) = V_pos;
        }
    }
    return J;
}

Eigen::MatrixXd CustomScene::computeJacobian_approx()
{
#ifdef ROPE
    double dropoff_const = 0.5;//0.5;
#else
    double dropoff_const = 0.7;//0.7 for colab folding;
#endif
    int numnodes = getNumDeformableObjectNodes();

    std::vector<btTransform> perts;
    double step_length = 0.2;
    double rot_angle = 0.2;
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


    Eigen::MatrixXd J(numnodes*3,perts.size()*num_auto_grippers);
    GripperKinematicObject::Ptr gripper;

    std::vector<btVector3> rot_line_pnts;
    std::vector<btVector4> plot_cols;


//    Eigen::VectorXd  V_before(numnodes*3);
//    for(int k = 0; k < numnodes; k++)
//    {
//        for(int j = 0; j < 3; j++)
//            V_before(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
//    }

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
        for(size_t i = 0; i < perts.size(); i++)
        {
            Eigen::VectorXd  V_pos(numnodes*3);

//                    btTransform dummy_tm(btQuaternion(0,0,0,1),btVector3(0,0,0));
//                    StepState innerstate;

//                    if( i >= 3)
//                    {
//                        if(g == 0)
//                            simulateInNewFork(innerstate, jacobian_sim_time, perts[i],dummy_tm);
//                        else
//                            simulateInNewFork(innerstate, jacobian_sim_time, dummy_tm,perts[i]);

//                        Eigen::VectorXd  V_after(V_before);
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
                        btVector3 transvec =
                            ((gripper->getWorldTransform()*perts[i]).getOrigin() - gripper->getWorldTransform().getOrigin())
                            *exp(-dist*dropoff_const)/step_length;
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
//                        btTransform T0_attached = btTransform(btQuaternion(0,0,0,1),node_pos[closest_ind]);
                        btTransform T0_attached = btTransform(btQuaternion(0,0,0,1),node_pos[k]);
                        btTransform T0_center = gripper->getWorldTransform();
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



//this is getting called before the step loop
void CustomScene::doJTracking()
{
    std::cout << "simTime: " << simTime << std::endl;
//    std::cout << "left_gripper1:\t" << PrettyPrint( left_gripper1->getWorldTransform() ) << std::endl;
//    std::cout << "left_gripper2:\t" << PrettyPrint( left_gripper2->getWorldTransform() ) << std::endl;
//    std::cout << "right_gripper1:\t" << PrettyPrint( right_gripper1->getWorldTransform() ) << std::endl;
//    std::cout << "right_gripper2:\t" << PrettyPrint( right_gripper2->getWorldTransform() ) << std::endl;

    // hack to start tracking on start of program
    static bool first_loop = true;
    if (first_loop)
    {
        getDeformableObjectNodes(prev_node_pos);
        bTracking = !bTracking;
        if(bTracking)
        {
            bFirstTrackingIteration = true;
            itrnumber = 1;
        }
        if(!bTracking)
            plot_points->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());

        first_loop = false;
    }


    // let the rope settle between iterations
    itrnumber++;
    if(itrnumber != 4)
    {
        loopState.skip_step = false;
        return;
    }
    else
    {
        loopState.skip_step = true;
        itrnumber = 0;
    }

#ifdef DO_ROTATION
    int dof_per_gripper = 6;
#else
    int dof_per_gripper = 3;
#endif

    ///////////////
#ifdef ROPE
    BulletObject::Ptr obj = cylinder;
#else
    BulletObject::Ptr obj = table;
#endif
    GripperKinematicObject::Ptr gripper;
//////////////////

    if(bTracking)
    {
        int numnodes = getNumDeformableObjectNodes();

        float step_limit = 0.05;
        Eigen::VectorXd V_step = Eigen::VectorXd::Zero(numnodes*3);

        Eigen::VectorXd V_trans;
        btTransform transtm1,transtm2;
        Eigen::MatrixXd J;

        float error = 0;
        std::vector<btVector3> plotpoints;
        std::vector<btVector4> plotcols;

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
        for(size_t i = 0; i < raw_new_nodes.size(); i++)
        {
            raw_new_nodes[i] = raw_new_nodes[i] + btVector3(box_muller(0,sigma),box_muller(0,sigma),box_muller(0,sigma));
        }

        //printf("%f\n",box_muller(0,sigma));
#endif

        if(true || bFirstTrackingIteration)
        {
            filtered_new_nodes = raw_new_nodes;
        }
        else
        {
            for(size_t i = 0; i< raw_new_nodes.size(); i++)
            {
                filtered_new_nodes[i] = mu*(raw_new_nodes[i] - filtered_new_nodes[i]) + filtered_new_nodes[i];

            }
        }





#ifdef DO_COVERAGE
        std::vector<btVector3> rot_line_pnts;
        std::vector<btVector4> rot_line_cols;
        std::vector<btVector4> plot_cols;

//        int last_ind = -1;
        for(size_t i = 0; i < cover_points.size(); i++)
        {
            int closest_ind = -1;
            double closest_dist = 1000000000;
            for(size_t j = 0; j < filtered_new_nodes.size(); j++)
            {
                float dist = (cover_points[i] - filtered_new_nodes[j]).length();
                if(dist < closest_dist)
                {
                    closest_dist = dist;
                    closest_ind = j;
                }
            }

            btVector3 targvec = cover_points[i] - filtered_new_nodes[closest_ind];






//            if (last_ind != closest_ind)
//                std::cout << std::endl;
//            last_ind = closest_ind;

//            std::cout << "Cover ind: " << i << " Rope ind: " << closest_ind << " delta: " << PrettyPrint( targvec ) << std::endl;

//            if (closest_ind == 20)
//                std::cout << targvec.x() << "\t"
//                    << targvec.y() << "\t"
//                    << targvec.z() << std::endl;




            rot_line_pnts.push_back(cover_points[i]);
            rot_line_pnts.push_back(filtered_new_nodes[closest_ind]);
            rot_line_cols.push_back(btVector4(0.8,0,0.8,1));

            error = error + targvec.length();

            ////test code
            int true_closest_ind = -1;
            float true_closest_dist = 1000000;
            for(size_t j = 0; j < clean_new_nodes.size(); j++)
            {
                float dist = (cover_points[i] - clean_new_nodes[j]).length();
                if(dist < true_closest_dist)
                {
                    true_closest_dist = dist;
                    true_closest_ind = j;
                }
            }
            if (true_closest_ind != closest_ind)
                cout << "Difference found!!!!!!!!!!\n";
            ////

#ifdef ROPE
            if(closest_dist < 0.2)
                continue;
#endif

            for(int j = 0; j < 3; j++)
                V_step(3*closest_ind + j) += targvec[j]; //accumulate targvecs

            plotpoints.push_back(filtered_new_nodes[closest_ind]);
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
        rot_lines->setPoints(rot_line_pnts, rot_line_cols);
#else
        Eigen::VectorXf V_delta = Eigen::VectorXf::Zero(numnodes*3);
        float node_change = 0;
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
            plotpoints.push_back( targpoint );
            plotcols.push_back(btVector4(1,0,0,1));

            plotpoints.push_back(filtered_new_nodes[(*ii).first]);
            plotcols.push_back(btVector4(0,targvec.length(),0,1));

            node_change = node_change + node_delta.length();
        }
#endif

#ifdef PRESERVE_LENGTH
        float tolerance = 0.1;
        Eigen::MatrixXd new_distance_matrix;

        computeDeformableObjectDistanceMatrix(filtered_new_nodes,new_distance_matrix);

        Eigen::MatrixXd node_distance_difference = new_distance_matrix - deformableobject_distance_matrix;


        for(int i = 0; i < node_distance_difference.rows(); i++)
        {
            for(int j = i; j < node_distance_difference.cols(); j++)
            {
                if(node_distance_difference(i,j) - tolerance> 0)
                {
                    //printf("Distance exceeded between nodes %d and %d\n",i,j);
                    btVector3 targvec = 0.5*node_distance_difference(i,j)*(filtered_new_nodes[j] - filtered_new_nodes[i]);
                    for(int k = 0; k < 3; k++)
                        V_step(3*i + k) += targvec[k];

                    for(int k = 0; k < 3; k++)
                        V_step(3*j + k) += -targvec[k]; //accumulate targvecs
                }
            }
        }
#endif


        plot_points->setPoints(plotpoints,plotcols);

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


//        Eigen::MatrixXd Jpinv= pinv(J.transpose()*J)*J.transpose();

        Eigen::VectorXd q_desired =
                WeightedLeastSquaresSolver(J, V_step, Eigen::VectorXd::Ones(numnodes*3), 0.001, 0.1);
//                Jpinv*V_step;
#ifdef AVOID_COLLISION
        if(obj)
        {
            Eigen::MatrixXd Jcollision = Eigen::MatrixXd::Zero(num_auto_grippers * 3, num_auto_grippers*dof_per_gripper);
            Eigen::VectorXd V_step_collision =  Eigen::VectorXd::Zero(num_auto_grippers * 3);
#ifdef ROPE
            float k2 = 10;
#else
            float k2 = 100;
#endif

            std::vector<btVector3> plotpoints;
            std::vector<btVector4> plotcols;

            std::vector<float> vclosest_dist(num_auto_grippers);
            for(int g =0; g < num_auto_grippers; g++)
            {
                if(g == 0)
                    gripper = left_gripper1;
                else if(g == 1)
                    gripper = left_gripper2;

                vclosest_dist[g] = BT_LARGE_FLOAT;

                btGjkEpaPenetrationDepthSolver epaSolver;
                btPointCollector gjkOutput;

                btGjkPairDetector convexConvex(dynamic_cast<btBoxShape*> (gripper->getChildren()[0]->collisionShape.get()),dynamic_cast<btConvexShape*> (obj->collisionShape.get()),&sGjkSimplexSolver,&epaSolver);

                btGjkPairDetector::ClosestPointInput input;

                gripper->children[0]->motionState->getWorldTransform(input.m_transformA);
                obj->motionState->getWorldTransform(input.m_transformB);
                input.m_maximumDistanceSquared = BT_LARGE_FLOAT;
                gjkOutput.m_distance = BT_LARGE_FLOAT;
                convexConvex.getClosestPoints(input, gjkOutput, 0);


                if (gjkOutput.m_hasResult)
                {
                    //cout << "has result" << endl;
                    //printf("distance: %10.4f\n", gjkOutput.m_distance);

                    // endPt is the point on the gripper that is closest to the object
                    btVector3 endPt = gjkOutput.m_pointInWorld + gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;
                    // startPt is the point on the object that is closest to the gripper
                    btVector3 startPt = (input.m_transformB*input.m_transformB.inverse())(gjkOutput.m_pointInWorld);

//                    plotpoints.push_back(startPt);
//                    plotpoints.push_back(endPt);
//                    plotcols.push_back(btVector4(0,0,1,1));
//                    rot_lines->setPoints(plotpoints,plotcols);

                    std::vector<btVector3> jacpoints;
                    std::vector<int> jacpoint_grippers;
                    jacpoints.push_back(endPt);
                    jacpoint_grippers.push_back(g);
                    Jcollision.block(g*3,0,3,Jcollision.cols()) = computePointsOnGripperJacobian(jacpoints, jacpoint_grippers);

                    Eigen::VectorXd V_coll_step(3);
                    V_coll_step[0] = endPt[0] - startPt[0];
                    V_coll_step[1] = endPt[1] - startPt[1];
                    V_coll_step[2] = endPt[2] - startPt[2];
                    V_coll_step = V_coll_step/V_coll_step.norm();

                    plotcols.push_back(btVector4(1,0,0,1));

                    if(gjkOutput.m_distance < vclosest_dist[g])
                    {
                       vclosest_dist[g] = gjkOutput.m_distance;
                       if(vclosest_dist[g] < 0)
                           V_coll_step = -V_coll_step;
                    }
                    V_step_collision[g*3 + 0] = V_coll_step[0];
                    V_step_collision[g*3 + 1] = V_coll_step[1];
                    V_step_collision[g*3 + 2] = V_coll_step[2];
                }
            }

            Eigen::MatrixXd Jpinv_collision= pinv(Jcollision.transpose()*Jcollision)*Jcollision.transpose();

            // Move away from collision
            Eigen::VectorXd q_collision = Jpinv_collision*V_step_collision;
            // in the nullspace of the collision Jacobian, do as much desired as we can
            Eigen::VectorXd q_desired_nullspace =
                    (Eigen::MatrixXd::Identity(Jcollision.cols(), Jcollision.cols())  - Jpinv_collision*Jcollision)*q_desired;

            Eigen::VectorXd term1 = q_collision + q_desired_nullspace;
            Eigen::VectorXd term2 = q_desired;
            std::vector<float> vK(num_auto_grippers);

            for(int g = 0; g < num_auto_grippers; g++)
            {
                //cout << " dist" << g << ": " << vclosest_dist[g];
                vK[g] = exp(-k2*vclosest_dist[g]);
                if(vK[g] > 1)
                    vK[g] = 1;

                //cout << " vK" << g << ": " << vK[g] <<" "<< dof_per_gripper << " " << term1.rows();
                term1.segment(g*dof_per_gripper,dof_per_gripper) = vK[g]*term1.segment(g*dof_per_gripper,dof_per_gripper);

                term2.segment(g*dof_per_gripper,dof_per_gripper) = (1 - vK[g])*term2.segment(g*dof_per_gripper,dof_per_gripper);
                //cout << " " << term1.transpose();
            }
            V_trans = term1 + term2;
        }
        else
        {
            V_trans = q_desired;
            //cout << "No object found for collision avoidance!" << endl;
        }
#else
        V_trans = q_desired;
#endif

        if(V_trans.norm() > step_limit)
            V_trans = V_trans/V_trans.norm()*step_limit;



        // Now that we've moved, set our new prev_pos
        getDeformableObjectNodes(prev_node_pos);


        last_jacobian = J;
        last_movement = V_trans;
        bFirstTrackingIteration = false;


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
    }

    loopState.skip_step = false;
}




void CustomScene::regraspWithOneGripper(GripperKinematicObject::Ptr gripper_to_attach, GripperKinematicObject::Ptr  gripper_to_detach)
{
    gripper_to_attach->toggleAttach(clothptr->softBody.get());
    gripper_to_detach->toggleAttach(clothptr->softBody.get());
    gripper_to_detach->toggleOpen();

    float apperture = gripper_to_attach->apperture;
    gripper_to_attach->apperture = 0.1;
    gripper_to_attach->toggleOpen();
    gripper_to_attach->apperture = apperture;

    gripper_to_attach->toggleAttach(clothptr->softBody.get());
    gripper_to_attach->toggleOpen();

    //gripper_to_detach->setWorldTransform(btTransform(btQuaternion(0,0,0,1),btVector3(100,5,0)));
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

/*    for (size_t i = 0; i < psb->m_clusters.size(); ++i) {
        psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.1;
    }*/

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

void CustomScene::drawAxes()
{
    if(!!left_axes1)
        left_axes1->setup(left_gripper1->getWorldTransform(),1);
    if(!!left_axes2)
        left_axes2->setup(left_gripper2->getWorldTransform(),1);
}

void CustomScene::makeRopeWorld()
{
    float rope_radius = .01;
    float segment_len = .025;
    const float table_height = .7;
    const float table_thickness = .05;
    int nLinks = 50;

    table = BoxObject::Ptr(new BoxObject(0,METERS*btVector3(.75,.75,table_thickness/2),
                btTransform(btQuaternion(0, 0, 0, 1), METERS*btVector3(1,0,table_height-table_thickness/2))));

//    std::cout << "Table: " << PrettyPrint( table->rigidBody->getCenterOfMassTransform() ) << std::endl;

    vector<btVector3> ctrlPts;
    for (int i = 0; i< nLinks; i++) {
      ctrlPts.push_back(METERS*btVector3(0.5+segment_len*i,0,table_height+5*rope_radius));
//      cout << PrettyPrint( ctrlPts.back() ) << std::endl;
    }
    ropePtr.reset(new CapsuleRope(ctrlPts,.01*METERS));

    env->add(ropePtr);
    env->add(table);

    vector<BulletObject::Ptr> children =  ropePtr->getChildren();
    for (size_t i = 0; i<children.size(); i++) {
      //children[i]->setColor(1,0,0,1);
        children[i]->setColor(0.15,0.65,0.15,1.0);
    }



#ifdef DO_COVERAGE

//    //translation offset
//    for(size_t i = 0; i < children.size(); i++)
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
    cylinder = CylinderStaticObject::Ptr(new CylinderStaticObject(0, radius, height, btTransform(btQuaternion(0, 0, 0, 1),
                    table->rigidBody->getCenterOfMassTransform().getOrigin()+btVector3(0,5,height/2))));
    env->add(cylinder);
    cylinder->setColor(179.0/255.0,176.0/255.0,160.0/255.0,1);

    for(float theta = 0; theta < 2*3.1415; theta += 0.3)
    {
        for(float h = 0; h < height; h += 0.2)
        {
            cover_points.push_back(cylinder->rigidBody->getCenterOfMassTransform().getOrigin()+
                    btVector3((radius+.01*METERS/2)*cos(theta),(radius+.01*METERS/2)*sin(theta),h-height/2));
//            std::cout << PrettyPrint( cover_points.back() ) << std::endl;
        }

    }
    cout << "num cover points " << cover_points.size() << endl;
//    std::cout << "Cylinder: " << PrettyPrint( cylinder->rigidBody->getCenterOfMassTransform() ) << std::endl;

    // grippers
    num_auto_grippers = 1;
    left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
    right_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
    right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));


#endif

    int gripper_closestobjectnodeind[num_auto_grippers];

    std::vector<btVector3> node_pos(ropePtr->getNodes());

    // NOTE: This variable is never read from
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
            childindex = children.size()-1;
            gripper_closestobjectnodeind[i] = children.size()-1;
            cur_gripper = left_gripper2;
        }

        cur_gripper->setWorldTransform(ropePtr->children[childindex]->rigidBody->getCenterOfMassTransform());
        cur_gripper->rigidGrab(ropePtr->children[childindex]->rigidBody.get(),gripper_closestobjectnodeind[i],env);



        gripper_node_distance_map[i].resize(node_pos.size());

        for(size_t j = 0; j < node_pos.size(); j++)
        {
            gripper_node_distance_map[i][j] = (node_pos[gripper_closestobjectnodeind[i]]-node_pos[j]).length();
        }
    }

    computeDeformableObjectDistanceMatrix(node_pos,deformableobject_distance_matrix);

    cout << "rope node length " << (node_pos[0]-node_pos[node_pos.size()-1]).length() << endl;


    left_axes1.reset(new PlotAxes());
    env->add(left_axes1);

}

void CustomScene::computeDeformableObjectDistanceMatrix( const std::vector<btVector3>& node_pos, Eigen::MatrixXd& distance_matrix)
{
    distance_matrix = Eigen::MatrixXd( node_pos.size(), node_pos.size());
    for(size_t i = 0; i < node_pos.size(); i++)
    {
        for(size_t j = i; j < node_pos.size(); j++)
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
    btTransform Tm_table(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(0, 0, table_height-table_thickness/2));
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
            createCloth(GeneralConfig::scale * 0.25, GeneralConfig::scale * btVector3(0.25f, 0, table_height+0.01)));
    //printf("scale: \%f\n meters: %f\n",GeneralConfig::scale, METERS);

    btSoftBody* psb = cloth->softBody.get();
    clothptr = clothptr_orig = cloth;
    psb->setTotalMass(0.1);

    addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, this->right_gripper1,psb));
    addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, this->left_gripper1,psb));
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




    // NOTE: This variable is never read from
    gripper_node_distance_map.resize(num_auto_grippers);

    for(int i = 0; i < num_auto_grippers; i++)
    {
        gripper_node_distance_map[i].resize(node_pos.size());

        for(size_t j = 0; j < node_pos.size(); j++)
        {
            gripper_node_distance_map[i][j] = (corner_pnts[i]-node_pos[j]).length();
        }
    }

#ifdef DO_COVERAGE
    corner_grasp_point_inds.resize(4);

    for(size_t i = 0; i < 4; i++)
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

        for(size_t j = 0; j < node_pos.size(); j++)
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
    for(size_t i = 0; i < node_pos.size(); i++)
    {
        if(node_pos[i][0] < mid_x) //look at points in left half
        //if(node_pos[i][0] < mid_x && (abs(node_pos[i][0] - min_x) < 0.01 || abs(node_pos[i][1] - min_y) < 0.01 || abs(node_pos[i][1] - max_y) < 0.01))
        {
            //float reflected_x = node_pos[i][0] + 2*(mid_x - node_pos[i][0]);
            btVector3 new_vec = point_reflector->reflect(node_pos[i]);
            float closest_dist = 100000;
            int closest_ind = -1;
            for(size_t j = 0; j < node_pos.size(); j++)
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
    for(size_t i = 0; i < node_pos.size(); i++)
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
    cout << left_tm.getOrigin()[0] << " " << left_tm.getOrigin()[1] << " " << left_tm.getOrigin()[2] << " " <<endl;
    cout << mid_x << " " << min_y << " " << node_pos[0][2] <<endl;
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

    left_gripper1->toggleOpen();
    left_gripper1->toggleAttach(clothptr->softBody.get());

    if(num_auto_grippers == 2)
    {
        left_gripper2->toggleOpen();
        left_gripper2->toggleAttach(clothptr->softBody.get());
    }
#ifndef DO_COVERAGE
    right_gripper1->toggleOpen();
    right_gripper1->toggleAttach(clothptr->softBody.get());
    manual_grippers_paths_.push_back( smmap::ManualGripperPath( right_gripper1, &smmap::gripperPath0 ) );

    right_gripper2->toggleOpen();
    right_gripper2->toggleAttach(clothptr->softBody.get());
    manual_grippers_paths_.push_back( smmap::ManualGripperPath( right_gripper2, &smmap::gripperPath1 ) );
#else
    right_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
    right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));

#endif

//    btMatrix3x3 proj_mat;
//    proj_mat[0][0] =   ; proj_mat[0][1] =  ;  proj_mat[0][2] = ;
//    proj_mat[1][0] =   ; proj_mat[1][1] =  ;  proj_mat[1][2] = ;
//    proj_mat[2][0] =   ; proj_mat[2][1] =  ;  proj_mat[2][2] = ;

//    for(size_t i = 0; i<cloth_boundary_inds.size();i++)
//    {

//    }

}

void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    addPreStepCallback(boost::bind(&CustomScene::drawAxes, this));

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

    setSyncTime(false);
    startViewer();
    stepFor(BulletConfig::dt, 2);


    addPreStepCallback(boost::bind(&CustomScene::doJTracking, this));

    for( auto& path: manual_grippers_paths_ )
    {
        addPreStepCallback(boost::bind(&smmap::ManualGripperPath::advanceGripper, path));
    }

    /*
    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);
    */
    //ProfilerStart("profile.txt");
    startFixedTimestepLoop(BulletConfig::dt);
    //ProfilerStop();
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(9, 0, 42);
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
