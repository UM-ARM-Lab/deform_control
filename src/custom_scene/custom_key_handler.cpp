#include "custom_key_handler.h"

#include "utils/util.h"

CustomKeyHandler::CustomKeyHandler(CustomScene &scene_)
    : scene(scene_)
{}

bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,osgGA::GUIActionAdapter & aa)
{
    (void)aa;

    switch (ea.getEventType())
    {
        case osgGA::GUIEventAdapter::KEYDOWN:
        {
            switch (ea.getKey())
            {
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

                case '[':
                {
                    scene.left_gripper1->toggle();
                    scene.left_gripper1->toggleattach(scene.clothPtr->softBody.get());
                    if(scene.num_auto_grippers > 1)
                    {
                        scene.left_gripper2->toggle();
                        scene.left_gripper2->toggleattach(scene.clothPtr->softBody.get());
                    }
                    break;
                }

                case ']':
                {
                    if(scene.num_auto_grippers > 1)
                        scene.corner_number_ += 2;
                    else
                        scene.corner_number_ += 1;
                    if(scene.corner_number_ > 3)
                        scene.corner_number_ = 0;
                    scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
                        scene.clothPtr->softBody->m_nodes[scene.corner_grasp_point_inds[scene.corner_number_]].m_x));
                    if(scene.num_auto_grippers > 1)
                        scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
                            scene.clothPtr->softBody->m_nodes[scene.corner_grasp_point_inds[scene.corner_number_+1]].m_x));
                    break;
                }

/*
                case 's':
                    scene.left_gripper2->toggle();
                    break;

                case 'z':
                    scene.left_gripper1->toggleattach(scene.clothPtr->softBody.get());
                    break;

                case 'x':
                    scene.left_gripper2->toggleattach(scene.clothPtr->softBody.get());
                    break;
*/

                case 'c':
                    scene.regraspWithOneGripper(scene.left_gripper1,scene.left_gripper2);
                    break;

                case 'v':
                    scene.regraspWithOneGripper(scene.right_gripper1,scene.right_gripper2);
                    break;

/*                case 'f':
                    // scene.regraspWithOneGripper(scene.right_gripper1,scene.left_gripper1);
                    // scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
                    scene.testRelease(scene.left_gripper1);
                    break;

                case 't':
                    scene.testRelease2(scene.left_gripper2);
                    break;

                case 'y':
                    scene.testRegrasp2(scene.left_gripper2);
                    break;

                case 'g':
                    // scene.regraspWithOneGripper(scene.right_gripper2,scene.left_gripper2);
                    // scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,110)));
                    scene.testRegrasp(scene.left_gripper1);
                    break;

                case 'k':
                    std::cout << "try to adjust first gripper" << std::endl;
                    scene.testAdjust(scene.left_gripper1);
                    break;

                case 'l':
                    // std::cout << "try to adjust second gripper" << std::endl;
                    scene.switchTarget();
                    break;
*/
                // case 'k':
                //     scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*
                //         scene.right_gripper1->getWorldTransform().getRotation(),
                //         scene.right_gripper1->getWorldTransform().getOrigin()));
                //     break;

                // case ',':
                //     scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*
                //         scene.right_gripper1->getWorldTransform().getRotation(),
                //         scene.right_gripper1->getWorldTransform().getOrigin()));
                //     break;


                // case 'l':
                //     scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*
                //         scene.right_gripper2->getWorldTransform().getRotation(),
                //         scene.right_gripper2->getWorldTransform().getOrigin()));
                //     break;

                case '.':
                    scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*
                        scene.right_gripper2->getWorldTransform().getRotation(),
                        scene.right_gripper2->getWorldTransform().getOrigin()));
                    break;


                // case 'y':
                //     scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*
                //         scene.right_gripper1->getWorldTransform().getRotation(),
                //         scene.right_gripper1->getWorldTransform().getOrigin()));
                //     break;


                case 'u':
                    scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*
                        scene.right_gripper2->getWorldTransform().getRotation(),
                        scene.right_gripper2->getWorldTransform().getOrigin()));
                    break;


                case 'i':
                    scene.left_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
                        scene.clothPtr->softBody->m_nodes[scene.robot_mid_point_ind].m_x));
                    break;

                case 'o':
                    scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),
                        scene.clothPtr->softBody->m_nodes[scene.user_mid_point_ind].m_x));
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
                    /// TODO: move this call to be inside of a control loop flagged
                    /// by bFirstTrackingIteration
                    scene.getDeformableObjectNodes(scene.prev_node_pos);
                    scene.bTracking = !scene.bTracking;
                    if(scene.bTracking)
                    {
                        scene.bFirstTrackingIteration = true;
                        scene.itrnumber = 0;
                    }
                    if(!scene.bTracking)
                    {
                        scene.plot_points->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());
                    }

                    break;
                }

            }
            break;
        }

        case osgGA::GUIEventAdapter::KEYUP:
        {
            switch (ea.getKey())
            {
                case '3':
                    scene.inputState.transGrabber0 = false; break;
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
        }

        case osgGA::GUIEventAdapter::PUSH:
        {
            scene.inputState.startDragging = true;
            break;
        }

        case osgGA::GUIEventAdapter::DRAG:
        {
            if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
            {
                // drag the active manipulator in the plane of view
                if ( (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
                      (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0 ||
                       scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1 ||
                       scene.inputState.transGrabber2 || scene.inputState.rotateGrabber2 ||
                       scene.inputState.transGrabber3 || scene.inputState.rotateGrabber3))
                {
                    if (scene.inputState.startDragging)
                    {
                        scene.inputState.dx = scene.inputState.dy = 0;
                    }
                    else
                    {
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
                    {
                        // if moving the manip, just set the origin appropriately
                        newTrans.setOrigin(dragVec + origTrans.getOrigin());
                    }
                    else if (scene.inputState.rotateGrabber0 || scene.inputState.rotateGrabber1 ||
                             scene.inputState.rotateGrabber2 || scene.inputState.rotateGrabber3)
                    {
                        // if we're rotating, the axis is perpendicular to the
                        // direction the mouse is dragging
                        btVector3 axis = normal.cross(dragVec);
                        btScalar angle = dragVec.length();
                        btQuaternion rot(axis, angle);
                        // we must ensure that we never get a bad rotation quaternion
                        // due to really small (effectively zero) mouse movements
                        // this is the easiest way to do this:
                        if (rot.length() > 0.99f && rot.length() < 1.01f)
                        {
                            newTrans.setRotation(rot * origTrans.getRotation());
                        }
                    }
/*
                    printf("newtrans: %f %f %f\n",newTrans.getOrigin()[0],newTrans.getOrigin()[1],newTrans.getOrigin()[2]);
                    softbody ->addForce(const btVector3& forceVector,int node)

                    std::vector<btVector3> plot_line;
                    std::vector<btVector4> plot_color;
                    plot_line.push_back(origTrans.getOrigin());
                    plot_line.push_back(origTrans.getOrigin() + 100*(newTrans.getOrigin()- origTrans.getOrigin()));
                    plot_color.push_back(btVector4(1,0,0,1));
                    scene.drag_line->setPoints(plot_line,plot_color);
                    btTransform TBullet_PR2Gripper = btTransform(btQuaternion(btVector3(0,1,0),3.14159265/2),btVector3(0,0,0));
                    btTransform TOR_newtrans = TBullet_PR2Gripper*newTrans;
                    TOR_newtrans.setOrigin(newTrans.getOrigin());
*/
                    if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0)
                    {
                        scene.left_gripper1->setWorldTransform(newTrans);
                    }
                    else if(scene.inputState.transGrabber1 || scene.inputState.rotateGrabber1)
                    {
                        scene.left_gripper2->setWorldTransform(newTrans);
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

        default:
        {
            break;
        }
    }
    return false;
}

