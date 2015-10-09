#ifndef COLAB_CLOTH_CUSTOM_SCENE_H
#define COLAB_CLOTH_CUSTOM_SCENE_H

#include "colab_cloth.h"

#include "gripper_kinematic_object.h"
#include "point_reflector.h"
#include "step_state.h"
#include "pr2_soft_body_gripper_action.h"

class ColabClothCustomScene : public Scene
{
    public:
#ifdef USE_PR2
        PR2SoftBodyGripperAction::Ptr leftAction, rightAction;
        PR2Manager pr2m;
#endif

        GripperKinematicObject::Ptr left_gripper1, right_gripper1, left_gripper1_orig, right_gripper1_orig, left_gripper1_fork, right_gripper1_fork;
        GripperKinematicObject::Ptr left_gripper2, right_gripper2;
        struct {
            bool transGrabber0,rotateGrabber0,transGrabber1,rotateGrabber1, transGrabber2,rotateGrabber2, transGrabber3,rotateGrabber3, startDragging;
            float dx, dy, lastX, lastY;
        } inputState;

        int num_auto_grippers;
        bool bTracking, bInTrackingLoop;
        PointReflector::Ptr point_reflector;
        BulletSoftObject::Ptr clothptr, clothptr_orig, clothptr_fork;
        BulletInstance::Ptr bullet2;
        OSGInstance::Ptr osg2;
        Fork::Ptr fork;
        RaveRobotObject::Ptr origRobot, tmpRobot;
        std::map<int, int> node_mirror_map;
        std::vector<std::vector<double> > gripper_node_distance_map;
        float jacobian_sim_time;
        std::vector<btVector3> prev_node_pos;
        std::vector<btVector3> filtered_new_nodes;
        Eigen::VectorXf last_V_step;
        PlotPoints::Ptr plot_points;
        PlotPoints::Ptr left_center_point;
        PlotAxes::Ptr left_axes1,left_axes2;
        PlotLines::Ptr rot_lines;
        Eigen::MatrixXf deformableobject_distance_matrix;
        int user_mid_point_ind, robot_mid_point_ind;
        Eigen::MatrixXf last_jacobian;
        Eigen::VectorXf last_movement;

        std::vector<btVector3> cover_points;
        //Eigen::VectorXf last_clothstate;
        bool bFirstTrackingIteration;
        int itrnumber;
        std::vector<int> corner_ind;
        std::vector<int> corner_grasp_point_inds;
        std::vector<int> gripperPosition;
        int corner_number;
        boost::shared_ptr<CapsuleRope> ropePtr;
        //std::vector<int> cloth_boundary_inds;
        btVoronoiSimplexSolver sGjkSimplexSolver;
    //    btGjkEpaPenetrationDepthSolver epaSolver;
    //    btPointCollector gjkOutput;
        CylinderStaticObject::Ptr cylinder;
        CylinderStaticObject::Ptr anotherCylinder;
        vector<CapsuleObject::Ptr> torus;
        BoxObject::Ptr table;
        btBvhTriangleMeshShape* shape;
        BulletObject::Ptr o;
        BulletObject::Ptr o1;
        BulletObject::Ptr o2;
        BulletObject::Ptr oT;
        BulletObject::Ptr oT1;
        BulletObject::Ptr oT2;
        BulletObject::Ptr oT3;
        BulletObject::Ptr oT4;
        BulletObject::Ptr oT5;
        BulletObject::Ptr oT6;
        BulletObject::Ptr oT7;
        BulletObject::Ptr oT8;
        BulletObject::Ptr oT9;
        BulletObject::Ptr oT10;
        BulletObject::Ptr oT11;

        BulletObject::Ptr oC;
        BulletObject::Ptr oC1;
        BulletObject::Ptr oC2;
        BulletObject::Ptr oC3;
        BulletObject::Ptr oC4;
        void makeBeltLoops();
        void makeCircuitLoops();
        void switchTarget();
        bool switched;

        double centerX, centerY, centerZ;
        double torusRadius, torusHeight;
        double gripperX;
        double gripperY;
        double gripperZ;

        double torusX;
        double torusY;
        double torusZ;

        double distanceGT;
        double distanceXZ;
        double pointOnTorusY, pointOnTorusZ, pointOnTorusX;
        double distanceToTorus;
        double px, py, pz;
        double anotherRadius;
        double A, B, C, alpha, beta;
        vector<bool> attached;


        // define the function to find the circle to use, and
        // find the vector to follow;
        // This vector is only for the tip of the rope;
        std::vector<double> findDirection (double x, double y, double z, bool tip);
        std::vector<double> findDirectionNotTip (double x, double y, double z,
                                        std::vector<double> axis);
        int findPointNotTip(std::vector<double> axis, double radius,
                                        std::vector<btVector3> points);
        // axis is six dimention;
        // first three numbers is the center: tip location;
        // last three numbers are the orientation of the axis;
        std::vector<double> BiotSavart(std::vector<double> point,
            std::vector<std::vector<double> > curve);
        std::vector<double> crossProduct (std::vector<double> u, std::vector<double> v);

        std::vector<std::vector<double> > findCircle(std::vector<double> normal,
                                                    std::vector<double> center);

        std::vector<std::vector<double> > findXCircle(std::vector<double> normal,
                                                    std::vector<double> center);
        std::vector<std::vector<double> > findYCircle(std::vector<double> normal,
                                                    std::vector<double> center);
        std::vector<std::vector<double> > findZCircle(std::vector<double> normal,
                                                    std::vector<double> center);
        std::vector<std::vector<double> > findXYCircle(std::vector<double> normal,
                                                    std::vector<double> center);
        std::vector<std::vector<double> > findXZCircle(std::vector<double> normal,
                                                    std::vector<double> center);
        std::vector<std::vector<double> > findYZCircle(std::vector<double> normal,
                                                    std::vector<double> center);


#ifdef USE_PR2
            ColabClothCustomScene() : pr2m(*this){
#else
            ColabClothCustomScene(){
#endif
            bTracking = bInTrackingLoop = false;
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

        void testRelease(GripperKinematicObject::Ptr  gripper_to_detach);
        void testRelease2(GripperKinematicObject::Ptr  gripper_to_detach);
        void testRegrasp(GripperKinematicObject::Ptr  gripper_to_detach);
        void testRegrasp2(GripperKinematicObject::Ptr  gripper_to_detach);
        void testAdjust(GripperKinematicObject::Ptr  gripper_to_detach);
        void Regrasp(GripperKinematicObject::Ptr  gripper, int place, int which);
        void Release(GripperKinematicObject::Ptr  gripper, int which);
        std::vector<int> gripperStrategyNoneFix();
        std::vector<int> gripperStrategyFix();
        std::vector<int> gripperStrategyNoneFixPush();
        Eigen::MatrixXf computeJacobian_approxTest(std::vector<int> locations);
        double gDNTCANIG(std::vector<int> locations, int input_ind, int &closest_ind);
        int trackPosition;
        bool random;
        int inTurn;
        int countSwitch;
        bool st;
        double variableScale;
        int distanceToTrack;


        void createFork();
        void destroyFork();
        void swapFork();

        Eigen::MatrixXf computeJacobian();
        Eigen::MatrixXf computeJacobian_parallel();
        Eigen::MatrixXf computeJacobian_approx();
        Eigen::MatrixXf computePointsOnGripperJacobian(std::vector<btVector3>& points_in_world_frame,std::vector<int>& autogripper_indices_per_point);

        double getDistfromNodeToClosestAttachedNodeInGripper(GripperKinematicObject::Ptr gripper, int input_ind, int &closest_ind);

        void simulateInNewFork(StepState& innerstate, float sim_time, btTransform& left_gripper1_tm, btTransform& left_gripper2_tm);
        void doJTracking();
        void drawAxes();
        void drawClosestPoints();
        void regraspWithOneGripper(GripperKinematicObject::Ptr gripper_to_attach, GripperKinematicObject::Ptr  gripper_to_detach);
        void computeDeformableObjectDistanceMatrix( const std::vector<btVector3>& node_pos, Eigen::MatrixXf& distance_matrix);
        void getDeformableObjectNodes(std::vector<btVector3>& vnodes);
        int getNumDeformableObjectNodes();

        BulletSoftObject::Ptr createCloth(btScalar half_side_length, const btVector3 &center);
        void makeRopeWorld();
        void makeClothWorld();

        void initializePloting();
        void run();
};

#endif
