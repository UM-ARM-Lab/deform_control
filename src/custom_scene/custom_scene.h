#ifndef CUSTOM_SCENE_H
#define CUSTOM_SCENE_H

#include "tests/colab_cloth.h"

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/rope.h"

#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"

#include "gripper_kinematic_object.h"
#include "point_reflector.h"
#include "step_state.h"

class CustomScene : public Scene
{
    public:
        enum DeformableType
        {
            ROPE,
            CLOTH
        };

        enum TaskType
        {
            COVERAGE,
            COLAB_FOLDING
        };

        CustomScene(  DeformableType deformable_type, TaskType task_type  );
        void run(  bool syncTime = false  );

        /// These are all public for CustomKeyHandler
        struct
        {
            bool transGrabber0, rotateGrabber0,
                 transGrabber1, rotateGrabber1,
                 transGrabber2, rotateGrabber2,
                 transGrabber3, rotateGrabber3,
                 startDragging;
            float dx, dy, lastX, lastY;
        } inputState;

        GripperKinematicObject::Ptr left_gripper1, right_gripper1;
        GripperKinematicObject::Ptr left_gripper2, right_gripper2;

        void regraspWithOneGripper( GripperKinematicObject::Ptr gripper_to_attach, GripperKinematicObject::Ptr gripper_to_detach );

        /// Called by CustomKeyHandler when 'j' sets it to track
        void getDeformableObjectNodes( std::vector<btVector3>& vnodes );
        std::vector<btVector3> prev_node_pos;

        /// Cloth related variables
        /// again for CustomKeyHandler
        int num_auto_grippers; // currently used as a flag ( mostly )
        int corner_number_;
        BulletSoftObject::Ptr clothPtr;
        std::vector<int> corner_grasp_point_inds;
        int user_mid_point_ind, robot_mid_point_ind;

        PlotPoints::Ptr plot_points;

        /// these don't exit in the scene code at the moment
        /// they are getting moved to a separate source file
        /// Here to keep CustomKeyHandler happy
        bool bTracking, bFirstTrackingIteration;
        int itrnumber;

    private:
        boost::shared_ptr<CapsuleRope> ropePtr;

        vector<CapsuleObject::Ptr> torus;
        CylinderStaticObject::Ptr cylinder;
        BoxObject::Ptr table;

        DeformableType deformable_type_;
        TaskType task_type_;


        PointReflector::Ptr point_reflector;
        std::map<int, int> node_mirror_map;
        std::vector<std::vector<double> > gripper_node_distance_map;
        std::vector<btVector3> filtered_new_nodes;
        Eigen::VectorXf last_V_step;
        PlotPoints::Ptr left_center_point;
        PlotAxes::Ptr left_axes1,left_axes2;
        PlotLines::Ptr rot_lines;
        Eigen::MatrixXf deformableobject_distance_matrix;
        Eigen::MatrixXf last_jacobian;
        Eigen::VectorXf last_movement;

        std::vector<btVector3> cover_points;
        //Eigen::VectorXf last_clothstate;
        std::vector<int> corner_ind;
        std::vector<int> gripperPosition;
        //std::vector<int> cloth_boundary_inds;
        btBvhTriangleMeshShape* shape;

        double centerX, centerY, centerZ;
        double torusRadius, torusHeight;
        double gripperX;
        double gripperY;
        double gripperZ;

        double distanceGT;
        double distanceXZ;
        double px, py, pz;
        double anotherRadius;
        double A, B, C, alpha, beta;
        vector<bool> attached;


        // axis is six dimention;
        // first three numbers is the center: tip location;
        // last three numbers are the orientation of the axis;
        std::vector<double> BiotSavart( std::vector<double> point,
            std::vector<std::vector<double> > curve );

        std::vector<int> gripperStrategyNoneFix();
        std::vector<int> gripperStrategyFix();
        std::vector<int> gripperStrategyNoneFixPush();

        int trackPosition;
        bool random;
        int inTurn;
        int countSwitch;
        bool st;
        double variableScale;
        int distanceToTrack;

        void drawAxes();
        void computeDeformableObjectDistanceMatrix(  const std::vector<btVector3>& node_pos, Eigen::MatrixXf& distance_matrix );
        int getNumDeformableObjectNodes();

        BulletSoftObject::Ptr createCloth( btScalar half_side_length, const btVector3 &center );
        void makeRopeWorld();
        void makeClothWorld();

        void initializePloting();

};

#endif
