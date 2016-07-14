#ifndef CUSTOM_SCENE_H
#define CUSTOM_SCENE_H

#include <atomic>
#include <mutex>

#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/rope.h"

#include "simulation/config_bullet.h"
#include "simulation/config_scene.h"
#include "simulation/config_viewer.h"

#include "gripper_kinematic_object.h"
#include "manual_gripper_path.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <arc_utilities/dijkstras.hpp>
#include <smmap_experiment_params/task_enums.h>
#include <smmap_msgs/messages.h>

struct SimForkResult
{
    public:
        BulletInstance::Ptr bullet_;
        OSGInstance::Ptr osg_;
        Fork::Ptr fork_;
        BulletSoftObject::Ptr cloth_;
        boost::shared_ptr<CapsuleRope> rope_;
        std::map<std::string, GripperKinematicObject::Ptr> grippers_;
};

struct ViewerData
{
    public:
        ViewerData(const Scene& scene, const SimForkResult& fork_result)
            : scene_(scene)
            , fork_result_(fork_result)
        {}

        boost::shared_ptr<osgbCollision::GLDebugDrawer> dbgDraw_;
        osgViewer::Viewer viewer_;
        osg::ref_ptr<EventHandler> manip_;

        const Scene& scene_;
        const SimForkResult& fork_result_;

    void draw()
    {
        if (!scene_.drawingOn)
        {
            return;
        }
        if (scene_.loopState.debugDraw)
        {
            // This call was moved to the start of the step() function as part of the bullet collision pipeline involves drawing things when debug draw is enabled
            if (!dbgDraw_->getActive())
            {
                dbgDraw_->BeginDraw();
            }
            fork_result_.bullet_->dynamicsWorld->debugDrawWorld();
            dbgDraw_->EndDraw();
        }
        viewer_.frame();
    }
};


class CustomScene : public Scene
{
    public:
        CustomScene(ros::NodeHandle& nh, smmap::DeformableType deformable_type, smmap::TaskType task_type);

        ////////////////////////////////////////////////////////////////////////
        // Main function that makes things happen
        ////////////////////////////////////////////////////////////////////////

        void run(bool drawScene = false, bool syncTime = false);

    private:
        /// Protects against multiple threads accessing data that modifies the
        /// environment/simulation at the same time
        std::mutex sim_mutex_;

        ////////////////////////////////////////////////////////////////////////
        // Construction helper functions
        ////////////////////////////////////////////////////////////////////////

        void makeTable();
        void makeCylinder();
        void makeRope();
        void makeCloth();

        void makeRopeWorld();
        void makeClothWorld();
        void findClothCornerNodes();

        int64_t xyzIndexToGridIndex(const int64_t x_ind, const int64_t y_ind, const int64_t z_ind) const;
        int64_t worldPosToGridIndex(const double x, const double y, const double z) const;
        int64_t worldPosToGridIndex(const btVector3& vec) const;
        void createEdgesToNeighbours(const int64_t x_starting_ind, const int64_t y_starting_ind, const int64_t z_starting_ind);
        void createFreeSpaceGraph();

        ////////////////////////////////////////////////////////////////////////
        // Main loop helper functions
        ////////////////////////////////////////////////////////////////////////


//        SimForkResult main_simulator_state_;
//        std::shared_ptr<ViewerData> main_simulator_viewer_;


        std::shared_ptr<ViewerData> createVisualizerForFork(SimForkResult& fork_result);


        SimForkResult createForkWithNoSimulationDone(
                const Environment::Ptr environment_to_fork,
                BulletSoftObject::Ptr cloth_to_fork,
                boost::shared_ptr<CapsuleRope> rope_to_fork,
                std::map<std::string, GripperKinematicObject::Ptr> grippers_to_fork);


        SimForkResult createForkWithNoSimulationDone();
//        SimForkResult createForkWithNoSimulationDone(const SimForkResult& sim_to_fork);
        SimForkResult createForkWithNoSimulationDone(const std::vector<std::string>& gripper_names, const std::vector<geometry_msgs::Pose>& gripper_poses);
//        SimForkResult createForkWithNoSimulationDone(const SimForkResult& sim_to_fork, const std::vector<std::string>& gripper_names, const std::vector<geometry_msgs::Pose>& gripper_poses);


        SimForkResult simulateInNewFork(const std::vector<std::string>& gripper_names, const std::vector<geometry_msgs::Pose>& gripper_poses);
//        SimForkResult simulateInNewFork(const SimForkResult& sim_to_fork, const std::vector<std::string>& gripper_names, const std::vector<geometry_msgs::Pose>& gripper_poses);





        static void ApplyTransform(
                const std::map<std::string, GripperKinematicObject::Ptr>& grippers_map,
                const std::string& name,
                const geometry_msgs::Pose& pose);
        void moveGrippers();
//        void moveGrippers(const std::map<std::string, GripperKinematicObject::Ptr>& grippers_map);
        smmap_msgs::SimulatorFeedback createSimulatorFbk() const;
        smmap_msgs::SimulatorFeedback createSimulatorFbk(const SimForkResult& result) const;

        ////////////////////////////////////////////////////////////////////////
        // Internal helper functions
        ////////////////////////////////////////////////////////////////////////

        std::vector<btVector3> getDeformableObjectNodes() const;
        std::vector<btVector3> getDeformableObjectNodes(const SimForkResult& result) const;
        btPointCollector collisionHelper(const GripperKinematicObject::Ptr& gripper) const;
        btPointCollector collisionHelper(const SphereObject::Ptr& sphere) const;

        ////////////////////////////////////////////////////////////////////////
        // ROS Callbacks
        ////////////////////////////////////////////////////////////////////////

        bool getGripperNamesCallback(
                smmap_msgs::GetGripperNames::Request& req,
                smmap_msgs::GetGripperNames::Response& res);
        bool getGripperAttachedNodeIndicesCallback(
                smmap_msgs::GetGripperAttachedNodeIndices::Request& req,
                smmap_msgs::GetGripperAttachedNodeIndices::Response& res);
        bool getGripperPoseCallback(
                smmap_msgs::GetGripperPose::Request& req,
                smmap_msgs::GetGripperPose::Response& res);
        bool gripperCollisionCheckCallback(
                smmap_msgs::GetGripperCollisionReport::Request& req,
                smmap_msgs::GetGripperCollisionReport::Response& res);
        bool getCoverPointsCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res);
        bool getMirrorLineCallback(
                smmap_msgs::GetMirrorLine::Request& req,
                smmap_msgs::GetMirrorLine::Response& res);
        bool getFreeSpaceGraphCallback(
                smmap_msgs::GetFreeSpaceGraphRequest& req,
                smmap_msgs::GetFreeSpaceGraphResponse& res);
        bool getObjectInitialConfigurationCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res);
        bool getObjectCurrentConfigurationCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res);




//        bool executeGripperMovementAndUpdateSimCallback(
//                smmap_msgs::ExecuteGripperMovement::Request& req,
//                smmap_msgs::ExecuteGripperMovement::Response& res);

        void testGripperPosesExecuteCallback(
                const smmap_msgs::TestGrippersPosesGoalConstPtr& goal);




        void visualizationMarkerCallback(visualization_msgs::Marker marker);
        void visualizationMarkerArrayCallback(visualization_msgs::MarkerArray marker_array);

        ////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////

        // Our internal version of ros::spin()
        static void spin(const double loop_rate);

        ////////////////////////////////////////////////////////////////////////
        // Pre-step Callbacks
        ////////////////////////////////////////////////////////////////////////

        void drawAxes();

        PlotPoints::Ptr plot_points_;
        PlotLines::Ptr plot_lines_;

        std::map<std::string, PlotLines::Ptr> visualization_line_markers_;
        std::map<std::string, PlotPoints::Ptr> visualization_point_markers_;
        std::map<std::string, PlotSpheres::Ptr> visualization_sphere_markers_;

        ////////////////////////////////////////////////////////////////////////
        // Post-step Callbacks
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Task Variables
        // TODO to be moved into a CustomSceneConfig file?
        ////////////////////////////////////////////////////////////////////////

        smmap::DeformableType deformable_type_;
        smmap::TaskType task_type_;

        ////////////////////////////////////////////////////////////////////////
        // Grippers
        ////////////////////////////////////////////////////////////////////////

        std::atomic<bool> advance_grippers_;

        GripperKinematicObject::Ptr collision_check_gripper_;

        std::map<std::string, PlotAxes::Ptr> gripper_axes_;
        std::map<std::string, GripperKinematicObject::Ptr> grippers_;
        std::vector<std::string> auto_grippers_;
        std::vector<std::string> manual_grippers_;

        std::vector<smmap::ManualGripperPath> manual_grippers_paths_;

        ////////////////////////////////////////////////////////////////////////
        // Shared world objects
        ////////////////////////////////////////////////////////////////////////

        std::map<std::string, BulletObject::Ptr> world_objects_;

        const double world_x_min_;
        const double world_x_step_;
        const int64_t world_x_num_steps_;

        const double world_y_min_;
        const double world_y_step_;
        const int64_t world_y_num_steps_;

        const double world_z_min_;
        const double world_z_step_;
        const int64_t world_z_num_steps_;
        arc_dijkstras::Graph<btVector3> free_space_graph_;
        size_t num_graph_edges_;
        std::vector<int64_t> cover_ind_to_free_space_graph_ind_;
        std::vector<PlotAxes::Ptr> graph_corners_;

        ////////////////////////////////////////////////////////////////////////
        // Rope world objects
        ////////////////////////////////////////////////////////////////////////

        boost::shared_ptr<CapsuleRope> rope_;

        ////////////////////////////////////////////////////////////////////////
        // Cloth world objects
        ////////////////////////////////////////////////////////////////////////

        BulletSoftObject::Ptr cloth_;
        std::vector<int> cloth_corner_node_indices_;

        ////////////////////////////////////////////////////////////////////////
        // Task specific objects
        ////////////////////////////////////////////////////////////////////////

        std::vector<btVector3> cover_points_;
        smmap_msgs::GetMirrorLine::Response mirror_line_data_;

        ////////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////////

        ros::NodeHandle nh_;

        ros::Publisher simulator_fbk_pub_;
        const double feedback_covariance_;

        ros::Subscriber visualization_marker_sub_;
        ros::Subscriber visualization_marker_array_sub_;

        ros::ServiceServer gripper_names_srv_;
        ros::ServiceServer gripper_attached_node_indices_srv_;
        ros::ServiceServer gripper_pose_srv_;
        ros::ServiceServer gripper_collision_check_srv_;
        ros::ServiceServer cover_points_srv_;
        ros::ServiceServer mirror_line_srv_;
        ros::ServiceServer free_space_graph_srv_;
        std::vector<geometry_msgs::Point> object_initial_configuration_;
        ros::ServiceServer object_initial_configuration_srv_;
        ros::ServiceServer object_current_configuration_srv_;

        actionlib::SimpleActionServer<smmap_msgs::CmdGrippersTrajectoryAction> cmd_grippers_traj_as_;
        smmap_msgs::CmdGrippersTrajectoryGoalConstPtr cmd_grippers_traj_goal_;
        smmap_msgs::CmdGrippersTrajectoryResult cmd_grippers_traj_result_;
        size_t cmd_grippers_traj_next_index_;

//        ros::ServiceServer execute_gripper_movement_and_update_sim_srv_;
        actionlib::SimpleActionServer<smmap_msgs::TestGrippersPosesAction> test_grippers_poses_as_;

        ////////////////////////////////////////////////////////////////////////
        // Low-pass filter / quasi static world data structures
        ////////////////////////////////////////////////////////////////////////

        double base_sim_time_;
        const size_t num_timesteps_to_execute_per_gripper_cmd_;

        ////////////////////////////////////////////////////////////////////////
        // Key Handler for our Custom Scene
        ////////////////////////////////////////////////////////////////////////

        class CustomKeyHandler : public osgGA::GUIEventHandler
        {
            public:
                CustomKeyHandler(CustomScene &scene);

                bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);

            private:
                CustomScene& scene_;

                // stores the current gripper motion type
                GripperKinematicObject::Ptr current_gripper_;
                bool translate_gripper_;
                bool rotate_gripper_;

                // used to track how much to drag by
                bool start_dragging_;
                float mouse_last_x_;
                float mouse_last_y_;

                GripperKinematicObject::Ptr getGripper(size_t gripper_num);
        };
};

#endif
