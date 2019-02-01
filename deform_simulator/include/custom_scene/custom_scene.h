#ifndef CUSTOM_SCENE_H
#define CUSTOM_SCENE_H

#include <atomic>
#include <mutex>
#include <unordered_map>

#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/rope.h"

#include "simulation/config_bullet.h"
#include "simulation/config_scene.h"
#include "simulation/config_viewer.h"
#include "simulation/recording.h"

#include "gripper_kinematic_object.h"
#include "manual_gripper_path.h"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/log.hpp>
#include <sdf_tools/tagged_object_collision_map.hpp>
#include <sdf_tools/sdf.hpp>
#include <deformable_manipulation_experiment_params/task_enums.h>
#include <deformable_manipulation_experiment_params/xyzgrid.h>
#include <deformable_manipulation_msgs/messages.h>

struct SimForkResult
{
    public:
        BulletInstance::Ptr bullet_;
        OSGInstance::Ptr osg_;
        Fork::Ptr fork_;
        CapsuleRope::Ptr rope_;
        BulletSoftObject::Ptr cloth_;
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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CustomScene(ros::NodeHandle& nh, const smmap::DeformableType deformable_type, const smmap::TaskType task_type);

        ////////////////////////////////////////////////////////////////////////
        // Main function that makes things happen
        ////////////////////////////////////////////////////////////////////////

        void initialize();
        bool initialized() const;
        void run();
        void terminate();
        void triggerTerminateService();
        bool finished();

    private:
        bool initialized_;
        std::atomic<bool> finished_;

    protected:
        ////////////////////////////////////////////////////////////////////////
        // Construction helper functions
        ////////////////////////////////////////////////////////////////////////

        void getWorldToBulletTransform();
        void initializePublishersSubscribersAndServices();
        void shutdownPublishersSubscribersAndServices();

        void makeBulletObjects();

        void makeRope();
        CapsuleRope::Ptr duplicateRopeAtInitialConfig() const;

        void makeCloth();
        void createClothMirrorLine();
        void makeClothLines();
        void updateClothLinesCallback();
        void makeGripperForceLines();
        void updateGripperForceLinesCallback();


        void makeRopeSingleRobotControlledGrippper();
        void makeRopeTwoRobotControlledGrippers();
        void makeClothTwoRobotControlledGrippers();
        void makeClothTwoHumanControlledGrippers();
        void addGrippersAndAxesToWorld();
        void makeCollisionCheckGripper();


        void makeTableSurface(
                const bool create_cover_points,
                const float stepsize = -1.0f,
                const bool add_legs = false);
        void makeCylinder(const bool create_cover_points);
        void makeSinglePoleObstacles();
        void makeWallObstacles();
        void makeClothWallObstacles();
        void makeClothDoubleSlitObstacles();
        void makeRopeMazeObstacles();
        void makeRopeZigMatchObstacles();
        void makeRopeHooksObstacles();
        void makeClothHooksObstacles();

        void makeGenericRegionCoverPoints();
        void loadCoverPointsFromFile();

        void createEdgesToNeighbours(
                const int64_t x_starting_ind,
                const int64_t y_starting_ind,
                const int64_t z_starting_ind);
        void createFreeSpaceGraph(const bool draw_graph_corners = false);
        void createCollisionMapAndSDF();

        ////////////////////////////////////////////////////////////////////////
        // Internal helper functions
        ////////////////////////////////////////////////////////////////////////

        static void SetGripperTransform(
                const std::map<std::string, GripperKinematicObject::Ptr>& grippers_map,
                const std::string& name,
                const btTransform& pose_in_bt_coords);

        btTransform getTransform(const std::string& parent, const std::string& child);

    public:
        geometry_msgs::PoseStamped transformPoseToBulletFrame(
                const std_msgs::Header& input_header,
                const geometry_msgs::Pose& pose_in_input_frame) const;

    protected:
        // Note that these really all should be ConstPtr, but far too much work to make that happen cleanly
        deformable_manipulation_msgs::WorldState createSimulatorFbk(
                const CapsuleRope::ConstPtr rope,
                const BulletSoftObject::ConstPtr cloth,
                const std::map<std::string, GripperKinematicObject::Ptr>& grippers) const;

        std::vector<btVector3> getDeformableObjectNodes(
                const CapsuleRope::ConstPtr rope,
                const BulletSoftObject::ConstPtr cloth) const;
        void setDeformableState(const std::vector<btVector3>& deform_config);

        // Note that these really should be ConstPtr, but far too much work to make that happen cleanly
        btPointCollector collisionHelper(const GripperKinematicObject::Ptr& gripper) const;
        btPointCollector collisionHelper(const SphereObject::Ptr& sphere) const;

        ////////////////////////////////////////////////////////////////////////////////
        // Fork and Fork-visualization management
        ////////////////////////////////////////////////////////////////////////////////

        std::shared_ptr<ViewerData> createVisualizerForFork(SimForkResult& fork_result);

        SimForkResult createForkWithNoSimulationDone(
                const Environment::Ptr environment_to_fork,
                BulletSoftObject::Ptr cloth_to_fork,
                boost::shared_ptr<CapsuleRope> rope_to_fork,
                std::map<std::string, GripperKinematicObject::Ptr> grippers_to_fork);

        SimForkResult createForkWithNoSimulationDone(
                const std::vector<std::string>& gripper_names,
                const std::vector<btTransform>& gripper_poses_in_bt_coords);

        SimForkResult simulateInNewFork(
                const std::vector<std::string>& gripper_names,
                const std::vector<btTransform>& gripper_poses_in_bt_coords);

        ////////////////////////////////////////////////////////////////////////
        // ROS Callbacks
        ////////////////////////////////////////////////////////////////////////

        bool getGripperNamesCallback(
                deformable_manipulation_msgs::GetGripperNames::Request& req,
                deformable_manipulation_msgs::GetGripperNames::Response& res);
        bool getGripperAttachedNodeIndicesCallback(
                deformable_manipulation_msgs::GetGripperAttachedNodeIndices::Request& req,
                deformable_manipulation_msgs::GetGripperAttachedNodeIndices::Response& res);

        // Used by StretchingAvoidanceController in SMMAP
        bool getGripperStretchingVectorInfoCallback(
                deformable_manipulation_msgs::GetGripperStretchingVectorInfo::Request& req,
                deformable_manipulation_msgs::GetGripperStretchingVectorInfo::Response& res);

        bool getGripperPoseCallback(
                deformable_manipulation_msgs::GetGripperPose::Request& req,
                deformable_manipulation_msgs::GetGripperPose::Response& res);
        bool getRobotConfigurationCallback(
                deformable_manipulation_msgs::GetRobotConfiguration::Request& req,
                deformable_manipulation_msgs::GetRobotConfiguration::Response& res);
        bool gripperCollisionCheckCallback(
                deformable_manipulation_msgs::GetGripperCollisionReport::Request& req,
                deformable_manipulation_msgs::GetGripperCollisionReport::Response& res);
        bool getCoverPointsCallback(
                deformable_manipulation_msgs::GetPointSet::Request& req,
                deformable_manipulation_msgs::GetPointSet::Response& res);
        bool getCoverPointNormalsCallback(
                deformable_manipulation_msgs::GetVector3Set::Request& req,
                deformable_manipulation_msgs::GetVector3Set::Response& res);
        bool getMirrorLineCallback(
                deformable_manipulation_msgs::GetMirrorLine::Request& req,
                deformable_manipulation_msgs::GetMirrorLine::Response& res);

        bool getFreeSpaceGraphCallback(
                deformable_manipulation_msgs::GetFreeSpaceGraphRequest& req,
                deformable_manipulation_msgs::GetFreeSpaceGraphResponse& res);
        bool getSignedDistanceFieldCallback(
                deformable_manipulation_msgs::GetSignedDistanceFieldRequest& req,
                deformable_manipulation_msgs::GetSignedDistanceFieldResponse& res);

        bool getObjectInitialConfigurationCallback(
                deformable_manipulation_msgs::GetPointSet::Request& req,
                deformable_manipulation_msgs::GetPointSet::Response& res);
        bool getObjectCurrentConfigurationCallback(
                deformable_manipulation_msgs::GetPointSet::Request& req,
                deformable_manipulation_msgs::GetPointSet::Response& res);
        bool getRopeCurrentNodeTransforms(
                deformable_manipulation_msgs::GetPoseSet::Request& req,
                deformable_manipulation_msgs::GetPoseSet::Response& res);

        bool executeRobotMotionCallback(
                deformable_manipulation_msgs::ExecuteRobotMotion::Request& req,
                deformable_manipulation_msgs::ExecuteRobotMotion::Response& res);

        bool testRobotMotionMicrostepsCallback(
                deformable_manipulation_msgs::TestRobotMotionMicrosteps::Request& req,
                deformable_manipulation_msgs::TestRobotMotionMicrosteps::Response& res);

        void testRobotMotionExecuteCallback(
                const deformable_manipulation_msgs::TestRobotMotionGoalConstPtr& goal);

        ////////////////////////////////////////////////////////////////////
        // Stuff, and things
        ////////////////////////////////////////////////////////////////////

    // Public to make things easy for writing the viewer manager
    public:
        /// Protects against multiple threads accessing data that modifies the
        /// environment/simulation at the same time
        std::mutex sim_mutex_;
    private:
        std::shared_ptr<ScreenRecorder> screen_recorder_;

        ////////////////////////////////////////////////////////////////////////
        // Pre-step Callbacks
        ////////////////////////////////////////////////////////////////////////

        void drawAxes();

        PlotPoints::Ptr plot_points_ = nullptr;
        PlotLines::Ptr plot_lines_ = nullptr;
        PlotLines::Ptr strain_lines_ = nullptr;
        PlotLines::Ptr gripper_force_lines_ = nullptr;

        ////////////////////////////////////////////////////////////////////////
        // Post-step Callbacks
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Task Variables
        // TODO: to be moved into a CustomSceneConfig file?
        ////////////////////////////////////////////////////////////////////////

        const smmap::DeformableType deformable_type_;
        const smmap::TaskType task_type_;
        btScalar max_strain_; // Not constant so that we can allow it to not be a parameter anywhere if we are not visualizing the strain lines

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

        std::unordered_map<std::string, BulletObject::Ptr> world_obstacles_;

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
        std::vector<btVector3> cover_point_normals_;
        deformable_manipulation_msgs::GetMirrorLine::Response mirror_line_data_;
        PlotLines::Ptr miror_line_vis_;

        ////////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////////

    // Public to make things easy for writing the viewer manager
    public:
        ros::NodeHandle nh_;
        ros::NodeHandle ph_;
        ros::AsyncSpinner ros_spinner_;

        const std::string bullet_frame_name_;
        const std::string world_frame_name_;
        tf2_ros::Buffer tf_buffer_;
        const tf2_ros::TransformListener tf_listener_;

        // Cannot be const because we need to wait for the buffer to collect data
        // Stores the transform that makes point_in_world_frame = world_to_bullet_tf_ * point_in_bullet_frame
        btTransform world_to_bullet_tf_;
        // Broadcasts an identity transform between the world and bullet if no one else is broadcasting within 10 seconds of startup
        tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    private:
        // Uses bullet (scaled) translational distances, and the bullet frame
        const smmap::XYZGrid work_space_grid_;
        arc_dijkstras::Graph<btVector3> free_space_graph_;
        size_t num_graph_edges_;
        std::vector<int64_t> cover_ind_to_free_space_graph_ind_;
        std::vector<PlotAxes::Ptr> graph_corners_;

        // Uses world (unscaled) translational distances, and the world frame
        const int sdf_resolution_scale_;
        sdf_tools::TaggedObjectCollisionMapGrid collision_map_for_export_;
        sdf_tools::SignedDistanceField sdf_for_export_;
        visualization_msgs::Marker collision_map_marker_for_export_;
        enum ObjectIds
        {
            FREESPACE, // This must be the first enum in the list, others can be added after it with no problems
            GENERIC_OBSTACLE,
            PEG,
            LAST_ID // This must the the last enum in the list, others can be added before it with no problems
        };

        std::atomic<bool> sim_running_;
        ros::Publisher simulator_fbk_pub_;

        ros::ServiceServer gripper_names_srv_;
        ros::ServiceServer gripper_attached_node_indices_srv_;
        ros::ServiceServer gripper_stretching_vector_info_srv_;
        ros::ServiceServer gripper_pose_srv_;
        ros::ServiceServer robot_configuration_srv_;
        ros::ServiceServer gripper_collision_check_srv_;
        ros::ServiceServer cover_points_srv_;
        ros::ServiceServer cover_point_normals_srv_;
        ros::ServiceServer mirror_line_srv_;
        ros::ServiceServer free_space_graph_srv_;
        ros::ServiceServer signed_distance_field_srv_;
        ros::ServiceServer rope_node_transforms_srv_;

        std::vector<geometry_msgs::Point> object_initial_configuration_;
        ros::ServiceServer object_initial_configuration_srv_;
        ros::ServiceServer object_current_configuration_srv_;

        ros::ServiceServer execute_gripper_movement_srv_;
        actionlib::SimpleActionServer<deformable_manipulation_msgs::TestRobotMotionAction> test_grippers_poses_as_;
        ros::ServiceServer test_robot_motion_microsteps_srv_;

        ////////////////////////////////////////////////////////////////////////
        // Low-pass filter / quasi static world data structures
        ////////////////////////////////////////////////////////////////////////

        double base_sim_time_;
        const size_t num_timesteps_to_execute_per_gripper_cmd_;

        ////////////////////////////////////////////////////////////////////////
        // Logging
        ////////////////////////////////////////////////////////////////////////

        Log::Log simulation_time_logger_;

        ////////////////////////////////////////////////////////////////////////
        // Key Handler for our Custom Scene
        ////////////////////////////////////////////////////////////////////////

        class CustomKeyHandler : public osgGA::GUIEventHandler
        {
            public:
                CustomKeyHandler(CustomScene &scene);

                bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

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
