#ifndef CUSTOM_SCENE_H
#define CUSTOM_SCENE_H

#include <boost/thread/mutex.hpp>
#include <Eigen/Dense>

#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/rope.h"

#include "simulation/config_bullet.h"
#include "simulation/config_scene.h"
#include "simulation/config_viewer.h"

#include "gripper_kinematic_object.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <smmap/task.h>
#include <smmap_msgs/messages.h>

class CustomScene : public Scene
{
    public:
        CustomScene( ros::NodeHandle& nh, smmap::DeformableType deformable_type, smmap::TaskType task_type );

        ////////////////////////////////////////////////////////////////////////
        // Main function that makes things happen
        ////////////////////////////////////////////////////////////////////////

        void run( bool syncTime = false );

    private:
        ////////////////////////////////////////////////////////////////////////
        // Construction helper functions
        ////////////////////////////////////////////////////////////////////////

        void makeTable( const float half_side_length, const bool set_cover_points = false );
        void makeCylinder( const bool set_cover_points = false );
        void makeRope();
        void makeCloth();

        void makeRopeWorld();
        void makeClothWorld();
        void findClothCornerNodes();

        ////////////////////////////////////////////////////////////////////////
        // Main loop helper functions
        ////////////////////////////////////////////////////////////////////////

        void moveGrippers();
        void publishSimulatorFbk();

        ////////////////////////////////////////////////////////////////////////
        // Internal helper functions
        ////////////////////////////////////////////////////////////////////////

        std::vector< btVector3 > getDeformableObjectNodes();
        Eigen::Vector3d collisionHelper( const std::string& gripper_name );

        ////////////////////////////////////////////////////////////////////////
        // ROS Callbacks
        ////////////////////////////////////////////////////////////////////////

        bool cmdGripperTrajCallback(
                smmap_msgs::CmdGrippersTrajectory::Request& req,
                smmap_msgs::CmdGrippersTrajectory::Response& res );
        bool getGripperNamesCallback(
                smmap_msgs::GetGripperNames::Request& req,
                smmap_msgs::GetGripperNames::Response& res );
        bool getGripperAttachedNodeIndicesCallback(
                smmap_msgs::GetGripperAttachedNodeIndices::Request& req,
                smmap_msgs::GetGripperAttachedNodeIndices::Response& res );
        bool getGripperPoseCallback(
                smmap_msgs::GetGripperPose::Request& req,
                smmap_msgs::GetGripperPose::Response& res );
        bool getCoverPointsCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res );
        bool getObjectInitialConfigurationCallback(
                smmap_msgs::GetPointSet::Request& req,
                smmap_msgs::GetPointSet::Response& res );

        void visualizationMarkerCallback( visualization_msgs::Marker marker );
        void visualizationMarkerArrayCallback( visualization_msgs::MarkerArray marker_array );

        ////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////

        // Our internal version of ros::spin()
        static void spin( double loop_rate );

        ////////////////////////////////////////////////////////////////////////
        // Pre-step Callbacks
        ////////////////////////////////////////////////////////////////////////

        void drawAxes();

        PlotPoints::Ptr plot_points_;
        PlotLines::Ptr plot_lines_;

        std::map< std::string,  PlotLines::Ptr > visualization_line_markers_;
        std::map< std::string,  PlotSpheres::Ptr > visualization_sphere_markers_;

        ////////////////////////////////////////////////////////////////////////
        // Post-step Callbacks
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // Task Variables TODO to be moved into a CustomSceneConfig file
        ////////////////////////////////////////////////////////////////////////

        smmap::DeformableType deformable_type_;
        smmap::TaskType task_type_;

        ////////////////////////////////////////////////////////////////////////
        // Grippers
        ////////////////////////////////////////////////////////////////////////

        std::map< std::string, PlotAxes::Ptr > gripper_axes_;
        std::map< std::string, GripperKinematicObject::Ptr > grippers_;
        std::vector< GripperKinematicObject::Ptr > auto_grippers_;

        ////////////////////////////////////////////////////////////////////////
        // Shared world objects
        ////////////////////////////////////////////////////////////////////////

        static constexpr float TABLE_X = 0.0; // METERS
        static constexpr float TABLE_Y = 0.0; // METERS
        static constexpr float TABLE_Z = 0.7; // METERS
        static constexpr float TABLE_THICKNESS = 0.05; // METERS
        BoxObject::Ptr table_;

        ////////////////////////////////////////////////////////////////////////
        // Rope world objects
        ////////////////////////////////////////////////////////////////////////

        static constexpr float ROPE_SEGMENT_LENGTH = 0.025; // METERS
        static constexpr float ROPE_RADIUS = 0.01; // METERS
        static constexpr int ROPE_NUM_LINKS = 50;
        static constexpr float ROPE_GRIPPER_APPERTURE = 0.03; // METERS
        static constexpr float ROPE_TABLE_HALF_SIDE_LENGTH = 1.5; // METERS
        static constexpr float ROPE_CYLINDER_RADIUS = 0.15; // METERS
        static constexpr float ROPE_CYLINDER_HEIGHT = 0.3; // METERS
        CylinderStaticObject::Ptr cylinder_;
        boost::shared_ptr< CapsuleRope > rope_;

        ////////////////////////////////////////////////////////////////////////
        // Cloth world objects
        ////////////////////////////////////////////////////////////////////////

        static constexpr float CLOTH_HALF_SIDE_LENGTH = 0.25; // METERS
        static constexpr float CLOTH_X = TABLE_X + CLOTH_HALF_SIDE_LENGTH; // METERS
        static constexpr float CLOTH_Y = TABLE_Y; // METERS
        static constexpr float CLOTH_Z = TABLE_Z + 0.1; // METERS
        static constexpr int CLOTH_DIVS = 45;
        static constexpr float CLOTH_GRIPPER_APPERTURE = 0.1; // METERS
        static constexpr float CLOTH_TABLE_HALF_SIDE_LENGTH = 0.2; // METERS
        BulletSoftObject::Ptr cloth_;
        std::vector< int > cloth_corner_node_indices_;

        ////////////////////////////////////////////////////////////////////////
        // Coverage task objects
        ////////////////////////////////////////////////////////////////////////

        std::vector< btVector3 > cover_points_;

        ////////////////////////////////////////////////////////////////////////
        // ROS Objects and Helpers
        ////////////////////////////////////////////////////////////////////////

        ros::NodeHandle nh_;

        // global input mutex
        boost::mutex input_mtx_;

        ros::ServiceServer cmd_grippers_traj_srv_;
        smmap_msgs::CmdGrippersTrajectory::Request next_gripper_traj_;
        smmap_msgs::CmdGrippersTrajectory::Request curr_gripper_traj_;
        size_t gripper_traj_index_;
        bool new_gripper_traj_ready_;

        ros::Publisher simulator_fbk_pub_;

        ros::ServiceServer gripper_names_srv_;
        ros::ServiceServer gripper_attached_node_indices_srv_;
        ros::ServiceServer gripper_pose_srv_;
        ros::ServiceServer cover_points_srv_;
        ros::ServiceServer object_initial_configuration_srv_;
        std::vector< geometry_msgs::Point > object_initial_configuration_;

        ros::Subscriber visualization_marker_sub_;
        ros::Subscriber visualization_marker_array_sub_;
};

#endif
