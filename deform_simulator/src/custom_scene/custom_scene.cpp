#include "custom_scene/custom_scene.h"

#include <omp.h>

#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <future>
#include <boost/polymorphic_pointer_cast.hpp>
#include <ros/callback_queue.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deformable_manipulation_experiment_params/ros_params.hpp>
#include <deformable_manipulation_experiment_params/serialization.h>
#include <deformable_manipulation_experiment_params/utility.hpp>
#include <arc_utilities/serialization_ros.hpp>
#include <arc_utilities/timing.hpp>
#include <arc_utilities/zlib_helpers.hpp>

#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>

#include <BulletSoftBody/btSoftBodyHelpers.h>

#include <bullet_helpers/bullet_internal_conversions.hpp>
#include <bullet_helpers/bullet_ros_conversions.hpp>
#include <bullet_helpers/bullet_math_helpers.hpp>
#include <bullet_helpers/bullet_pretty_print.hpp>
#include <bullet_helpers/bullet_cloth_helpers.hpp>

#include "utils/util.h"
#include "custom_scene/internal_utils.hpp"
#include "custom_scene/table_kinematic_object.h"

using namespace BulletHelpers;
using namespace smmap;
namespace dmm = deformable_manipulation_msgs;
using ColorBuilder = arc_helpers::RGBAColorBuilder<std_msgs::ColorRGBA>;

// TODO: Put this magic number somewhere else
#pragma message "Magic numbers here"
static const btVector4 FLOOR_COLOR(224.0f/255.0f, 224.0f/255.0f, 224.0f/255.0f, 1.0f);
static const btVector4 GRIPPER_COLOR(0.0f, 0.0f, 0.6f, 1.0f);

// NOTE: this 0.3 ought to be 2*M_PI/21=0.299199... however that chops off the last value, probably due to rounding
#define ROPE_CYLINDER_ANGLE_DISCRETIZATION                          (0.3f)                  // radians
#define ROPE_CYLINDER_HEIGHT_DISCRETIZATION_RATIO                   (30.0f)                 // unitless
#define ROPE_CYLINDER_TWO_GRIPPERS_START_HEIGHT_RATIO               (8.0f)                  // unitless
#define ROPE_CYLINDER_TWO_GRIPPERS_END_HEIGHT_RATIO                 (8.0f)                  // unitless
#define CLOTH_CYLINDER_ANGLE_DISCRETIZATION_RATIO                   (8.0f)                  // unitless
#define CLOTH_WAFR_HORIZONTAL_CYLINDER_START_ANGLE                  ((float)M_PI - 0.524f)  // radians
#define CLOTH_WAFR_HORIZONTAL_CYLINDER_END_ANGLE                    ((float)2.0f * M_PI)    // radians
#define CLOTH_WAFR_HORIZONTAL_CYLINDER_ANGLE_DISCRETIZATION         (0.523f)                // radians
#define CLOTH_WAFR_HORIZONTAL_CYLINDER_HEIGHT_DISCRETIZATION_RATIO  (30.0f)                 // unitless
#define CLOTH_TABLE_COVERAGE_COVER_POINT_STEPSIZE                   (0.0125f * METERS)      // meters
#define CLOTH_SINGLE_POLE_DOUBLE_SLIT_COVER_POINT_STEPSIZE          (0.025f * METERS)       // meters

static std::vector<geometry_msgs::Pose> InterpolateVectors(
        const std::vector<geometry_msgs::Pose>& vec1,
        const std::vector<geometry_msgs::Pose>& vec2,
        const double ratio)
{
    assert(vec1.size() == vec2.size());
    std::vector<geometry_msgs::Pose> result(vec1.size());
    for (size_t idx = 0; idx < vec1.size(); ++idx)
    {
        result[idx] = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                    EigenHelpers::Interpolate(
                        EigenHelpersConversions::GeometryPoseToEigenIsometry3d(vec1[idx]),
                        EigenHelpersConversions::GeometryPoseToEigenIsometry3d(vec2[idx]),
                        ratio));
    }
    return result;
}

static std::vector<btTransform> InterpolateVectors(
        const std::vector<btTransform>& vec1,
        const std::vector<btTransform>& vec2,
        const btScalar ratio)
{
    assert(vec1.size() == vec2.size());
    std::vector<btTransform> result(vec1.size());
    for (size_t idx = 0; idx < vec1.size(); ++idx)
    {
        result[idx] = btTransform(vec1[idx].getRotation().slerp(vec2[idx].getRotation(), ratio),
                                  vec1[idx].getOrigin().lerp(vec2[idx].getOrigin(), ratio));
    }
    return result;
}


static bool IsPointInsideAABB(
        const btVector3& box_min,
        const btVector3& box_max,
        const btVector3& point)
{
    return (point.x() >= box_min.x() && point.x() <= box_max.x()) &&
           (point.y() >= box_min.y() && point.y() <= box_max.y()) &&
           (point.z() >= box_min.z() && point.z() <= box_max.z());
}

static bool IsPointInsideOABB(
        const btTransform& com,
        const btVector3& half_extents,
        const btVector3& point)
{
    return IsPointInsideAABB(-half_extents,
                             half_extents,
                             com.invXform(point));
}

////////////////////////////////////////////////////////////////////////////////
// Constructor and Destructor
////////////////////////////////////////////////////////////////////////////////

CustomScene::CustomScene(ros::NodeHandle& nh,
                         const DeformableType deformable_type,
                         const TaskType task_type)
    : initialized_(false)
    , finished_(false)
    , screen_recorder_(nullptr)
    , plot_points_(boost::make_shared<PlotPoints>(0.4f * METERS))
    , plot_lines_(boost::make_shared<PlotLines>(0.25f * METERS))
    , deformable_type_(deformable_type)
    , task_type_(task_type)
    , advance_grippers_(true)
    , nh_(nh)
    , ph_("~")
    , ros_spinner_(1)
    , bullet_frame_name_(smmap::GetBulletFrameName())
    , world_frame_name_(smmap::GetWorldFrameName())
    , tf_buffer_()
    , tf_listener_(tf_buffer_)

    // Uses the bullet_frame and bullet coords
    , work_space_grid_(Eigen::Isometry3d(Eigen::Translation3d(
                                             GetWorldXMinBulletFrame(nh) * METERS,
                                             GetWorldYMinBulletFrame(nh) * METERS,
                                             GetWorldZMinBulletFrame(nh) * METERS)),
                       bullet_frame_name_,
                       GetWorldXStep(nh) * METERS,
                       GetWorldYStep(nh) * METERS,
                       GetWorldZStep(nh) * METERS,
                       GetWorldXNumSteps(nh),
                       GetWorldYNumSteps(nh),
                       GetWorldZNumSteps(nh))
    , free_space_graph_((size_t)work_space_grid_.getNumCells() + 1000)
    , num_graph_edges_(0)


    // Uses bullet_frame but world distances while being built, is transformed to the world frame once built
    , sdf_resolution_scale_(GetSDFResolutionScale(nh))
    , collision_map_for_export_(Eigen::Isometry3d(Eigen::Translation3d(
                                                      GetWorldXMinBulletFrame(nh),
                                                      GetWorldYMinBulletFrame(nh),
                                                      GetWorldZMinBulletFrame(nh))),
                                bullet_frame_name_,
                                work_space_grid_.minStepDimension() / METERS / sdf_resolution_scale_,
                                GetWorldXMaxBulletFrame(nh) - GetWorldXMinBulletFrame(nh),
                                GetWorldYMaxBulletFrame(nh) - GetWorldYMinBulletFrame(nh),
                                GetWorldZMaxBulletFrame(nh) - GetWorldZMinBulletFrame(nh),
                                sdf_tools::TAGGED_OBJECT_COLLISION_CELL(0.0, 0))
    , sim_running_(false)
    , test_grippers_poses_as_(nh_,
                              GetTestRobotMotionTopic(nh_),
                              boost::bind(&CustomScene::testRobotMotionExecuteCallback, this, _1), false)
    , generate_transition_data_as_(nh_,
                                   GetGenerateTransitionDataTopic(nh_),
                                   boost::bind(&CustomScene::generateTransitionDataExecuteCallback, this, _1), false)
    , num_timesteps_to_execute_per_gripper_cmd_(GetNumSimstepsPerGripperCommand(ph_))
    , simulation_time_logger_(GetLogFolder(nh_) + "bullet_sim_time.txt")
{
    makeBulletObjects();
    // We'll create the various ROS publishers, subscribers and services later so
    // that TF has a chance to collect some data before we need to use it
}

////////////////////////////////////////////////////////////////////////////////
// Main function that makes things happen
////////////////////////////////////////////////////////////////////////////////

void CustomScene::initialize()
{
    // ros::ok() is only here to bypass things if a fast SIGINT is received
    if (!ros::ok())
    {
        return;
    }

    // Run the startup code for the viewer and everything else
    {
        // Start listening for TF messages
        ros_spinner_.start();
        getWorldToBulletTransform();

        // Store the initial configuration as it will be needed by other libraries
        // TODO: find a better way to do this that exposes less internals
        object_initial_configuration_ = toRosPointVector(world_to_bullet_tf_, getDeformableObjectNodes(rope_, cloth_), METERS);

        ViewerConfig::windowWidth = GetViewerWidth(ph_);
        ViewerConfig::windowHeight = GetViewerHeight(ph_);

        // Note that viewer cleans up this memory
        viewer.addEventHandler(new CustomKeyHandler(*this));

        // When the viewer closes, shutdown the simulation
        // The terminate callback is used instead of terminate directly
        //   in order to properly close all elements of this program
        //   (I.e.; the bullet viewer and the QT viewer
        addVoidCallback(osgGA::GUIEventAdapter::EventType::CLOSE_WINDOW,
                        boost::bind(&CustomScene::triggerTerminateService, this));

        addPreStepCallback(boost::bind(&CustomScene::drawAxes, this));

        // if syncTime is set, the simulator blocks until the real time elapsed
        // matches the simulator time elapsed, or something, it's not clear
        setSyncTime(false);
        setDrawing(ROSHelpers::GetParam(ph_, "start_bullet_viewer", true));
        if (drawingOn)
        {
            startViewer();
        }
        screen_recorder_ = std::make_shared<ScreenRecorder>(&viewer, GetScreenshotsEnabled(ph_), GetScreenshotFolder(nh_));

        // Create a thread to create the free space graph and collision map while the object settles
        const bool draw_free_space_graph_corners = drawingOn && false;
        auto free_space_graph_future = std::async(std::launch::async, &CustomScene::createFreeSpaceGraph, this, draw_free_space_graph_corners);
        auto collision_map_future = std::async(std::launch::async, &CustomScene::createCollisionMapAndSDF, this);

        // Let the object settle before anything else happens
        {
            const float settle_time = GetSettlingTime(ph_);
            ROS_INFO("Waiting %.1f seconds for the scene to settle", settle_time);
            stepFor(BulletConfig::dt, settle_time);
        }

        // Wait for the graph to be finished being made
        {
            while (free_space_graph_future.wait_for(std::chrono::microseconds(10000)) != std::future_status::ready)
            {
                step(0);
            }
            free_space_graph_future.get();

            while (collision_map_future.wait_for(std::chrono::microseconds(10000)) != std::future_status::ready)
            {
                step(0);
            }
            collision_map_future.get();
        }

        base_sim_time_ = simTime;

        // Startup the action servers
        test_grippers_poses_as_.start();
        generate_transition_data_as_.start();
    }

    ROS_INFO("Simulation ready, starting publishers, subscribers, and services");
    initializePublishersSubscribersAndServices();

    initialized_ = true;
}

bool CustomScene::initialized() const
{
    return initialized_;
}

void CustomScene::run()
{
    // Used for constant visualization, independent of robot control structure
    ros::Publisher bullet_visualization_pub = nh_.advertise<visualization_msgs::MarkerArray>("bullet_visualization_export", 1);
    visualization_msgs::MarkerArray bullet_visualization_markers = collision_map_marker_array_for_export_;
    bullet_visualization_markers.markers.reserve(bullet_visualization_markers.markers.size() + 5);

    // Build markers for regular publishing
    size_t deformable_object_marker_ind;
    size_t first_gripper_marker_ind;
    {
//        bullet_visualization_markers.markers.push_back(collision_map_marker_for_export_);

        deformable_object_marker_ind = bullet_visualization_markers.markers.size();
        visualization_msgs::Marker deformable_object_state_marker;
        deformable_object_state_marker.header.frame_id = world_frame_name_;
        deformable_object_state_marker.ns = "deformable_object";
        deformable_object_state_marker.type = visualization_msgs::Marker::POINTS;
        deformable_object_state_marker.scale.x = 0.01;
        deformable_object_state_marker.scale.y = 0.01;
        deformable_object_state_marker.color = ColorBuilder::MakeFromFloatColors(0.0f, 1.0f, 0.0f, 0.5f);
        bullet_visualization_markers.markers.push_back(deformable_object_state_marker);

        visualization_msgs::Marker cover_points_marker;
        cover_points_marker.header.frame_id = world_frame_name_;
        cover_points_marker.ns = "cover_points";
        cover_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        cover_points_marker.scale.x = 0.05;
        cover_points_marker.scale.y = 0.05;
        cover_points_marker.scale.z = 0.05;
        cover_points_marker.color = ColorBuilder::MakeFromFloatColors(1.0f, 0.0f, 1.0f, 0.3f);
        cover_points_marker.points = toRosPointVector(world_to_bullet_tf_, cover_points_, METERS);
        bullet_visualization_markers.markers.push_back(cover_points_marker);

        first_gripper_marker_ind = bullet_visualization_markers.markers.size();
        for (size_t ind = 0; ind < auto_grippers_.size(); ++ind)
        {
            const auto& gripper_name = auto_grippers_[ind];
            const auto& gripper = grippers_.at(gripper_name);

            visualization_msgs::Marker gripper_marker;
            gripper_marker.header.frame_id = world_frame_name_;
            gripper_marker.ns = "grippers";
            gripper_marker.id = (int)ind + 1;
            gripper_marker.type = visualization_msgs::Marker::CUBE;
            gripper_marker.scale.x = gripper->getHalfExtents().x() * 2.0 / METERS;
            gripper_marker.scale.y = gripper->getHalfExtents().y() * 2.0 / METERS;
            gripper_marker.scale.z = gripper->getHalfExtents().z() * 2.0 / METERS;
            gripper_marker.pose = toRosPose(world_to_bullet_tf_, gripper->getWorldTransform(), METERS);
            gripper_marker.color = ColorBuilder::MakeFromFloatColors(0, 0, 1, 1);

            bullet_visualization_markers.markers.push_back(gripper_marker);
        }
    }

    ROS_INFO("Simulation spinning...");
    sim_running_ = true;
    // Run the simulation - this loop only redraws the scene, actual work is done in service callbacks
    while (sim_running_ && ros::ok())
    {
        dmm::WorldState sim_fbk;
        {
            std::lock_guard<std::mutex> lock(sim_mutex_);
            step(0);
            sim_fbk = createSimulatorFbk(rope_, cloth_, grippers_);
        }

        simulator_fbk_pub_.publish(sim_fbk);
        bullet_visualization_markers.markers[deformable_object_marker_ind].points = sim_fbk.object_configuration;
        for (size_t gripper_ind = 0; gripper_ind < sim_fbk.gripper_poses.size(); gripper_ind++)
        {
            bullet_visualization_markers.markers.at(first_gripper_marker_ind + gripper_ind).pose = sim_fbk.gripper_poses[gripper_ind];
        }
        bullet_visualization_pub.publish(bullet_visualization_markers);

        if (drawingOn)
        {
//            osg::Vec3d eye, center, up;
//            manip->getTransformation(eye, center, up);

//            std::cout << eye.x()/METERS    << "f, " << eye.y()/METERS    << "f, " << eye.z()/METERS << "f    "
//                      << center.x()/METERS << "f, " << center.y()/METERS << "f, " << center.z()/METERS << "f    "
//                      << up.x()/METERS     << "f, " << up.y()/METERS     << "f, " << up.z()/METERS << "f"
//                      << std::endl;
        }

        arc_helpers::Sleep(0.02);
    }

    shutdownPublishersSubscribersAndServices();
    finished_ = true;
}

void CustomScene::terminate()
{
    if (screen_recorder_ != nullptr)
    {
        screen_recorder_->zipScreenshots();
    }
    sim_running_ = false;
}

// Exists to ensure that all simulation components are terminated
// I.e. Bullet and QT
void CustomScene::triggerTerminateService()
{
    std_srvs::Empty srv_data;
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>(GetTerminateSimulationTopic(nh_));
    client.call(srv_data);
}

bool CustomScene::finished()
{
    return finished_;
}

////////////////////////////////////////////////////////////////////////////////
// Construction helper functions
////////////////////////////////////////////////////////////////////////////////

void CustomScene::getWorldToBulletTransform()
{
    // Get the transform from bullet to the world frame for use in data output
    const double timeout = GetTFWaitTime(ph_);
    if (timeout < 5.0)
    {
        ROS_WARN_STREAM("Waiting for tf from world to bullet frame for at most " << timeout << " seconds");
    }
    else
    {
        ROS_INFO_STREAM("Waiting for tf from world to bullet frame for at most " << timeout << " seconds");
    }
    geometry_msgs::TransformStamped world_to_bullet_tf_as_ros;
    try
    {
        world_to_bullet_tf_as_ros =
                tf_buffer_.lookupTransform(world_frame_name_, bullet_frame_name_, ros::Time(0.0), ros::Duration(timeout));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        ROS_WARN("Assuming this means that no transform has been broadcast from world to bullet, starting my own broadcaster with an identity transform");
        world_to_bullet_tf_as_ros.child_frame_id = bullet_frame_name_;

        world_to_bullet_tf_as_ros.transform.translation.x = 0.0;
        world_to_bullet_tf_as_ros.transform.translation.y = 0.0;
        world_to_bullet_tf_as_ros.transform.translation.z = 0.0;
        world_to_bullet_tf_as_ros.transform.rotation.x = 0.0;
        world_to_bullet_tf_as_ros.transform.rotation.y = 0.0;
        world_to_bullet_tf_as_ros.transform.rotation.z = 0.0;
        world_to_bullet_tf_as_ros.transform.rotation.w = 1.0;

        world_to_bullet_tf_as_ros.header.frame_id = world_frame_name_;
        world_to_bullet_tf_as_ros.header.stamp = ros::Time::now();

        static_broadcaster_.sendTransform(world_to_bullet_tf_as_ros);
    }
    world_to_bullet_tf_ = toBulletTransform(world_to_bullet_tf_as_ros.transform, METERS);
}

void CustomScene::initializePublishersSubscribersAndServices()
{
     ROS_INFO("Creating subscribers and publishers");
    // Publish to the feedback channel
    simulator_fbk_pub_ = nh_.advertise<dmm::WorldState>(
            GetWorldStateTopic(nh_), 20);

    ROS_INFO("Creating services");
    // Create a service to let others know the internal gripper names
    gripper_names_srv_ = nh_.advertiseService(
            GetGripperNamesTopic(nh_), &CustomScene::getGripperNamesCallback, this);

    // Create a service to let others know what nodes the grippers are attached too
    gripper_attached_node_indices_srv_ = nh_.advertiseService(
            GetGripperAttachedNodeIndicesTopic(nh_), &CustomScene::getGripperAttachedNodeIndicesCallback, this);

    // Create a service to let others know stretching vector information
    gripper_stretching_vector_info_srv_ = nh_.advertiseService(
            GetGripperStretchingVectorInfoTopic(nh_), &CustomScene::getGripperStretchingVectorInfoCallback, this);

    // Create a service to let others know the current gripper pose
    gripper_pose_srv_ = nh_.advertiseService(
            GetGripperPoseTopic(nh_), &CustomScene::getGripperPoseCallback, this);

    robot_configuration_srv_ = nh_.advertiseService(
            GetRobotConfigurationTopic(nh_), &CustomScene::getRobotConfigurationCallback, this);

    // Create a service to let others know the current gripper pose
    gripper_collision_check_srv_ = nh_.advertiseService(
            GetGripperCollisionCheckTopic(nh_), &CustomScene::gripperCollisionCheckCallback, this);

    // Create a service to let others know the cover points
    cover_points_srv_ = nh_.advertiseService(
            GetCoverPointsTopic(nh_), &CustomScene::getCoverPointsCallback, this);

    cover_point_normals_srv_ = nh_.advertiseService(
            GetCoverPointNormalsTopic(nh_), &CustomScene::getCoverPointNormalsCallback, this);

    // Create a service to let others know the mirror line data
    mirror_line_srv_ = nh_.advertiseService(
            GetMirrorLineTopic(nh_), &CustomScene::getMirrorLineCallback, this);

    // Create a service to let others know what the free space of the world looks like
    free_space_graph_srv_ = nh_.advertiseService(
            GetFreeSpaceGraphTopic(nh_), &CustomScene::getFreeSpaceGraphCallback, this);

    // Create a service to let others know what the signed distance field of the world looks like
    signed_distance_field_srv_ = nh_.advertiseService(
            GetSignedDistanceFieldTopic(nh_), &CustomScene::getSignedDistanceFieldCallback, this);

    // Create a service to let others know the object initial configuration
    object_initial_configuration_srv_ = nh_.advertiseService(
            GetObjectInitialConfigurationTopic(nh_), &CustomScene::getObjectInitialConfigurationCallback, this);

    // Create a service to let others know the object current configuration
    object_current_configuration_srv_ = nh_.advertiseService(
            GetObjectCurrentConfigurationTopic(nh_), &CustomScene::getObjectCurrentConfigurationCallback, this);

    // Create a service to let others know the exact full node transforms for the rope
    if (GetDeformableType(nh_) == DeformableType::ROPE)
    {
        rope_node_transforms_srv_ = nh_.advertiseService(
                    GetRopeCurrentNodeTransformsTopic(nh_), &CustomScene::getRopeCurrentNodeTransforms, this);
    }

    // Create a service to listen to in order to move the grippers and advance sim time
    execute_gripper_movement_srv_ = nh_.advertiseService(
            GetExecuteRobotMotionTopic(nh_), &CustomScene::executeRobotMotionCallback, this);

    // Create a service for testing robot motions given a particular rope starting point
    test_robot_motion_microsteps_srv_ = nh_.advertiseService(
            GetTestRobotMotionMicrostepsTopic(nh_), &CustomScene::testRobotMotionMicrostepsCallback, this);
}

void CustomScene::shutdownPublishersSubscribersAndServices()
{
    test_grippers_poses_as_.shutdown();
    generate_transition_data_as_.shutdown();
    simulator_fbk_pub_.shutdown();
    gripper_names_srv_.shutdown();
    gripper_attached_node_indices_srv_.shutdown();
    gripper_stretching_vector_info_srv_.shutdown();
    gripper_pose_srv_.shutdown();
    robot_configuration_srv_.shutdown();
    gripper_collision_check_srv_.shutdown();
    cover_points_srv_.shutdown();
    cover_point_normals_srv_.shutdown();
    mirror_line_srv_.shutdown();
    free_space_graph_srv_.shutdown();
    signed_distance_field_srv_.shutdown();
    object_initial_configuration_srv_.shutdown();
    object_current_configuration_srv_.shutdown();
    if (GetDeformableType(nh_) == DeformableType::ROPE)
    {
        rope_node_transforms_srv_.shutdown();
    }
    execute_gripper_movement_srv_.shutdown();
}


void CustomScene::makeBulletObjects()
{
    // Can be overriden inside each builder function
    object_color_map_[GENERIC_OBSTACLE] = ColorBuilder::MakeFromFloatColors(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 1.0f);
    object_color_map_[PEG]              = ColorBuilder::MakeFromFloatColors(100.0f/255.0f, 100.0f/255.0f, 100.0f/255.0f, 1.0f);

    ROS_INFO("Building the world");
    switch (task_type_)
    {
        case TaskType::ROPE_CYLINDER_COVERAGE:
            makeRope();
            makeRopeSingleRobotControlledGrippper();
            makeTableSurface(false);
            makeCylinder(true);
            break;

        case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
            makeRope();
            makeRopeTwoRobotControlledGrippers();
            makeTableSurface(false);
            makeCylinder(true);
            break;

        case TaskType::ROPE_MAZE:
            makeRope();
            makeRopeTwoRobotControlledGrippers();
            makeRopeMazeObstacles();
            break;

        case TaskType::ROPE_ZIG_MATCH:
            makeRope();
            makeRopeTwoRobotControlledGrippers();
            makeRopeZigMatchObstacles();
            break;

        case TaskType::CLOTH_CYLINDER_COVERAGE:
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeCylinder(true);
            break;

        case TaskType::CLOTH_TABLE_COVERAGE:
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeTableSurface(true, CLOTH_TABLE_COVERAGE_COVER_POINT_STEPSIZE);
            break;

        case TaskType::CLOTH_COLAB_FOLDING:
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeClothTwoHumanControlledGrippers();
            break;

        case TaskType::CLOTH_WAFR:
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeCylinder(true);
            break;

        case TaskType::CLOTH_SINGLE_POLE:
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeSinglePoleObstacles();
            break;

        case TaskType::CLOTH_WALL:
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeClothWallObstacles();
            makeGenericRegionCoverPoints();
            break;

        case TaskType::CLOTH_DOUBLE_SLIT:
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeClothDoubleSlitObstacles();
            break;

        case TaskType::ROPE_TABLE_LINEAR_MOTION:
        case TaskType::ROPE_TABLE_PENTRATION:
            makeRope();
            makeRopeSingleRobotControlledGrippper();
            makeTableSurface(false);
            break;

        case TaskType::CLOTH_TABLE_LINEAR_MOTION:
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeTableSurface(false);
            break;

        case TaskType::CLOTH_PLACEMAT_LIVE_ROBOT:
            // Creating the cloth just so that various other parts of the code have valid data to parse.
            // The cloth part of the simulation is not actually used for anything.
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeTableSurface(false);
            makeCylinder(false);
            loadCoverPointsFromFile();
            break;

        case TaskType::CLOTH_PLACEMAT_LINEAR_MOTION:
            // Creating the cloth just so that various other parts of the code have valid data to parse.
            // The cloth part of the simulation is not actually used for anything.
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeTableSurface(false);
            break;

        case TaskType::ROPE_HOOKS:
        case TaskType::ROPE_HOOKS_DATA_GENERATION:
            makeRope();
            makeRopeTwoRobotControlledGrippers();
            makeRopeHooksObstacles();
            break;

        case TaskType::CLOTH_HOOKS:
            makeCloth();
            makeClothTwoRobotControlledGrippers();
            makeClothHooksObstacles();
            break;

        default:
            ROS_FATAL_STREAM("Unknown task type #" << task_type_ << " " << GetTaskTypeString(nh_));
            throw_arc_exception(std::invalid_argument, "Unknown task type " + std::to_string(task_type_));
    }

    addGrippersAndAxesToWorld();
    makeCollisionCheckGripper();

    assert(cover_points_.size() == cover_point_normals_.size());
}

void CustomScene::makeRope()
{
    // find the needed table parameters
    const btVector3 rope_com =
            btVector3(GetRopeCenterOfMassX(nh_),
                      GetRopeCenterOfMassY(nh_),
                      GetRopeCenterOfMassZ(nh_)) * METERS;

    const btScalar rope_unit_vec_x = GetRopeExtensionVectorX(nh_);
    const btScalar rope_unit_vec_y = GetRopeExtensionVectorY(nh_);
    const btScalar rope_unit_vec_z = GetRopeExtensionVectorZ(nh_);
    const btVector3 rope_unit_vec(rope_unit_vec_x, rope_unit_vec_y, rope_unit_vec_z);

    const float rope_segment_length = GetRopeSegmentLength(nh_) * METERS;
    const size_t num_control_points = (size_t)GetRopeNumLinks(nh_) + 1;

    // make the rope
    std::vector<btVector3> control_points(num_control_points);
    for (size_t n = 0; n < num_control_points; n++)
    {
        control_points[n] = rope_com +
                ((btScalar)n - (btScalar)(num_control_points) / 2.0f) * rope_segment_length * rope_unit_vec;
    }

    rope_ = boost::make_shared<CapsuleRope>(control_points, GetRopeRadius(nh_) * METERS);

    // color the rope
    std::vector<BulletObject::Ptr> children = rope_->getChildren();
    for (size_t j = 0; j < children.size(); j++)
    {
        children[j]->setColor(0.15f, 0.65f, 0.15f, 1.0f);
    }

    // add the table and rope to the world
    env->add(rope_);
}

CapsuleRope::Ptr CustomScene::duplicateRopeAtInitialConfig() const
{
    assert(rope_ != nullptr);
    return boost::make_shared<CapsuleRope>(
                rope_->initalCtrlPoints,
                rope_->radius,
                rope_->angStiffness,
                rope_->angDamping,
                rope_->linDamping,
                rope_->angLimit);
}

void CustomScene::makeCloth()
{
    // Create the cloth itself
    {
        // cloth parameters
        const btVector3 cloth_center = btVector3(
                    GetClothCenterOfMassX(nh_),
                    GetClothCenterOfMassY(nh_),
                    GetClothCenterOfMassZ(nh_)) * METERS;

        const btScalar cloth_x_half_side_length = GetClothXSize(nh_) * METERS / 2.0f;
        const btScalar cloth_y_half_side_length = GetClothYSize(nh_) * METERS / 2.0f;

        btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
            env->bullet->softBodyWorldInfo,
            cloth_center + btVector3(-cloth_x_half_side_length, -cloth_y_half_side_length, 0),
            cloth_center + btVector3(+cloth_x_half_side_length, -cloth_y_half_side_length, 0),
            cloth_center + btVector3(-cloth_x_half_side_length, +cloth_y_half_side_length, 0),
            cloth_center + btVector3(+cloth_x_half_side_length, +cloth_y_half_side_length, 0),
            GetClothNumControlPointsX(nh_), GetClothNumControlPointsY(nh_),
            0, true);
        psb->setTotalMass(0.1f, true);

    //    psb->m_cfg.viterations = 10;     // Velocity solver iterations   - default 0 - changing this to 10 causes the cloth to just pass through objects it seems
        psb->m_cfg.piterations = 10;     // Positions solver iterations  - default 1 - DmitrySim 10
    //    psb->m_cfg.diterations = 10;     // Drift solver iterations      - default 0 - changing this to 10 or 100 doesn't seem to cause any meaningful change
        psb->m_cfg.citerations = 10;     // Cluster solver iterations    - default 4

        psb->m_cfg.collisions   =
                btSoftBody::fCollision::CL_SS |         // Cluster collisions, soft vs. soft
                btSoftBody::fCollision::CL_RS |         // Cluster collisions, soft vs. rigid
                btSoftBody::fCollision::CL_SELF;        // Cluster collisions, self - requires CL_SS

        psb->getCollisionShape()->setMargin(0.0025f * METERS); // default 0.25 - DmitrySim 0.05


        psb->m_cfg.kDP          = 0.05f;    // Damping coeffient [0, +inf]          - default 0
        psb->m_cfg.kDF          = 1.0f;     // Dynamic friction coefficient [0,1]   - default 0.2
    //    psb->m_cfg.kMT          = 0.0f;     // Pose matching coefficient [0,1]      - default 0
    //    psb->m_cfg.kKHR         = 1.0f;     // Kinetic contacts hardness [0,1]      - default 0.1


        btSoftBody::Material *pm = psb->m_materials[0];
        pm->m_kLST = GetClothLinearStiffness(ph_);     // Linear stiffness coefficient [0,1]       - default is 1 - 0.2 makes it rubbery (handles self collisions better)
    //    pm->m_kAST = 0.90f;     // Area/Angular stiffness coefficient [0,1] - default is 1
    //    pm->m_kVST = 1;        // Volume stiffness coefficient [0,1]       - default is 1
        const int distance = 2; // node radius for creating constraints
        psb->generateBendingConstraints(distance, pm);
        psb->randomizeConstraints();


        if (task_type_ == TaskType::CLOTH_TABLE_COVERAGE)
        {
            psb->generateClusters(0);
        }
        else
        {
            // splits the soft body volume up into the given number of small, convex clusters,
            // which consecutively will be used for collision detection with other soft bodies or rigid bodies.
            // Sending '0' causes the function to use the number of tetrahedral/face elemtents as the number of clusters
    //        psb->generateClusters(num_divs * num_divs / 25);
            psb->generateClusters(500);
            for (int i = 0; i < psb->m_clusters.size(); ++i)
            {
                psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.001f; // default 0.01
    //            psb->m_clusters[i]->m_maxSelfCollisionImpulse = 100.0f; // maximum self impulse that is *ignored (I think)* - default 100
            }
        }

        cloth_ = boost::make_shared<BulletSoftObject>(psb);
    }

    // note that we need to add the cloth to the environment before setting the
    // color, otherwise we get a segfault
    env->add(cloth_);
    cloth_->setColor(0.15f, 0.65f, 0.15f, 1.0f);

    // Add visualizations
    {
        findClothCornerNodes(cloth_->softBody.get(), cloth_corner_node_indices_);
        if (VisualizeStrainLines(ph_))
        {
            makeClothLines();
            makeGripperForceLines();
        }
    }
}



/*
 *  Creates a line for each link of a cloth
 */
void CustomScene::makeClothLines()
{
    strain_lines_ = boost::make_shared<PlotLines>(0.05f * METERS);
    max_strain_ = GetMaxStrain(ph_);
    
    addPreStepCallback(boost::bind(&CustomScene::updateClothLinesCallback, this));
    env->add(strain_lines_);
}

/*
 *  Updates all cloth strain lines with new positions and opacities
 */
void CustomScene::updateClothLinesCallback()
{
    const btSoftBody* psb = cloth_->softBody.get();
    const int num_links = psb->m_links.size();
    std::vector<btVector3> cloth_lines_endpoints;
    cloth_lines_endpoints.reserve(num_links * 2);
    std::vector<btVector4> cloth_strain_color;
    cloth_strain_color.reserve(num_links);
    std::vector<btScalar> strains = getStrain(psb);

    for(int i = 0; i < num_links; ++i)
    {
        const btSoftBody::Link& l = psb->m_links[i];
        cloth_lines_endpoints.push_back(l.m_n[0]->m_x);
        cloth_lines_endpoints.push_back(l.m_n[1]->m_x);

        btScalar relative_strain = strains[i]/max_strain_;
        btScalar alpha = std::min(std::max(relative_strain, 0.0f), 1.0f);
        btVector4 color(1.0f, 0.0f, 0.0f, alpha);
        cloth_strain_color.push_back(color);
    }
    
    strain_lines_->setPoints(cloth_lines_endpoints, cloth_strain_color);
}

void CustomScene::makeGripperForceLines()
{
    gripper_force_lines_ = boost::make_shared<PlotLines>(0.2f * METERS);
    
    addPreStepCallback(boost::bind(&CustomScene::updateGripperForceLinesCallback, this));
    env->add(gripper_force_lines_);
}

void CustomScene::updateGripperForceLinesCallback(){
    std::vector<btVector3> force_lines_endpoints;
    force_lines_endpoints.reserve(auto_grippers_.size()*2);
    
    for (const std::string &gripper_name: auto_grippers_)
    {
        const GripperKinematicObject::Ptr gripper = grippers_.at(gripper_name);
        btVector3 force = gripper->calculateSoftBodyForce();
        btVector3 gripper_pos = gripper->getWorldTransform().getOrigin();
        force_lines_endpoints.push_back(gripper_pos);
        force_lines_endpoints.push_back(gripper_pos - force);
    }
    std::vector<btVector4> gripper_force_color(auto_grippers_.size(), btVector4(0, 0, 1, 1));

    gripper_force_lines_->setPoints(force_lines_endpoints, gripper_force_color);
}


void CustomScene::createClothMirrorLine()
{
    const btSoftBody::tNodeArray cloth_nodes = cloth_->softBody->m_nodes;
    const btVector3& min_x_min_y = cloth_nodes[cloth_corner_node_indices_[0]].m_x;
//    const btVector3& min_x_max_y = cloth_nodes[cloth_corner_node_indices_[1]].m_x;
//    const btVector3& max_x_min_y = cloth_nodes[cloth_corner_node_indices_[2]].m_x;
    const btVector3& max_x_max_y = cloth_nodes[cloth_corner_node_indices_[3]].m_x;

    // TODO: Update mirror line data with non-bullet frames
    mirror_line_data_.header.frame_id = bullet_frame_name_;
    mirror_line_data_.header.stamp = ros::Time::now();
    mirror_line_data_.min_y = min_x_min_y.y() / METERS;
    mirror_line_data_.max_y = max_x_max_y.y() / METERS;
    mirror_line_data_.mid_x = (min_x_min_y.x() + (max_x_max_y.x() - min_x_min_y.x()) / 2) / METERS;

    std::vector<btVector3> mirror_line_points;
    mirror_line_points.push_back(btVector3((btScalar)mirror_line_data_.mid_x, (btScalar)mirror_line_data_.min_y, 0.8f) * METERS);
    mirror_line_points.push_back(btVector3((btScalar)mirror_line_data_.mid_x, (btScalar)mirror_line_data_.max_y, 0.8f) * METERS);
    std::vector<btVector4> mirror_line_colors;
    mirror_line_colors.push_back(btVector4(1,0,0,1));

    miror_line_vis_ = boost::make_shared<PlotLines>(0.1f * METERS);
    miror_line_vis_->setPoints(mirror_line_points, mirror_line_colors);
    env->add(miror_line_vis_);
}



void CustomScene::makeRopeSingleRobotControlledGrippper()
{
    // rope_gripper (the only)
    {
        const std::string gripper_name = "rope_gripper";

        // add a single auto gripper to the world
        grippers_[gripper_name] = boost::make_shared<GripperKinematicObject>(
                    env,
                    gripper_name,
                    GetGripperApperture(nh_) * METERS,
                    GRIPPER_COLOR);
        grippers_[gripper_name]->setWorldTransform(rope_->getChildren()[0]->rigidBody->getCenterOfMassTransform());
        grippers_[gripper_name]->rigidGrab(rope_->getChildren()[0]->rigidBody.get(), 0);

        auto_grippers_.push_back(gripper_name);
        // We don't add the gripper to the world here; that is done in addGrippersAndAxesToWorld()
    }
}

void CustomScene::makeRopeTwoRobotControlledGrippers()
{
    // rope_gripper0
    {
        const std::string gripper_name = "rope_gripper0";

        // add a single auto gripper to the world
        grippers_[gripper_name] = boost::make_shared<GripperKinematicObject>(
                    env,
                    gripper_name,
                    GetGripperApperture(nh_) * METERS,
                    GRIPPER_COLOR);

        const size_t object_node_ind = 0;
        grippers_[gripper_name]->setWorldTransform(rope_->getChildren()[object_node_ind]->rigidBody->getCenterOfMassTransform());
        grippers_[gripper_name]->rigidGrab(rope_->getChildren()[object_node_ind]->rigidBody.get(), object_node_ind);

        auto_grippers_.push_back(gripper_name);
        // We don't add the gripper to the world here; that is done in addGrippersAndAxesToWorld()
    }

    // rope_gripper1
    {
        const std::string gripper_name = "rope_gripper1";

        // add a single auto gripper to the world
        grippers_[gripper_name] = boost::make_shared<GripperKinematicObject>(
                    env,
                    gripper_name,
                    GetGripperApperture(nh_) * METERS,
                    GRIPPER_COLOR);

        const size_t object_node_ind = rope_->getChildren().size() - 1;
        grippers_[gripper_name]->setWorldTransform(rope_->getChildren()[object_node_ind]->rigidBody->getCenterOfMassTransform());
        grippers_[gripper_name]->rigidGrab(rope_->getChildren()[object_node_ind]->rigidBody.get(), object_node_ind);

        auto_grippers_.push_back(gripper_name);
        // We don't add the gripper to the world here; that is done in addGrippersAndAxesToWorld()
    }
}

void CustomScene::makeClothTwoRobotControlledGrippers()
{
    // auto gripper0
    const std::string auto_gripper0_name = GetGripper0Name(nh_);
    {
        grippers_[auto_gripper0_name] = boost::make_shared<GripperKinematicObject>(
                    env,
                    auto_gripper0_name,
                    GetGripperApperture(nh_) * METERS,
                    GRIPPER_COLOR);
        const btVector3 gripper_half_extents = grippers_[auto_gripper0_name]->getHalfExtents();

        // Retreive the node index to put the gripper on, defaulting to one of the corners if no parameter exists
        const int node_idx = ROSHelpers::GetParam<int>(nh_, auto_gripper0_name + "_gripper_attached_node_idx", cloth_corner_node_indices_[0]);
        grippers_[auto_gripper0_name]->setWorldTransform(
                btTransform(btQuaternion(0, 0, 0, 1),
                             cloth_->softBody->m_nodes[node_idx].m_x
                             + btVector3(gripper_half_extents.x(), gripper_half_extents.y(), 0)));

        // Grip the cloth
        grippers_[auto_gripper0_name]->toggleOpen();
        grippers_[auto_gripper0_name]->toggleAttach(cloth_->softBody.get());

        // Add a callback in case this gripper gets step_openclose activated on it
        // This is a state machine whose input comes from CustomKeyHandler
        addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, grippers_[auto_gripper0_name], cloth_->softBody.get()));

        auto_grippers_.push_back(auto_gripper0_name);
        // We don't add the gripper to the world here; that is done in addGrippersAndAxesToWorld()
    }

    // auto gripper1
    const std::string auto_gripper1_name = GetGripper1Name(nh_);
    {
        grippers_[auto_gripper1_name] = boost::make_shared<GripperKinematicObject>(
                    env,
                    auto_gripper1_name,
                    GetGripperApperture(nh_) * METERS,
                    GRIPPER_COLOR);
        const btVector3 gripper_half_extents = grippers_[auto_gripper1_name]->getHalfExtents();

        // Retreive the node index to put the gripper on, defaulting to one of the corners if no parameter exists
        const int node_idx = ROSHelpers::GetParam<int>(nh_, auto_gripper1_name + "_gripper_attached_node_idx", cloth_corner_node_indices_[1]);
        grippers_[auto_gripper1_name]->setWorldTransform(
                    btTransform(btQuaternion(0, 0, 0, 1),
                                cloth_->softBody->m_nodes[node_idx].m_x
                + btVector3(gripper_half_extents.x(), -gripper_half_extents.y(), 0)));

        // Grip the cloth
        grippers_[auto_gripper1_name]->toggleOpen();
        grippers_[auto_gripper1_name]->toggleAttach(cloth_->softBody.get());

        // Add a callback in case this gripper gets step_openclose activated on it
        // This is a state machine whose input comes from CustomKeyHandler
        addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, grippers_[auto_gripper1_name], cloth_->softBody.get()));

        auto_grippers_.push_back(auto_gripper1_name);
        // We don't add the gripper to the world here; that is done in addGrippersAndAxesToWorld()
    }

    // Set stretching detection vector infomation
    {
        grippers_[auto_gripper0_name]->setClothGeoInfoToAnotherGripper(
                    grippers_[auto_gripper1_name],
                    cloth_->softBody,
                    GetClothNumControlPointsX(nh_));

        grippers_[auto_gripper1_name]->setClothGeoInfoToAnotherGripper(
                    grippers_[auto_gripper0_name],
                    cloth_->softBody,
                    GetClothNumControlPointsX(nh_));
    }
}

void CustomScene::makeClothTwoHumanControlledGrippers()
{
    // manual gripper0
    {
        const std::string manual_gripper0_name = "manual_gripper0";
        grippers_[manual_gripper0_name] = boost::make_shared<GripperKinematicObject>(
                    env,
                    manual_gripper0_name,
                    GetGripperApperture(nh_) * METERS,
                    btVector4(0.6f, 0.6f, 0.6f, 0.4f));
        const btVector3 gripper_half_extents = grippers_[manual_gripper0_name]->getHalfExtents();
        grippers_[manual_gripper0_name]->setWorldTransform(
                    btTransform(btQuaternion(0, 0, 0, 1),
                                 cloth_->softBody->m_nodes[cloth_corner_node_indices_[2]].m_x
                + btVector3(-gripper_half_extents.x(), gripper_half_extents.y(), 0)));

        // Add a callback in case this gripper gets step_openclose activated on it
        // This is a state machine whose input comes from CustomKeyHandler
        addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, grippers_[manual_gripper0_name], cloth_->softBody.get()));

        manual_grippers_.push_back(manual_gripper0_name);
        manual_grippers_paths_.push_back(ManualGripperPath(grippers_[manual_gripper0_name], &gripperPath0));
        // We don't add the gripper to the world here; that is done in addGrippersAndAxesToWorld()
    }

    // manual gripper1
    {
        const std::string manual_gripper1_name = "manual_gripper1";
        grippers_[manual_gripper1_name] = boost::make_shared<GripperKinematicObject>(
                    env,
                    manual_gripper1_name,
                    GetGripperApperture(nh_) * METERS,
                    btVector4(0.6f, 0.6f, 0.6f, 0.4f));
        const btVector3 gripper_half_extents = grippers_[manual_gripper1_name]->getHalfExtents();
        grippers_[manual_gripper1_name]->setWorldTransform(
                    btTransform(btQuaternion(0, 0, 0, 1),
                                 cloth_->softBody->m_nodes[cloth_corner_node_indices_[3]].m_x
                + btVector3(-gripper_half_extents.x(), -gripper_half_extents.y(), 0)));

        // Add a callback in case this gripper gets step_openclose activated on it
        // This is a state machine whose input comes from CustomKeyHandler
        addPreStepCallback(boost::bind(&GripperKinematicObject::step_openclose, grippers_[manual_gripper1_name], cloth_->softBody.get()));

        manual_grippers_.push_back(manual_gripper1_name);
        manual_grippers_paths_.push_back(ManualGripperPath(grippers_[manual_gripper1_name], &gripperPath1));
        // We don't add the gripper to the world here; that is done in addGrippersAndAxesToWorld()
    }
}

void CustomScene::addGrippersAndAxesToWorld()
{
    const bool display_axes = ROSHelpers::GetParam<bool>(ph_, "display_grippers_axes", true);
    for (auto& gripper: grippers_)
    {
        env->add(gripper.second);
        if (display_axes)
        {
            gripper_axes_[gripper.first] = boost::make_shared<PlotAxes>();
            env->add(gripper_axes_[gripper.first]);
        }
    }
}

void CustomScene::makeCollisionCheckGripper()
{
    switch (deformable_type_)
    {
        case ROPE:
        // Add a collision check gripper that is in the same kinematic state as used for the rope experiments for collision checking
        {
            collision_check_gripper_ = boost::make_shared<GripperKinematicObject>(
                        env,
                        "collision_check_gripper",
                        GetGripperApperture(nh_) * METERS,
                        btVector4(1.0f, 0.0f, 0.0f, 0.0f));
            collision_check_gripper_->setWorldTransform(btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0)));
            // We don't want to add this to the world, because then it shows up as a object to collide with
            break;
        }

        case CLOTH:
        // Add a collision check gripper that is in the same kinematic state as used for the cloth experiments
        {
            collision_check_gripper_ = boost::make_shared<GripperKinematicObject>(
                        env,
                        "collision_check_gripper",
                        GetGripperApperture(nh_) * METERS,
                        btVector4(1.0f, 0.0f, 0.0f, 0.0f));
            collision_check_gripper_->setWorldTransform(btTransform(btQuaternion(0.0, 0.0, 0.0, 1.0)));
            collision_check_gripper_->toggleOpen();
            // We don't want to add this to the world, because then it shows up as a object to collide with
            break;
        }

        default:
        {
            ROS_FATAL_STREAM("Unknown deformable type " << deformable_type_);
            throw_arc_exception(std::invalid_argument, "Unknown deformable type " + std::to_string(deformable_type_));
        }
    }
}



void CustomScene::makeTableSurface(
        const bool create_cover_points,
        const float stepsize,
        const bool add_legs)
{
    // table parameters
    const float table_half_thickness = GetTableThickness(nh_) / 2.0f * METERS;

    const btVector3 table_surface_position =
            btVector3(GetTableSurfaceX(nh_),
                      GetTableSurfaceY(nh_),
                      GetTableSurfaceZ(nh_)) * METERS;

    const btVector3 table_half_extents =
            btVector3(GetTableHalfExtentsX(nh_),
                      GetTableHalfExtentsY(nh_),
                      GetTableHeight(nh_) / 2.0f) * METERS;

    // create the table
    TableKinematicObject::Ptr table =
            boost::make_shared<TableKinematicObject>(
                "table",
                btTransform(btQuaternion(0, 0, 0, 1), table_surface_position),
                table_half_extents * 2.0f,
                table_half_thickness * 2.0f,
                btVector4(0.4f, 0.4f, 0.4f, 1.0f),
                add_legs);

    env->add(table);
//    world_table_obstacles_["table"] = table;
    world_obstacles_["table_surface"] = table->getChildren()[0];
    if (add_legs)
    {
        world_obstacles_["table_leg1"] = table->getChildren()[1];
        world_obstacles_["table_leg2"] = table->getChildren()[2];
        world_obstacles_["table_leg3"] = table->getChildren()[3];
        world_obstacles_["table_leg4"] = table->getChildren()[4];
    }


    // if we are doing a table coverage task, create the table coverage points
    if (create_cover_points)
    {
        assert(stepsize > 0);
//        btTransform table_tf = table->rigidBody->getCenterOfMassTransform();

        const float num_x_steps = std::floor(table_half_extents.x() * 2.0f / stepsize);
        const float total_x_coverage = num_x_steps * stepsize;
        const float x_offset = table_half_extents.x() - (total_x_coverage / 2.0f);

        const float num_y_steps = std::floor(table_half_extents.y() * 2.0f / stepsize);
        const float total_y_coverage = num_y_steps * stepsize;
        const float y_offset = table_half_extents.y() - (total_y_coverage / 2.0f);

        const float cloth_collision_margin = cloth_->softBody->getCollisionShape()->getMargin();
        std::vector<btVector3> cloth_coverage_lines;
        for (float y = -table_half_extents.y() + y_offset; y <= table_half_extents.y(); y += stepsize)
        {
            // Add a coverage line to the visualization
            cloth_coverage_lines.push_back(
                        table_surface_position + btVector3 (-table_half_extents.x(), y, cloth_collision_margin * 2.0f));

            cloth_coverage_lines.push_back(
                        table_surface_position + btVector3 (+table_half_extents.x(), y, cloth_collision_margin * 2.0f));

            // Add many coverage points along the coverage line
            for(float x = x_offset - table_half_extents.x(); x <= table_half_extents.x(); x += stepsize)
            {
                cover_points_.push_back(
                            table_surface_position + btVector3(x, y, cloth_collision_margin * 2.0f));

                cover_point_normals_.push_back(btVector3(0.0f, 0.0f, 1.0f));
            }
        }
        ROS_INFO_STREAM("Number of cover points: " << cover_points_.size());

        std::vector<btVector4> cloth_coverage_color(cloth_coverage_lines.size(), btVector4(1, 0, 0, 1));
        plot_lines_->setPoints(cloth_coverage_lines, cloth_coverage_color);
        env->add(plot_lines_);
    }
}

void CustomScene::makeCylinder(const bool create_cover_points)
{
    // cylinder parameters
    const btVector3 cylinder_com_origin =
        btVector3(GetCylinderCenterOfMassX(nh_),
                  GetCylinderCenterOfMassY(nh_),
                  GetCylinderCenterOfMassZ(nh_)) * METERS;

    const btScalar cylinder_radius = GetCylinderRadius(nh_) * METERS;
    const btScalar cylinder_height = GetCylinderHeight(nh_) * METERS;

    // create a cylinder
    CylinderStaticObject::Ptr cylinder = boost::make_shared<CylinderStaticObject>(
                0, cylinder_radius, cylinder_height,
                btTransform(btQuaternion(0, 0, 0, 1), cylinder_com_origin));
    cylinder->setColor(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 0.5f);

    // add the cylinder to the world
    env->add(cylinder);
    world_obstacles_["cylinder"] = cylinder;

    if (create_cover_points)
    {
        switch (task_type_)
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            {
                for (float theta = 0; theta < 2.0f * M_PI; theta += ROPE_CYLINDER_ANGLE_DISCRETIZATION)
                {
                    // 31 points per theta
                    for (float h = -cylinder_height / 2.0f; h < cylinder_height / 2.0f; h += cylinder_height / ROPE_CYLINDER_HEIGHT_DISCRETIZATION_RATIO)
                    {
                        cover_points_.push_back(
                                cylinder_com_origin
                                + btVector3((cylinder_radius + rope_->radius / 2.0f) * std::cos(theta),
                                             (cylinder_radius + rope_->radius / 2.0f) * std::sin(theta),
                                             h));

                        cover_point_normals_.push_back(btVector3(std::cos(theta), std::sin(theta), 0.0f));
                    }
                }
                break;
            }

            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
            {
                for (float theta = 0; theta < 2.0f * M_PI; theta += ROPE_CYLINDER_ANGLE_DISCRETIZATION)
                {
                    for (float h = -cylinder_height / ROPE_CYLINDER_TWO_GRIPPERS_START_HEIGHT_RATIO;
                         h < cylinder_height / ROPE_CYLINDER_TWO_GRIPPERS_END_HEIGHT_RATIO;
                         h += cylinder_height / ROPE_CYLINDER_HEIGHT_DISCRETIZATION_RATIO)
                    {
                        cover_points_.push_back(
                                cylinder_com_origin
                                + btVector3((cylinder_radius + rope_->radius / 2.0f) * std::cos(theta),
                                             (cylinder_radius + rope_->radius / 2.0f) * std::sin(theta),
                                             h));

                        cover_point_normals_.push_back(btVector3(std::cos(theta), std::sin(theta), 0.0f));
                    }
                }
                break;
            }

            case TaskType::CLOTH_WAFR:
            {
                const float cloth_collision_margin = cloth_->softBody->getCollisionShape()->getMargin();

                for (float x = -cylinder_radius;
                     x <= cylinder_radius;
                     x += cylinder_radius / CLOTH_CYLINDER_ANGLE_DISCRETIZATION_RATIO)
                {
                    for (float y = -cylinder_radius;
                         y <= cylinder_radius;
                         y += cylinder_radius / CLOTH_CYLINDER_ANGLE_DISCRETIZATION_RATIO)
                    {
                        // Only accept those points that are within the bounding circle
                        if (x * x + y * y < cylinder_radius * cylinder_radius)
                        {
                            cover_points_.push_back(
                                        cylinder_com_origin
                                        + btVector3(x, y, cylinder_height / 2.0f + cloth_collision_margin));

                            cover_point_normals_.push_back(btVector3(0.0f, 0.0f, 1.0f));
                        }
                    }
                }

                ////////////////////////////////////////////////////////////////////////
                // Horizontal Cylinder above first cylinder
                ////////////////////////////////////////////////////////////////////////

                const btVector3 horizontal_cylinder_com_origin = cylinder_com_origin +
                        btVector3(GetWafrCylinderRelativeCenterOfMassX(nh_),
                                  GetWafrCylinderRelativeCenterOfMassY(nh_),
                                  GetWafrCylinderRelativeCenterOfMassZ(nh_)) * METERS;

                CylinderStaticObject::Ptr horizontal_cylinder = boost::make_shared<CylinderStaticObject>(
                            0.0f, GetWafrCylinderRadius(nh_) * METERS, GetWafrCylinderHeight(nh_) * METERS,
                            btTransform(btQuaternion(btVector3(1, 0, 0), (float)M_PI/2.0f), horizontal_cylinder_com_origin));
                horizontal_cylinder->setColor(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 0.5f);

                // add the cylinder to the world
                env->add(horizontal_cylinder);
                world_obstacles_["horizontal_cylinder"] = horizontal_cylinder;

                for (float theta = CLOTH_WAFR_HORIZONTAL_CYLINDER_START_ANGLE;
                     theta <= CLOTH_WAFR_HORIZONTAL_CYLINDER_END_ANGLE;
                     theta += CLOTH_WAFR_HORIZONTAL_CYLINDER_ANGLE_DISCRETIZATION)
                {
                    const float cover_points_radius =
                            horizontal_cylinder->getRadius()
                            + cloth_collision_margin
                            + (btScalar)GetRobotMinGripperDistanceToObstacles() * METERS;

                    for (float h = -horizontal_cylinder->getHeight()/2.0f;
                         h <= horizontal_cylinder->getHeight()/1.99f;
                         h += horizontal_cylinder->getHeight() / CLOTH_WAFR_HORIZONTAL_CYLINDER_HEIGHT_DISCRETIZATION_RATIO)
                    {
                        cover_points_.push_back(
                                    horizontal_cylinder_com_origin
                                    + btVector3(
                                        cover_points_radius * std::sin(theta),
                                        h,
                                        cover_points_radius * std::cos(theta)));

                        cover_point_normals_.push_back(btVector3(std::sin(theta), 0.0f, std::cos(theta)));
                    }
                }
                break;
            }

            default:
                    ROS_FATAL_NAMED("deform_simulator", "Making cylinder cover points for unknown task, this should not be possible");
                    assert(false);
        }

        std::vector<btVector4> coverage_color(cover_points_.size(), btVector4(1, 0, 0, 1));
        plot_points_->setPoints(cover_points_, coverage_color);
        env->add(plot_points_);
    }
}

void CustomScene::makeSinglePoleObstacles()
{
    makeTableSurface(true, CLOTH_SINGLE_POLE_DOUBLE_SLIT_COVER_POINT_STEPSIZE, true);

    const btVector4 wall_color(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 1.0f);
    const btVector4 table_color(0.4f, 0.4f, 0.4f, 1.0f);

    const float table_half_thickness = GetTableThickness(nh_) / 2.0f * METERS;

    const btVector3 table_surface_position =
            btVector3(GetTableSurfaceX(nh_),
                      GetTableSurfaceY(nh_),
                      GetTableSurfaceZ(nh_)) * METERS;

    const btVector3 table_half_extents =
            btVector3(GetTableHalfExtentsX(nh_),
                      GetTableHalfExtentsY(nh_),
                      GetTableHeight(nh_) / 2.0f) * METERS;


    // cylinder parameters
    const btVector3 cylinder_com_origin =
        btVector3(GetCylinderCenterOfMassX(nh_),
                  GetCylinderCenterOfMassY(nh_),
                  GetCylinderCenterOfMassZ(nh_)) * METERS;

    const btScalar cylinder_radius = GetCylinderRadius(nh_) * METERS;
    const btScalar cylinder_height = GetCylinderHeight(nh_) * METERS;

    // Make the bottom floor to ensure that the free space graph doesn't go down through the floor due to rounding
    {
        const btVector3 floor_half_extents = btVector3(75.0f, 75.0f, 5.0f);
        const btVector3 floor_com = btVector3(0.0f, 0.0f, -floor_half_extents.z());

        BoxObject::Ptr floor = boost::make_shared<BoxObject>(
                    0, floor_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), floor_com));
        floor->setColor(FLOOR_COLOR);

        // add the wall to the world
        env->add(floor);
        world_obstacles_["bottom_floor"] = floor;
    }

    // create a cylinder directly in front of the table
    {
        CylinderStaticObject::Ptr cylinder = boost::make_shared<CylinderStaticObject>(
                    0, cylinder_radius, cylinder_height,
                    btTransform(btQuaternion(0, 0, 0, 1), cylinder_com_origin));
        cylinder->setColor(wall_color);

        // add the cylinder to the world
        env->add(cylinder);
        world_obstacles_["center_cylinder"] = cylinder;
     }

    // create a cylinder directly to the left of the table (left from cloth starting position)
    {
        CylinderStaticObject::Ptr cylinder = boost::make_shared<CylinderStaticObject>(
                    0, cylinder_radius, cylinder_height,
                    btTransform(btQuaternion(0, 0, 0, 1), cylinder_com_origin + btVector3(0.0f, -0.6f * METERS, 0.0f)));
        cylinder->setColor(wall_color);

        // add the cylinder to the world
        env->add(cylinder);
        world_obstacles_["left_cylinder"] = cylinder;
    }

    // create a cylinder directly to the right of the table (right from cloth starting position)
    {
        CylinderStaticObject::Ptr cylinder = boost::make_shared<CylinderStaticObject>(
                    0, cylinder_radius, cylinder_height,
                    btTransform(btQuaternion(0, 0, 0, 1), cylinder_com_origin + btVector3(0.0f, 0.6f * METERS, 0.0f)));
        cylinder->setColor(wall_color);

        // add the cylinder to the world
        env->add(cylinder);
        world_obstacles_["left_cylinder"] = cylinder;
    }

    // create the left table
    {
        TableKinematicObject::Ptr table =
                boost::make_shared<TableKinematicObject>(
                    "left_table",
                    btTransform(btQuaternion(0, 0, 0, 1), table_surface_position + btVector3(0.0f, -0.6f * METERS, 0.0f)),
                    table_half_extents * 2.0f,
                    table_half_thickness * 2.0f,
                    table_color,
                    true);

        env->add(table);
        world_obstacles_["left_table_surface"] = table->getChildren()[0];
        world_obstacles_["left_table_leg1"] = table->getChildren()[1];
        world_obstacles_["left_table_leg2"] = table->getChildren()[2];
        world_obstacles_["left_table_leg3"] = table->getChildren()[3];
        world_obstacles_["left_table_leg4"] = table->getChildren()[4];
    }

    // create the right table
    {
        TableKinematicObject::Ptr table =
                boost::make_shared<TableKinematicObject>(
                    "right_table",
                    btTransform(btQuaternion(0, 0, 0, 1), table_surface_position + btVector3(0.0f, 0.6f * METERS, 0.0f)),
                    table_half_extents * 2.0f,
                    table_half_thickness * 2.0f,
                    table_color,
                    true);

        env->add(table);
        world_obstacles_["right_table_surface"] = table->getChildren()[0];
        world_obstacles_["right_table_leg1"] = table->getChildren()[1];
        world_obstacles_["right_table_leg2"] = table->getChildren()[2];
        world_obstacles_["right_table_leg3"] = table->getChildren()[3];
        world_obstacles_["right_table_leg4"] = table->getChildren()[4];
    }
}

void CustomScene::makeClothWallObstacles()
{
    // Cylinder parameters
    const btVector3 cylinder_com_origin =
        btVector3(GetCylinderCenterOfMassX(nh_),
                  GetCylinderCenterOfMassY(nh_),
                  GetCylinderCenterOfMassZ(nh_)) * METERS;

    const btScalar cylinder_radius = GetCylinderRadius(nh_) * METERS;
    const btScalar cylinder_height = GetCylinderHeight(nh_) * METERS;

    // Make the bottom floor to ensure that the free space graph doesn't go down through the floor due to rounding
    {
        const btVector3 floor_half_extents = btVector3(75.0f, 75.0f, 5.0f);
        const btVector3 floor_com = btVector3(0.0f, 0.0f, -floor_half_extents.z());

        BoxObject::Ptr floor = boost::make_shared<BoxObject>(
                    0, floor_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), floor_com));
        floor->setColor(FLOOR_COLOR);

        // add the wall to the world
        env->add(floor);
        world_obstacles_["bottom_floor"] = floor;
    }

    // create a cylinder
    {
        CylinderStaticObject::Ptr cylinder = boost::make_shared<CylinderStaticObject>(
                    0, cylinder_radius, cylinder_height,
                    btTransform(btQuaternion(0, 0, 0, 1), cylinder_com_origin));
        cylinder->setColor(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 0.5f);

        // add the cylinder to the world
        env->add(cylinder);
        world_obstacles_["cylinder"] = cylinder;
    }

    // Wall parameters
    {
        const btVector3 wall_half_extents(cylinder_radius, 0.5f * METERS, cylinder_height / 2.0f);
        const btVector3 wall_com = cylinder_com_origin - btVector3(0, wall_half_extents.y(), 0);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 0.5f);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["wall"] = wall;
    }
}

void CustomScene::makeClothDoubleSlitObstacles()
{
    makeTableSurface(true, CLOTH_SINGLE_POLE_DOUBLE_SLIT_COVER_POINT_STEPSIZE, true);

    const btVector4 wall_color(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 1.0f);
    const btVector4 table_color(0.4f, 0.4f, 0.4f, 1.0f);

    const float table_half_thickness = GetTableThickness(nh_) / 2.0f * METERS;

    const btVector3 table_surface_position =
            btVector3(GetTableSurfaceX(nh_),
                      GetTableSurfaceY(nh_),
                      GetTableSurfaceZ(nh_)) * METERS;

    const btVector3 table_half_extents =
            btVector3(GetTableHalfExtentsX(nh_),
                      GetTableHalfExtentsY(nh_),
                      GetTableHeight(nh_) / 2.0f) * METERS;

    const btVector3 wall_section_half_extents =
            btVector3(0.04f, 0.115f, GetWallHeight(nh_) / 2.0f) * METERS;

    const btVector3 center_wall_section_com =
            btVector3(0.0f, 0.0f, GetWallCenterOfMassZ(nh_)) * METERS;

    const btVector3 outside_wall_offset =
            btVector3(0.0f, 0.3f, 0.0f) * METERS;

    // Make the bottom floor to ensure that the free space graph doesn't go down through the floor due to rounding
    {
        const btVector3 floor_half_extents = btVector3(75.0f, 75.0f, 5.0f);
        const btVector3 floor_com = btVector3(0.0f, 0.0f, -floor_half_extents.z());

        BoxObject::Ptr floor = boost::make_shared<BoxObject>(
                    0, floor_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), floor_com));
        floor->setColor(FLOOR_COLOR);

        // add the wall to the world
        env->add(floor);
        world_obstacles_["bottom_floor"] = floor;
    }

    // Center wall
    {
        BoxObject::Ptr center_wall = boost::make_shared<BoxObject>(
                    0, wall_section_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), center_wall_section_com));
        center_wall->setColor(wall_color);

        // add the wall to the world
        env->add(center_wall);
        world_obstacles_["center_wall"] = center_wall;
    }

    // Left wall
    {
        BoxObject::Ptr left_wall = boost::make_shared<BoxObject>(
                    0, wall_section_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), center_wall_section_com + outside_wall_offset));
        left_wall->setColor(wall_color);

        // add the wall to the world
        env->add(left_wall);
        world_obstacles_["left_wall"] = left_wall;
    }

    // Right wall
    {
        BoxObject::Ptr right_wall = boost::make_shared<BoxObject>(
                    0, wall_section_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), center_wall_section_com - outside_wall_offset));
        right_wall->setColor(wall_color);

        // add the wall to the world
        env->add(right_wall);
        world_obstacles_["right_wall"] = right_wall;
    }

    // Far left wall
    {
        BoxObject::Ptr far_left_wall = boost::make_shared<BoxObject>(
                    0, wall_section_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), center_wall_section_com + 2.0f * outside_wall_offset));
        far_left_wall->setColor(wall_color);

        // add the wall to the world
        env->add(far_left_wall);
        world_obstacles_["far_left_wall"] = far_left_wall;
    }

    // Far right wall
    {
        BoxObject::Ptr far_right_wall = boost::make_shared<BoxObject>(
                    0, wall_section_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), center_wall_section_com - 2.0f * outside_wall_offset));
        far_right_wall->setColor(wall_color);

        // add the wall to the world
        env->add(far_right_wall);
        world_obstacles_["far_right_wall"] = far_right_wall;
    }

    // create the left table
    {
        TableKinematicObject::Ptr table =
                boost::make_shared<TableKinematicObject>(
                    "left_table",
                    btTransform(btQuaternion(0, 0, 0, 1), table_surface_position + btVector3(0.0f, -0.6f * METERS, 0.0f)),
                    table_half_extents * 2.0f,
                    table_half_thickness * 2.0f,
                    table_color,
                    true);

        env->add(table);
//        world_table_obstacles_["left_table"] = table;
        world_obstacles_["left_table_surface"] = table->getChildren()[0];
        world_obstacles_["left_table_leg1"] = table->getChildren()[1];
        world_obstacles_["left_table_leg2"] = table->getChildren()[2];
        world_obstacles_["left_table_leg3"] = table->getChildren()[3];
        world_obstacles_["left_table_leg4"] = table->getChildren()[4];
    }

    // create the right table
    {
        TableKinematicObject::Ptr table =
                boost::make_shared<TableKinematicObject>(
                    "right_table",
                    btTransform(btQuaternion(0, 0, 0, 1), table_surface_position + btVector3(0.0f, 0.6f * METERS, 0.0f)),
                    table_half_extents * 2.0f,
                    table_half_thickness * 2.0f,
                    table_color,
                    true);

        env->add(table);
//        world_table_obstacles_["right_table"] = table;
        world_obstacles_["right_table_surface"] = table->getChildren()[0];
        world_obstacles_["right_table_leg1"] = table->getChildren()[1];
        world_obstacles_["right_table_leg2"] = table->getChildren()[2];
        world_obstacles_["right_table_leg3"] = table->getChildren()[3];
        world_obstacles_["right_table_leg4"] = table->getChildren()[4];
    }
}

void CustomScene::makeRopeMazeObstacles()
{
    const btVector3 world_min = btVector3(
                (btScalar)GetWorldXMinBulletFrame(nh_),
                (btScalar)GetWorldYMinBulletFrame(nh_),
                (btScalar)GetWorldZMinBulletFrame(nh_)) * METERS;

    const btVector3 world_max = btVector3(
                (btScalar)GetWorldXMaxBulletFrame(nh_),
                (btScalar)GetWorldYMaxBulletFrame(nh_),
                (btScalar)GetWorldZMaxBulletFrame(nh_)) * METERS;

    const float wall_thickness = (btScalar)GetWorldResolution(nh_) * 4.0f * METERS;
    const btVector3 world_center = (world_max + world_min) / 2.0f;
    const btVector3 world_size = world_max - world_min;
    const btVector3 first_floor_center = world_center - btVector3(0.0f, 0.0f, world_size.z() + wall_thickness) / 4.0f;
    const btVector3 second_floor_center = world_center + btVector3(0.0f, 0.0f, world_size.z() + wall_thickness) / 4.0f;
    const float internal_wall_height = (world_size.z() - wall_thickness) / 2.0f;


    const float outer_walls_alpha = GetOuterWallsAlpha(ph_);
    const float floor_divider_alpha = GetFloorDividerAlpha(ph_);
    const float first_floor_alpha = GetFirstFloorAlpha(ph_);
    const float second_floor_alpha = GetSecondFloorAlpha(ph_);
    const btVector4 outer_walls_color(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, outer_walls_alpha);      // grayish
    const btVector4 floor_divider_color(165.0f/255.0f, 42.0f/255.0f, 42.0f/255.0f, floor_divider_alpha);    // brown
    const btVector4 first_floor_color(148.0f/255.0f, 0.0f/255.0f, 211.0f/255.0f, first_floor_alpha);        // purple
    const btVector4 second_floor_color(0.0f/255.0f, 128.0f/255.0f, 128.0f/255.0f, second_floor_alpha);      // teal

    // Make the bottom floor to ensure that the free space graph doesn't go down through the floor due to rounding
    {
        const btVector3 floor_half_extents = btVector3(world_size.x() + wall_thickness, world_size.y() + wall_thickness, wall_thickness) / 2.0f;
        const btVector3 floor_com = btVector3(world_center.x(), world_center.y(), world_min.z() - wall_thickness / 2.0f);

        BoxObject::Ptr floor = boost::make_shared<BoxObject>(
                    0, floor_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), floor_com));
        floor->setColor(0.0f, 0.0f, 0.0f, 0.0f);

        // add the wall to the world
        env->add(floor);
        world_obstacles_["bottom_floor"] = floor;
    }

    // Make the outer walls
    {
        const btVector3 wall_half_extents = btVector3(world_size.x() + wall_thickness, wall_thickness, world_size.z()) / 2.0f;
        const btVector3 wall_com = btVector3(world_center.x(), world_min.y(), world_center.z());

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(outer_walls_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["outer_wall0"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(world_size.x() + wall_thickness, wall_thickness, world_size.z()) / 2.0f;
        const btVector3 wall_com = btVector3(world_center.x(), world_max.y(), world_center.z());

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(outer_walls_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["outer_wall1"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(wall_thickness, world_size.y() - wall_thickness, world_size.z()) / 2.0f;
        const btVector3 wall_com = btVector3(world_min.x(), world_center.y(), world_center.z());

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(outer_walls_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["outer_wall2"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(wall_thickness, world_size.y() - wall_thickness, world_size.z()) / 2.0f;
        const btVector3 wall_com = btVector3(world_max.x(), world_center.y(), world_center.z());

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(outer_walls_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["outer_wall3"] = wall;
    }

    // Make 1st floor obstacles
    {
        const btVector3 wall_half_extents = btVector3(wall_thickness, 0.6f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = first_floor_center + btVector3(-0.7f * METERS, 0.3f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(first_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["first_floor_obstacle0"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(0.9f * METERS, wall_thickness, internal_wall_height) / 2.0f;
        const btVector3 wall_com = first_floor_center + btVector3(-0.15f * METERS, 0.1f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(first_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["first_floor_obstacle1"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(wall_thickness, 0.3f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = first_floor_center + btVector3(-0.3f * METERS, 0.55f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(first_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["first_floor_obstacle2"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(0.4f * METERS, wall_thickness, internal_wall_height) / 2.0f;
        const btVector3 wall_com = first_floor_center + btVector3(0.3f * METERS, 0.5f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(first_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["first_floor_obstacle3"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(wall_thickness, 0.6f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = first_floor_center + btVector3(0.8f * METERS, 0.4f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(first_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["first_floor_obstacle4"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(0.9f * METERS, wall_thickness, internal_wall_height) / 2.0f;
        const btVector3 wall_com = first_floor_center + btVector3(0.75f * METERS, -0.2f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(first_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["first_floor_obstacle5"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(wall_thickness, 0.7f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = first_floor_center + btVector3(0.0f * METERS, -0.55f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(first_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["first_floor_obstacle6"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(0.7f * METERS, wall_thickness, internal_wall_height) / 2.0f;
        const btVector3 wall_com = first_floor_center + btVector3(-0.85f * METERS, -0.4f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(first_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["first_floor_obstacle7"] = wall;
    }

    // Make the divider between floors
    {
        const btVector3 divider_half_extents = btVector3(world_size.x() - wall_thickness, world_size.y() - wall_thickness - 0.4f * METERS, wall_thickness) / 2.0f;
        const btVector3 divider_com = world_center + btVector3(0.0f, 0.2f * METERS, 0.0f);

        BoxObject::Ptr divider = boost::make_shared<BoxObject>(
                    0, divider_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), divider_com));
        divider->setColor(floor_divider_color);

        // add the wall to the world
        env->add(divider);
        world_obstacles_["horizontal_divider0"] = divider;
    }
    {
        const btVector3 divider_half_extents = btVector3(1.6f * METERS, 0.4f * METERS, wall_thickness) / 2.0f;
        const btVector3 divider_com = world_center + btVector3(0.0f, -0.7f * METERS, 0.0f);

        BoxObject::Ptr divider = boost::make_shared<BoxObject>(
                    0, divider_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), divider_com));
        divider->setColor(floor_divider_color);

        // add the wall to the world
        env->add(divider);
        world_obstacles_["horizontal_divider1"] = divider;
    }

    // Make 2nd floor obstacles
    {
        const btVector3 wall_half_extents = btVector3(0.4f * METERS, wall_thickness, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(-0.2f * METERS, 0.3f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["second_floor_obstacle0"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(0.4f * METERS, wall_thickness, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(-0.8f * METERS, 0.3f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["second_floor_obstacle1"] = wall;
    }    
    {
        const btVector3 wall_half_extents = btVector3(0.6f * METERS, wall_thickness, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(-0.5f * METERS, -0.1f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["second_floor_obstacle2"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(0.7f * METERS, wall_thickness, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(0.45f * METERS, -0.5f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["second_floor_obstacle3"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(wall_thickness, 0.5f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(-0.7f * METERS, -0.45f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["second_floor_obstacle4"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(wall_thickness, 0.4f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(-0.2f * METERS, -0.7f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["second_floor_obstacle5"] = wall;
    }

    // Make the goal region
    {
        const btVector3 wall_half_extents = btVector3(0.8f * METERS, wall_thickness, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(0.8f * METERS, 0.0f, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["goal_border_wall0"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(wall_thickness, 0.8f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(0.1f * METERS, 0.3f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["goal_border_wall1"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(0.2f * METERS, 0.2f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(0.9f * METERS, 0.65f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["goal_obstacle0"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(0.2f * METERS, 0.2f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(0.5f * METERS, 0.35f * METERS, 0.0f);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["goal_obstacle1"] = wall;
    }

    // Create the target points
    const float rope_segment_length = (btScalar)GetRopeSegmentLength(nh_) * METERS;
    const float rope_radius = GetRopeRadius(nh_) * METERS;
    const int num_rope_links = GetRopeNumLinks(nh_);
    assert(num_rope_links % 2 == 1);

    const int num_vertical_per_side = std::min<int>(14, (num_rope_links - 1) / 2);
    const int num_horizontal_per_side = (num_rope_links - (num_vertical_per_side * 2) - 1) / 2;

    const btVector3 rope_center =
            world_center +
            btVector3(0.7f * METERS, 0.5f * METERS, 0.0f) +                  // Center in the goal region (x,y)
            btVector3(0.0f,          0.0f,          wall_thickness / 2.0f) + // Move up out of the floor
            btVector3(0.0f,          0.0f,          rope_radius);            // Move up so that the target points are off the floor just enough

    const btVector3 middle_unit_vector = btVector3(0.1f * METERS, rope_segment_length * (float)(num_vertical_per_side - 1), 0.0f).normalized();

    // Create the part in y (middle visually on start)
    {
        for (int cover_idx = -num_vertical_per_side; cover_idx <= num_vertical_per_side; ++cover_idx)
        {
            const btVector3 target_point = rope_center + middle_unit_vector * (float)cover_idx * rope_segment_length;
            cover_points_.push_back(target_point);
        }
    }
    // Create the first part in x (towards the middle of the maze visually on start)
    {
        const btVector3 start_point = cover_points_.back();
        for (int cover_idx = 0; cover_idx < num_horizontal_per_side; ++cover_idx)
        {
            const btVector3 target_point = start_point + btVector3(rope_segment_length, 0.0f, 0.0f) * (float)(cover_idx + 1);
            cover_points_.push_back(target_point);
        }
    }
    // Create the second part in x (top right corner visually on start)
    {
        const btVector3 start_point = cover_points_.front();
        for (int cover_idx = 0; cover_idx < num_horizontal_per_side; ++cover_idx)
        {
            const btVector3 target_point = start_point - btVector3(rope_segment_length, 0.0f, 0.0f) * (float)(cover_idx + 1);
            cover_points_.insert(cover_points_.begin(), target_point);
        }
    }

    // Set the cover point normals to all be pointing in positive Z
    cover_point_normals_ = std::vector<btVector3>(cover_points_.size(), btVector3(0.0f, 0.0f, 1.0f));


    std::vector<btVector4> coverage_color(cover_points_.size(), btVector4(1.0f, 0.0f, 0.0f, 1.0f));
    plot_points_->setPoints(cover_points_, coverage_color);
    env->add(plot_points_);

    assert((int)cover_points_.size() == num_rope_links);
    ROS_INFO_STREAM("Num cover points: " << cover_points_.size());
}

void CustomScene::makeRopeZigMatchObstacles()
{
    const btVector3 world_min = btVector3(
                (btScalar)GetWorldXMinBulletFrame(nh_),
                (btScalar)GetWorldYMinBulletFrame(nh_),
                (btScalar)GetWorldZMinBulletFrame(nh_)) * METERS;

    const btVector3 world_max = btVector3(
                (btScalar)GetWorldXMaxBulletFrame(nh_),
                (btScalar)GetWorldYMaxBulletFrame(nh_),
                (btScalar)GetWorldZMaxBulletFrame(nh_)) * METERS;

    const float wall_thickness = 0.2f * METERS;
    const btVector3 world_center = (world_max + world_min) / 2.0f;
    const btVector3 world_size = world_max - world_min;
    const btVector3 second_floor_center = world_center + btVector3(0.0f, 0.0f, world_size.z() + wall_thickness) / 4.0f;
    const float internal_wall_height = (world_size.z() - wall_thickness) / 2.0f;

    const float second_floor_alpha = GetSecondFloorAlpha(ph_);
    const btVector4 second_floor_color(0.0f/255.0f, 128.0f/255.0f, 128.0f/255.0f, second_floor_alpha);      // teal

    // Make the goal region    
    {
        const btVector3 wall_half_extents = btVector3(0.2f * METERS, 0.2f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(0.9f * METERS, 0.65f * METERS, 0.0f * METERS);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["goal_obstacle0"] = wall;
    }
    {
        const btVector3 wall_half_extents = btVector3(0.2f * METERS, 0.2f * METERS, internal_wall_height) / 2.0f;
        const btVector3 wall_com = second_floor_center + btVector3(0.5f * METERS, 0.35f * METERS, 0.0f * METERS);

        BoxObject::Ptr wall = boost::make_shared<BoxObject>(
                    0, wall_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), wall_com));
        wall->setColor(second_floor_color);

        // add the wall to the world
        env->add(wall);
        world_obstacles_["goal_obstacle1"] = wall;
    }

    // Create the target points
    const float rope_segment_length = GetRopeSegmentLength(nh_) * METERS;
    const float rope_radius = GetRopeRadius(nh_) * METERS;
    const int num_rope_links = GetRopeNumLinks(nh_);
    assert(num_rope_links % 2 == 1);

    const int num_vertical_per_side = std::min<int>(14, (num_rope_links - 1) / 2);
    const int num_horizontal_per_side = (num_rope_links - (num_vertical_per_side * 2) - 1) / 2;

    const btVector3 rope_center =
            world_center +
            btVector3(0.7f * METERS, 0.5f * METERS, 0.0f) +                         // Center in the goal region (x,y)
            btVector3(0.0f,          0.0f,          internal_wall_height / 2.0f) +  // Move up out of the floor
            btVector3(0.0f,          0.0f,          rope_radius);                   // Move up so that the target points are off the floor just enough

    const btVector3 middle_unit_vector = btVector3(0.1f * METERS, rope_segment_length * (float)(num_vertical_per_side - 1), 0.0f).normalized();

    // Create the part in y (middle visually on start)
    {
        for (int cover_idx = -num_vertical_per_side; cover_idx <= num_vertical_per_side; ++cover_idx)
        {
            const btVector3 target_point = rope_center + middle_unit_vector * (float)cover_idx * rope_segment_length;
            cover_points_.push_back(target_point);
        }
    }
    // Create the first part in x (towards the middle of the maze visually on start)
    {
        const btVector3 start_point = cover_points_.back();
        for (int cover_idx = 0; cover_idx < num_horizontal_per_side; ++cover_idx)
        {
            const btVector3 target_point = start_point + btVector3(rope_segment_length, 0.0f, 0.0f) * (float)(cover_idx + 1);
            cover_points_.push_back(target_point);
        }
    }
    // Create the second part in x (top right corner visually on start)
    {
        const btVector3 start_point = cover_points_.front();
        for (int cover_idx = 0; cover_idx < num_horizontal_per_side; ++cover_idx)
        {
            const btVector3 target_point = start_point - btVector3(rope_segment_length, 0.0f, 0.0f) * (float)(cover_idx + 1);
            cover_points_.insert(cover_points_.begin(), target_point);
        }
    }

    // Set the cover point normals to all be pointing in positive Z
    cover_point_normals_ = std::vector<btVector3>(cover_points_.size(), btVector3(0.0f, 0.0f, 1.0f));

    std::vector<btVector4> coverage_color(cover_points_.size(), btVector4(1.0f, 0.0f, 0.0f, 1.0f));
    plot_points_->setPoints(cover_points_, coverage_color);
    env->add(plot_points_);

    assert((int)cover_points_.size() == num_rope_links);
    ROS_INFO_STREAM("Num cover points: " << cover_points_.size());
}

void CustomScene::makeRopeHooksObstacles()
{
    const btVector3 world_min = btVector3(
                (btScalar)GetWorldXMinBulletFrame(nh_),
                (btScalar)GetWorldYMinBulletFrame(nh_),
                (btScalar)GetWorldZMinBulletFrame(nh_)) * METERS;

    const btVector3 world_max = btVector3(
                (btScalar)GetWorldXMaxBulletFrame(nh_),
                (btScalar)GetWorldYMaxBulletFrame(nh_),
                (btScalar)GetWorldZMaxBulletFrame(nh_)) * METERS;

    const float wall_thickness = ROSHelpers::GetParamRequired<btScalar>(nh_, "wall_thickness", __func__).GetImmutable() * METERS;
    const btVector3 world_center = (world_max + world_min) / 2.0f;
    const btVector3 world_size = world_max - world_min;

    const btVector4 obstacles_color(148.0f/255.0f, 0.0f/255.0f, 211.0f/255.0f, 1.0f);                       // purple
    const btVector4 pole_color(0.0f/255.0f, 128.0f/255.0f, 128.0f/255.0f, 1.0f);                            // teal
    const btVector4 hook_color(0.0f/255.0f, 128.0f/255.0f, 128.0f/255.0f, 1.0f);                            // teal
    object_color_map_[INITIAL_OBSTACLE] = ColorBuilder::MakeFromFloatColors(pole_color.x(), pole_color.y(), pole_color.x(), pole_color.w());
    object_color_map_[HOOK] = ColorBuilder::MakeFromFloatColors(pole_color.x(), pole_color.y(), pole_color.x(), pole_color.w());
    object_color_map_[LOWER_OBSTACLES] = object_color_map_[GENERIC_OBSTACLE];
    object_color_map_[UPPER_OBSTACLES] = object_color_map_[GENERIC_OBSTACLE];

    // Make the floor to ensure that the free space graph doesn't go down through the floor due to rounding
    {
        const btVector3 floor_half_extents = btVector3(
                    world_size.x(),
                    world_size.y(),
                    wall_thickness) / 2.0f;
        const btVector3 floor_com = btVector3(
                    world_center.x(),
                    world_center.y(),
                    world_min.z() - wall_thickness / 2.0f);

        BoxObject::Ptr floor = boost::make_shared<BoxObject>(
                    0, floor_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), floor_com));
        floor->setColor(179.0f/255.0f, 176.0f/255.0f, 160.0f/255.0f, 1.0f);

        // add the wall to the world
        env->add(floor);
        world_obstacles_["floor"] = floor;
    }

    // Make the ceiling to ensure that the free space graph doesn't go down through the floor due to rounding
    {
        const btVector3 ceiling_half_extents = btVector3(world_size.x() + wall_thickness, world_size.y() + wall_thickness, wall_thickness) / 2.0f;
        const btVector3 ceiling_com = btVector3(world_center.x(), world_center.y(), world_max.z() + wall_thickness / 2.0f);

        BoxObject::Ptr ceiling = boost::make_shared<BoxObject>(
                    0, ceiling_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), ceiling_com));
        ceiling->setColor(0.0f, 0.0f, 0.0f, 0.0f);

        // add the wall to the world
        env->add(ceiling);
        world_obstacles_["ceiling"] = ceiling;
    }

    // Make the initial obstacle to go around
    {
        // obstacle parameters
        const btVector3 obstacle_com =
            btVector3(GetCylinderCenterOfMassX(nh_),
                      GetCylinderCenterOfMassY(nh_),
                      GetCylinderCenterOfMassZ(nh_)) * METERS;

        const btScalar obstacle_radius = GetCylinderRadius(nh_) * METERS;
        const btScalar obstacle_height = GetCylinderHeight(nh_) * METERS;
        const btVector3 obstacle_half_extents(obstacle_radius, obstacle_radius, obstacle_height / 2.0f);

        // create a box
        BoxObject::Ptr obstacle = boost::make_shared<BoxObject>(
                    0, obstacle_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), obstacle_com));
        obstacle->setColor(pole_color);

        // add the cylinder to the world
        env->add(obstacle);
        world_obstacles_["initial_obstacle"] = obstacle;
    }

    ////////////////////////////////////////////////////////////////////////////
    // "Task barrier" wall settings - i.e. the ones directly impeeding task progress
    ////////////////////////////////////////////////////////////////////////////

    const btScalar task_progress_wall_width =        ROSHelpers::GetParamRequired<btScalar>(nh_, "task_progress_wall_width", __func__).GetImmutable() * METERS;
    const btScalar task_progress_wall_length =       ROSHelpers::GetParamRequired<btScalar>(nh_, "task_progress_wall_length", __func__).GetImmutable() * METERS;
    const btScalar task_progress_wall_lower_height = ROSHelpers::GetParamRequired<btScalar>(nh_, "task_progress_wall_lower_height", __func__).GetImmutable() * METERS;
    const btScalar task_progress_wall_upper_height = ROSHelpers::GetParamRequired<btScalar>(nh_, "task_progress_wall_upper_height", __func__).GetImmutable() * METERS;
    const btScalar task_progress_wall_x_com =        ROSHelpers::GetParamRequired<btScalar>(nh_, "task_progress_wall_x_com", __func__).GetImmutable() * METERS;
    const btScalar task_progress_wall_y_com =        ROSHelpers::GetParamRequired<btScalar>(nh_, "task_progress_wall_y_com", __func__).GetImmutable() * METERS;
    const btScalar task_progress_wall_lower_z_com =  ROSHelpers::GetParamRequired<btScalar>(nh_, "task_progress_wall_lower_z_com", __func__).GetImmutable() * METERS;
    const btScalar task_progress_wall_upper_z_com =  ROSHelpers::GetParamRequired<btScalar>(nh_, "task_progress_wall_upper_z_com", __func__).GetImmutable() * METERS;

    // Vertical wall blocking lower portion of arena - between the start and the goal
    {
        const btVector3 obstacle_half_extents(
                    task_progress_wall_width / 2.0f,
                    task_progress_wall_length / 2.0f,
                    task_progress_wall_lower_height / 2.0f);

        // obstacle parameters
        const btVector3 obstacle_com(
                    task_progress_wall_x_com,
                    task_progress_wall_y_com,
                    task_progress_wall_lower_z_com);

        // create a box
        BoxObject::Ptr obstacle = boost::make_shared<BoxObject>(
                    0, obstacle_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), obstacle_com));
        obstacle->setColor(obstacles_color);

        // add the box to the world
        env->add(obstacle);
        world_obstacles_["task_progress_wall_lower"] = obstacle;
    }
    // Vertical wall blocking upper portion of arena - between the start and the goal
    {
        const btVector3 obstacle_half_extents(
                    task_progress_wall_width / 2.0f,
                    task_progress_wall_length / 2.0f,
                    task_progress_wall_upper_height / 2.0f);

        // obstacle parameters
        const btVector3 obstacle_com(
                    task_progress_wall_x_com,
                    task_progress_wall_y_com,
                    task_progress_wall_upper_z_com);

        // create a box
        BoxObject::Ptr obstacle = boost::make_shared<BoxObject>(
                    0, obstacle_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), obstacle_com));
        obstacle->setColor(obstacles_color);
//        obstacle->setColor(btVector4(0, 0, 0, 0));

        // add the box to the world
        env->add(obstacle);
        world_obstacles_["task_progress_wall_upper"] = obstacle;
    }

    ////////////////////////////////////////////////////////////////////////////
    // "Gripper separator" wall settings - i.e. the ones preventing the grippers
    // on a particular side of the arena, but allowing the rope to pass through it
    ////////////////////////////////////////////////////////////////////////////

    const btScalar gripper_separator_length =       ROSHelpers::GetParamRequired<btScalar>(nh_, "gripper_separator_length", __func__).GetImmutable() * METERS;
    const btScalar gripper_separator_width =        ROSHelpers::GetParamRequired<btScalar>(nh_, "gripper_separator_width", __func__).GetImmutable() * METERS;
    const btScalar gripper_separator_lower_height = ROSHelpers::GetParamRequired<btScalar>(nh_, "gripper_separator_lower_height", __func__).GetImmutable() * METERS;
    const btScalar gripper_separator_upper_height = ROSHelpers::GetParamRequired<btScalar>(nh_, "gripper_separator_upper_height", __func__).GetImmutable() * METERS;
    const btScalar gripper_separator_x_com =        ROSHelpers::GetParamRequired<btScalar>(nh_, "gripper_separator_x_com", __func__).GetImmutable() * METERS;
    const btScalar gripper_separator_y_com =        ROSHelpers::GetParamRequired<btScalar>(nh_, "gripper_separator_y_com", __func__).GetImmutable() * METERS;
    const btScalar gripper_separator_lower_z_com =  ROSHelpers::GetParamRequired<btScalar>(nh_, "gripper_separator_lower_z_com", __func__).GetImmutable() * METERS;
    const btScalar gripper_separator_upper_z_com =  ROSHelpers::GetParamRequired<btScalar>(nh_, "gripper_separator_upper_z_com", __func__).GetImmutable() * METERS;

    // Vertical wall blocking lower portion of arena - between the grippers
    {
        const btVector3 obstacle_half_extents(
                    gripper_separator_length / 2.0f,
                    gripper_separator_width / 2.0f,
                    gripper_separator_lower_height / 2.0f);

        // obstacle parameters
        const btVector3 obstacle_com(
                    gripper_separator_x_com,
                    gripper_separator_y_com,
                    gripper_separator_lower_z_com);

        // create a box
        BoxObject::Ptr obstacle = boost::make_shared<BoxObject>(
                    0, obstacle_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), obstacle_com));
        obstacle->setColor(obstacles_color);

        // add the box to the world
        env->add(obstacle);
        world_obstacles_["gripper_separator_lower"] = obstacle;
    }
    // Vertical wall blocking upper portion of arena - between the grippers
    {
        const btVector3 obstacle_half_extents(
                    gripper_separator_length / 2.0f,
                    gripper_separator_width / 2.0f,
                    gripper_separator_upper_height / 2.0f);

        // obstacle parameters
        const btVector3 obstacle_com(
                    gripper_separator_x_com,
                    gripper_separator_y_com,
                    gripper_separator_upper_z_com);

        // create a box
        BoxObject::Ptr obstacle = boost::make_shared<BoxObject>(
                    0, obstacle_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), obstacle_com));
        obstacle->setColor(obstacles_color);
//        obstacle->setColor(btVector4(0, 0, 0, 0));

        // add the box to the world
        env->add(obstacle);
        world_obstacles_["gripper_separator_upper"] = obstacle;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Hook that is "affixed" to the task progress wall
    ////////////////////////////////////////////////////////////////////////////

    const btScalar hook_length =    ROSHelpers::GetParamRequired<btScalar>(nh_, "hook_length", __func__).GetImmutable() * METERS;
    const btScalar hook_radius =    ROSHelpers::GetParamRequired<btScalar>(nh_, "hook_radius", __func__).GetImmutable() * METERS;
    const btScalar hook_com_x =     ROSHelpers::GetParamRequired<btScalar>(nh_, "hook_com_x", __func__).GetImmutable() * METERS;
    const btScalar hook_com_y =     ROSHelpers::GetParamRequired<btScalar>(nh_, "hook_com_y", __func__).GetImmutable() * METERS;
    const btScalar hook_com_z =     ROSHelpers::GetParamRequired<btScalar>(nh_, "hook_com_z", __func__).GetImmutable() * METERS;

    // Build the hook on the side of the wall towards the starting region (negative x)
    {
        const btVector3 obstacle_half_extents(
                    hook_length / 2.0f,
                    hook_radius,
                    hook_radius);

        const btVector3 obstacle_com(
                    hook_com_x,
                    hook_com_y,
                    hook_com_z);

        // create a box
        BoxObject::Ptr obstacle = boost::make_shared<BoxObject>(
                    0, obstacle_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), obstacle_com));
        obstacle->setColor(hook_color);

        // add the box to the world
        env->add(obstacle);
        world_obstacles_["hook"] = obstacle;
    }

    // Ensure that the gripper must pass on the left (positive y) of the hook
    {
        const btScalar hook_left_face_pos = hook_com_y + hook_radius;
        const btScalar gripper_separator_left_face_pos = gripper_separator_y_com + gripper_separator_width / 2.0f;
        assert(hook_left_face_pos > gripper_separator_left_face_pos);

        const btVector3 obstacle_half_extents(
                    task_progress_wall_width / 2.0f,
                    (hook_left_face_pos - gripper_separator_left_face_pos) / 2.0f,
                    gripper_separator_lower_height / 2.0f);

        const btVector3 obstacle_com(
                    task_progress_wall_x_com,
                    (hook_left_face_pos + gripper_separator_left_face_pos) / 2.0f,
                    world_min.z() + obstacle_half_extents.z());

        // create a box
        BoxObject::Ptr obstacle = boost::make_shared<BoxObject>(
                    0, obstacle_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), obstacle_com));
        obstacle->setColor(obstacles_color);

        // add the box to the world
        env->add(obstacle);
        world_obstacles_["hook_gripper_blocker_lower"] = obstacle;
    }
    {
        const btScalar hook_left_face_pos = hook_com_y + hook_radius;
        const btScalar gripper_separator_left_face_pos = gripper_separator_y_com + gripper_separator_width / 2.0f;
        assert(hook_left_face_pos > gripper_separator_left_face_pos);

        const btVector3 obstacle_half_extents(
                    task_progress_wall_width / 2.0f,
                    (hook_left_face_pos - gripper_separator_left_face_pos) / 2.0f,
                    gripper_separator_upper_height / 2.0f);

        const btVector3 obstacle_com(
                    task_progress_wall_x_com,
                    (hook_left_face_pos + gripper_separator_left_face_pos) / 2.0f,
                    world_max.z() - obstacle_half_extents.z());

        // create a box
        BoxObject::Ptr obstacle = boost::make_shared<BoxObject>(
                    0, obstacle_half_extents,
                    btTransform(btQuaternion(0, 0, 0, 1), obstacle_com));
        obstacle->setColor(obstacles_color);
//        obstacle->setColor(btVector4(0, 0, 0, 0));

        // add the box to the world
        env->add(obstacle);
        world_obstacles_["hook_gripper_blocker_upper"] = obstacle;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Make the target region
    ////////////////////////////////////////////////////////////////////////////

    const std::vector<btVector3> rope_nodes = rope_->getNodes();
    for (size_t cover_idx = 0; cover_idx < rope_nodes.size(); ++cover_idx)
    {
        const btVector3 target_point(world_max.x() - 2 * wall_thickness, rope_nodes[cover_idx].y(), rope_nodes[cover_idx].z());
        cover_points_.push_back(target_point);
    }

    // Set the cover point normals to all be pointing in positive Z
    cover_point_normals_ = std::vector<btVector3>(cover_points_.size(), btVector3(0.0f, 0.0f, 1.0f));

    // Visualize the cover points
    std::vector<btVector4> coverage_color(cover_points_.size(), btVector4(1.0f, 0.0f, 0.0f, 1.0f));
    plot_points_->setPoints(cover_points_, coverage_color);
    env->add(plot_points_);
}

void CustomScene::makeClothHooksObstacles()
{
    assert(false && "makeClothHooksObstacles not implemented!");
}


void CustomScene::makeGenericRegionCoverPoints()
{
    ROS_WARN("Generic region cover points is using an arbitrary surface normal");

    const btScalar x_min = GetCoverRegionXMin(nh_) * METERS;
    const size_t x_steps = GetCoverRegionXSteps(nh_);
    const btScalar x_res = GetCoverRegionXRes(nh_) * METERS;

    const btScalar y_min = GetCoverRegionYMin(nh_) * METERS;
    const size_t y_steps = GetCoverRegionYSteps(nh_);
    const btScalar y_res = GetCoverRegionYRes(nh_) * METERS;

    const btScalar z_min = GetCoverRegionZMin(nh_) * METERS;
    const size_t z_steps = GetCoverRegionZSteps(nh_);
    const btScalar z_res = GetCoverRegionZRes(nh_) * METERS;

    cover_points_.reserve(cover_points_.size() + x_steps * y_steps * z_steps);

    for (size_t x_ind = 0; x_ind < x_steps; ++x_ind)
    {
        const btScalar x = x_min + (btScalar)x_ind * x_res;
        for (size_t y_ind = 0; y_ind < y_steps; ++y_ind)
        {
            const btScalar y = y_min + (btScalar)y_ind * y_res;
            for (size_t z_ind = 0; z_ind < z_steps; ++z_ind)
            {
                const btScalar z = z_min + (btScalar)z_ind * z_res;
                cover_points_.push_back(btVector3(x, y, z));
                cover_point_normals_.push_back(btVector3(0.0f, 0.0f, 1.0f));
            }
        }
    }

    const std::vector<btVector4> coverage_color(cover_points_.size(), btVector4(1, 0, 0, 1));
    plot_points_->setPoints(cover_points_, coverage_color);
    env->add(plot_points_);

    ROS_INFO_STREAM("Num cover points: " << cover_points_.size());
}

void CustomScene::loadCoverPointsFromFile()
{
    // Load the parameters for which file to read from
    const std::string log_folder = GetLogFolder(nh_);
    const std::string file_name_prefix = ROSHelpers::GetParam<std::string>(ph_, "cover_points_file_name_prefix", "cover_points");
    const std::string file_name_suffix = ROSHelpers::GetParam<std::string>(ph_, "cover_points_file_name_suffix_to_load", "cloth_near_robot");
    const std::string file_name = file_name_prefix + "__" + file_name_suffix + ".compressed";

    // Load and decompress
    const auto deserialized = smmap::LoadAndDeserializePointSet(log_folder, file_name);
    const std::string& cover_points_frame = deserialized.first;
    const EigenHelpers::VectorVector3d& cover_points_in_native_frame = deserialized.second;

    // Transform into bullet native frame
    const geometry_msgs::Transform cover_points_frame_to_bt_frame_ros =
            tf_buffer_.lookupTransform(bullet_frame_name_, cover_points_frame, ros::Time(0), ros::Duration(5.0)).transform;
    const Eigen::Isometry3d cover_points_frame_to_bt_frame_eigen =
            EigenHelpersConversions::GeometryTransformToEigenIsometry3d(cover_points_frame_to_bt_frame_ros);

    // Add each point to the cover points list, in the bullet frame
    for (const auto& cover_point_native_frame_eigen : cover_points_in_native_frame)
    {
        const Eigen::Vector3d cover_point_bt_frame_world_distances = cover_points_frame_to_bt_frame_eigen * cover_point_native_frame_eigen;
        const btVector3 cover_point_bt_coords = btVector3((btScalar)cover_point_bt_frame_world_distances.x(),
                                                          (btScalar)cover_point_bt_frame_world_distances.y(),
                                                          (btScalar)cover_point_bt_frame_world_distances.z()) * METERS;

        ROS_WARN("Offsetting the cover points from their saved position by 0.02 m");
        #warning "Cover point position hack for cloth placemat demo"
        // Offset the cover points from the table by a bit
        cover_points_.push_back(cover_point_bt_coords + btVector3(0.0f, 0.0f, 0.02f * METERS));
        cover_point_normals_.push_back(btVector3(0.0f, 0.0f, 1.0f));
    }

    const std::vector<btVector4> coverage_color(cover_points_.size(), btVector4(1, 0, 0, 1));
    plot_points_->setPoints(cover_points_, coverage_color);
    env->add(plot_points_);

    ROS_INFO_STREAM("Num cover points: " << cover_points_.size());
}


void CustomScene::createEdgesToNeighbours(
        const int64_t x_starting_ind,
        const int64_t y_starting_ind,
        const int64_t z_starting_ind)
{
    const int64_t starting_ind = work_space_grid_.xyzIndexToGridIndex(x_starting_ind, y_starting_ind, z_starting_ind);
    assert(starting_ind >= 0);
    const Eigen::Vector3d starting_pos_eigen = work_space_grid_.xyzIndexToWorldPosition(x_starting_ind, y_starting_ind, z_starting_ind);
    const btVector3 starting_pos((btScalar)starting_pos_eigen.x(), (btScalar)starting_pos_eigen.y(), (btScalar)starting_pos_eigen.z());

    // Note that the constructor for btTransform() does not initialize anything, but we set the value in the very next line
    SphereObject::Ptr test_sphere = boost::make_shared<SphereObject>(0, work_space_grid_.minStepDimension() * 0.01, btTransform(), true);
    test_sphere->motionState->setKinematicPos(btTransform(btQuaternion(0, 0, 0, 1), starting_pos));

    // Note that these are in [min, max) form - i.e. exclude the max
    const int64_t x_min_ind = std::max(0L, x_starting_ind - 1);
    const int64_t x_max_ind = std::min(work_space_grid_.getXNumSteps(), x_starting_ind + 2);
    const int64_t y_min_ind = std::max(0L, y_starting_ind - 1);
    const int64_t y_max_ind = std::min(work_space_grid_.getYNumSteps(), y_starting_ind + 2);
    const int64_t z_min_ind = std::max(0L, z_starting_ind - 1);
    const int64_t z_max_ind = std::min(work_space_grid_.getZNumSteps(), z_starting_ind + 2);

    for (int64_t x_loop_ind = x_min_ind; x_loop_ind < x_max_ind; x_loop_ind++)
    {
        for (int64_t y_loop_ind = y_min_ind; y_loop_ind < y_max_ind; y_loop_ind++)
        {
            for (int64_t z_loop_ind = z_min_ind; z_loop_ind < z_max_ind; z_loop_ind++)
            {
                // Only count a neighbour if it's not the same as the starting position
                if (x_starting_ind != x_loop_ind || y_starting_ind != y_loop_ind || z_starting_ind != z_loop_ind)
                {
                    const Eigen::Vector3d test_pos_eigen = work_space_grid_.xyzIndexToWorldPosition(x_loop_ind, y_loop_ind, z_loop_ind);
                    const btVector3 test_pos((btScalar)test_pos_eigen.x(), (btScalar)test_pos_eigen.y(), (btScalar)test_pos_eigen.z());
                    test_sphere->motionState->setKinematicPos(btTransform(btQuaternion(0, 0, 0, 1), test_pos));

                    // If we are not in collision already, then check for neighbours
                    #warning "This check needs to be addressed to allow for errors in perception"
                    if (collisionHelper(test_sphere).m_distance >= 0.0)
                    {
                        const btScalar dist = (test_pos - starting_pos).length();

                        const int64_t loop_ind = work_space_grid_.xyzIndexToGridIndex(x_loop_ind, y_loop_ind, z_loop_ind);
                        assert(loop_ind >= 0);
                        free_space_graph_.AddEdgeBetweenNodes(starting_ind, loop_ind, dist);
                        num_graph_edges_ += 1;
                    }
                }
            }
        }
    }
}

// Note that this function adds edges that move out of collision, to help deal with penetration inaccuracies in Bullet
void CustomScene::createFreeSpaceGraph(const bool draw_graph_corners)
{
    ROS_INFO("Creating free space graph and SDF for export");
    if (draw_graph_corners)
    {
        // Debugging - draw the corners of the grid
        std::cout << "Drawing the 8 corners of the graph world\n";
        graph_corners_.reserve((8));

        const btVector3 world_min = btVector3(
                    (btScalar)GetWorldXMinBulletFrame(nh_),
                    (btScalar)GetWorldYMinBulletFrame(nh_),
                    (btScalar)GetWorldZMinBulletFrame(nh_)) * METERS;

        const btVector3 world_max = btVector3(
                    (btScalar)GetWorldXMaxBulletFrame(nh_),
                    (btScalar)GetWorldYMaxBulletFrame(nh_),
                    (btScalar)GetWorldZMaxBulletFrame(nh_)) * METERS;

        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(world_min.x(), world_min.y(), world_min.z())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(world_min.x(), world_min.y(), world_max.z())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(world_min.x(), world_max.y(), world_min.z())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(world_min.x(), world_max.y(), world_max.z())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(world_max.x(), world_min.y(), world_min.z())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(world_max.x(), world_min.y(), world_max.z())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(world_max.x(), world_max.y(), world_min.z())), 1.0f));
        graph_corners_.push_back(boost::make_shared<PlotAxes>(btTransform(btQuaternion(0, 0, 0, 1), btVector3(world_max.x(), world_max.y(), world_max.z())), 1.0f));

        for (auto& corner: graph_corners_)
        {
            env->add(corner);
        }
    }

    // First add all of the cells to the graph with no edges
    for (int64_t x_ind = 0; x_ind < work_space_grid_.getXNumSteps(); x_ind++)
    {
        for (int64_t y_ind = 0; y_ind < work_space_grid_.getYNumSteps(); y_ind++)
        {
            for (int64_t z_ind = 0; z_ind < work_space_grid_.getZNumSteps(); z_ind++)
            {
                const Eigen::Vector3d node_pos_eigen = work_space_grid_.xyzIndexToWorldPosition(x_ind, y_ind, z_ind);
                const btVector3 node_pos((btScalar)node_pos_eigen.x(), (btScalar)node_pos_eigen.y(), (btScalar)node_pos_eigen.z());
                const int64_t node_ind = free_space_graph_.AddNode(node_pos);
                // Double checking my math here
                assert(node_ind == work_space_grid_.worldPosToGridIndex(node_pos_eigen));
            }
        }
    }

    // Next add edges between adjacent nodes that are not in collision
    for (int64_t x_ind = 0; x_ind < work_space_grid_.getXNumSteps(); x_ind++)
    {
        for (int64_t y_ind = 0; y_ind < work_space_grid_.getYNumSteps(); y_ind++)
        {
            for (int64_t z_ind = 0; z_ind < work_space_grid_.getZNumSteps(); z_ind++)
            {
                createEdgesToNeighbours(x_ind, y_ind, z_ind);
            }
        }
    }

    // Last connect the cover points to the graph
    cover_ind_to_free_space_graph_ind_.resize(cover_points_.size());
    for (size_t cover_ind = 0; cover_ind < cover_points_.size(); cover_ind++)
    {
        const btVector3& cover_point = cover_points_[cover_ind];

        // Find the nearest node in the graph due to the grid
        btVector3 nearest_node_point = cover_point;
        int64_t graph_ind = work_space_grid_.worldPosToGridIndex(nearest_node_point.x(), nearest_node_point.y(), nearest_node_point.z());

        assert(graph_ind >= 0);
        assert(graph_ind < work_space_grid_.getNumCells());

        const double step_size = work_space_grid_.minStepDimension() / 4.0;
        SphereObject::Ptr test_sphere = boost::make_shared<SphereObject>(0, 0.001*METERS, btTransform(), true);

        // If the node is in collision, find the nearest grid node that is not in collision
        while (free_space_graph_.GetNodeImmutable(graph_ind).GetInEdgesImmutable().size() == 0)
        {
            // Find the direction to move
            test_sphere->motionState->setKinematicPos(btTransform(btQuaternion(0, 0, 0, 1), nearest_node_point));
            btPointCollector collision_result = collisionHelper(test_sphere);

            // Move a step out of collision
            nearest_node_point = nearest_node_point + collision_result.m_normalOnBInWorld * (btScalar)step_size;

            graph_ind = work_space_grid_.worldPosToGridIndex(nearest_node_point.x(), nearest_node_point.y(), nearest_node_point.z());
            assert(graph_ind >= 0);
            assert(graph_ind < work_space_grid_.getNumCells());
        }

        // Add an edge between the cover point and the nearest point on the grid
        const btVector3& graph_point = free_space_graph_.GetNodeImmutable(graph_ind).GetValueImmutable();
        const double dist = (graph_point - cover_point).length();
        const int64_t cover_node_ind = free_space_graph_.AddNode(cover_point);
        free_space_graph_.AddEdgesBetweenNodes(cover_node_ind, graph_ind, dist);
        cover_ind_to_free_space_graph_ind_[cover_ind] = graph_ind;

        num_graph_edges_ += 2;
    }

    assert(free_space_graph_.CheckGraphLinkage());
}

void CustomScene::createCollisionMapAndSDF()
{
    const std::string collision_map_file_location = GetCollisionMapStorageLocation(nh_);

    // First check if there are saved results we can use
    try
    {
        const int rv = system(("rosrun deformable_manipulation_experiment_params md5check.sh " + GetTaskTypeString(nh_)).c_str());
        ROS_INFO_STREAM("Checking if a valid collision map already exists: rv: " << rv);
        if (rv != EXIT_SUCCESS)
        {
            throw_arc_exception(std::invalid_argument, "Stored md5sum either does not exist, or needs to be regenerated");
        }
        ROS_INFO("Valid md5sum exists, loading if possible");
        const auto buffer = ZlibHelpers::LoadFromFileAndDecompress(collision_map_file_location);
        // Value deserializer is unused, so pass nullptr
        const uint64_t map_bytes_read = collision_map_for_export_.DeserializeSelf(buffer, 0, nullptr);
        const uint64_t sdf_bytes_read = sdf_for_export_.DeserializeSelf(buffer, map_bytes_read);
        const auto message_array_deserialized =
                arc_utilities::RosMessageDeserializationWrapper<visualization_msgs::MarkerArray>(buffer, map_bytes_read + sdf_bytes_read);
        collision_map_marker_array_for_export_ = message_array_deserialized.first;
        const uint64_t message_array_bytes_read = message_array_deserialized.second;
        assert(map_bytes_read + sdf_bytes_read + message_array_bytes_read == buffer.size());
    }
    catch (const std::exception& e)
    {
        arc_utilities::Stopwatch stopwatch;
        ROS_WARN_STREAM("No valid collision map found: " << e.what());
        ROS_INFO("Generating collision map");

        SphereObject::Ptr test_sphere = boost::make_shared<SphereObject>(0, work_space_grid_.minStepDimension() / sdf_resolution_scale_ * 0.01, btTransform(), true);

        // Itterate through the collision map, checking for collision
        for (int64_t x_ind = 0; x_ind < collision_map_for_export_.GetNumXCells(); x_ind++)
        {
            for (int64_t y_ind = 0; y_ind < collision_map_for_export_.GetNumYCells(); y_ind++)
            {
                for (int64_t z_ind = 0; z_ind < collision_map_for_export_.GetNumZCells(); z_ind++)
                {
                    const Eigen::Vector4d pos = collision_map_for_export_.GridIndexToLocation(x_ind, y_ind, z_ind);

                    const btVector3 point = btVector3((btScalar)(pos.x()), (btScalar)(pos.y()), (btScalar)(pos.z())) * METERS;
                    test_sphere->motionState->setKinematicPos(btTransform(btQuaternion(0, 0, 0, 1), point));
                    const bool freespace = (collisionHelper(test_sphere).m_distance >= 0.0);
                    if (freespace)
                    {
                        collision_map_for_export_.SetValue(x_ind, y_ind, z_ind, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(0.0, FREESPACE));
                    }
                    else
                    {
                        switch (task_type_)
                        {
                            case TaskType::CLOTH_PLACEMAT_LIVE_ROBOT:
                            {
                                BoxObject::Ptr table = boost::polymorphic_pointer_cast<BoxObject>(world_obstacles_["table_surface"]);
                                btTransform table_surface_transform;
                                table->motionState->getWorldTransform(table_surface_transform);
                                const float table_surface_z =  table_surface_transform.getOrigin().z() + table->halfExtents.z();

                                if (pos[2] * METERS > table_surface_z)
                                {
                                    collision_map_for_export_.SetValue(x_ind, y_ind, z_ind, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, PEG));
                                }
                                else
                                {
                                    collision_map_for_export_.SetValue(x_ind, y_ind, z_ind, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, GENERIC_OBSTACLE));
                                }
                                break;
                            }

                            case TaskType::ROPE_HOOKS:
                            case TaskType::ROPE_HOOKS_DATA_GENERATION:
                            {
                                BoxObject::Ptr hook                       = boost::polymorphic_pointer_cast<BoxObject>(world_obstacles_["hook"]);
                                btTransform hook_com;                       hook->motionState->getWorldTransform(hook_com);
                                BoxObject::Ptr initial_obstacle           = boost::polymorphic_pointer_cast<BoxObject>(world_obstacles_["initial_obstacle"]);
                                btTransform initial_obstacle_com;           initial_obstacle->motionState->getWorldTransform(initial_obstacle_com);
                                BoxObject::Ptr task_progress_wall_lower   = boost::polymorphic_pointer_cast<BoxObject>(world_obstacles_["task_progress_wall_lower"]);
                                btTransform task_progress_wall_lower_com;   task_progress_wall_lower->motionState->getWorldTransform(task_progress_wall_lower_com);
                                BoxObject::Ptr task_progress_wall_upper   = boost::polymorphic_pointer_cast<BoxObject>(world_obstacles_["task_progress_wall_upper"]);
                                btTransform task_progress_wall_upper_com;   task_progress_wall_upper->motionState->getWorldTransform(task_progress_wall_upper_com);
                                BoxObject::Ptr gripper_separator_lower    = boost::polymorphic_pointer_cast<BoxObject>(world_obstacles_["gripper_separator_lower"]);
                                btTransform gripper_separator_lower_com;    gripper_separator_lower->motionState->getWorldTransform(gripper_separator_lower_com);
                                BoxObject::Ptr gripper_separator_upper    = boost::polymorphic_pointer_cast<BoxObject>(world_obstacles_["gripper_separator_upper"]);
                                btTransform gripper_separator_upper_com;    gripper_separator_upper->motionState->getWorldTransform(gripper_separator_upper_com);
                                BoxObject::Ptr hook_gripper_blocker_lower = boost::polymorphic_pointer_cast<BoxObject>(world_obstacles_["hook_gripper_blocker_lower"]);
                                btTransform hook_gripper_blocker_lower_com; hook_gripper_blocker_lower->motionState->getWorldTransform(hook_gripper_blocker_lower_com);
                                BoxObject::Ptr hook_gripper_blocker_upper = boost::polymorphic_pointer_cast<BoxObject>(world_obstacles_["hook_gripper_blocker_upper"]);
                                btTransform hook_gripper_blocker_upper_com; hook_gripper_blocker_upper->motionState->getWorldTransform(hook_gripper_blocker_upper_com);

                                if (IsPointInsideOABB(hook_com, hook->halfExtents, point))
                                {
                                    collision_map_for_export_.SetValue(x_ind, y_ind, z_ind, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, HOOK));
                                }
                                else if (IsPointInsideOABB(initial_obstacle_com, initial_obstacle->halfExtents, point))
                                {
                                    collision_map_for_export_.SetValue(x_ind, y_ind, z_ind, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, INITIAL_OBSTACLE));
                                }
                                else if (IsPointInsideOABB(task_progress_wall_lower_com, task_progress_wall_lower->halfExtents, point)
                                         || IsPointInsideOABB(gripper_separator_lower_com, gripper_separator_lower->halfExtents, point)
                                         || IsPointInsideOABB(hook_gripper_blocker_lower_com, hook_gripper_blocker_lower->halfExtents, point))
                                {
                                    collision_map_for_export_.SetValue(x_ind, y_ind, z_ind, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, LOWER_OBSTACLES));
                                }
                                else if (IsPointInsideOABB(task_progress_wall_upper_com, task_progress_wall_upper->halfExtents, point)
                                         || IsPointInsideOABB(gripper_separator_upper_com, gripper_separator_upper->halfExtents, point)
                                         || IsPointInsideOABB(hook_gripper_blocker_upper_com, hook_gripper_blocker_upper->halfExtents, point))
                                {
                                    collision_map_for_export_.SetValue(x_ind, y_ind, z_ind, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, UPPER_OBSTACLES));
                                }
                                else
                                {
                                    std::cout << "Point:        " << PrettyPrint::PrettyPrint(point, true, " ") << std::endl;
                                    std::cout << "Hook:         " << PrettyPrint::PrettyPrint(hook_com, true, " ")                          << "    " << PrettyPrint::PrettyPrint(hook->halfExtents, true, " ") << std::endl;
                                    std::cout << "Initial obs:  " << PrettyPrint::PrettyPrint(initial_obstacle_com, true, " ")              << "    " << PrettyPrint::PrettyPrint(initial_obstacle->halfExtents, true, " ") << std::endl;
                                    std::cout << "progress dow: " << PrettyPrint::PrettyPrint(task_progress_wall_lower_com, true, " ")      << "    " << PrettyPrint::PrettyPrint(task_progress_wall_lower->halfExtents, true, " ") << std::endl;
                                    std::cout << "progress up:  " << PrettyPrint::PrettyPrint(task_progress_wall_upper_com, true, " ")      << "    " << PrettyPrint::PrettyPrint(task_progress_wall_upper->halfExtents, true, " ") << std::endl;
                                    std::cout << "seperate dow: " << PrettyPrint::PrettyPrint(gripper_separator_lower_com, true, " ")       << "    " << PrettyPrint::PrettyPrint(gripper_separator_lower->halfExtents, true, " ") << std::endl;
                                    std::cout << "seperate up:  " << PrettyPrint::PrettyPrint(gripper_separator_upper_com, true, " ")       << "    " << PrettyPrint::PrettyPrint(gripper_separator_upper->halfExtents, true, " ") << std::endl;
                                    std::cout << "g block dow:  " << PrettyPrint::PrettyPrint(hook_gripper_blocker_lower_com, true, " ")    << "    " << PrettyPrint::PrettyPrint(hook_gripper_blocker_lower->halfExtents, true, " ") << std::endl;
                                    std::cout << "g block up:   " << PrettyPrint::PrettyPrint(hook_gripper_blocker_upper_com, true, " ")    << "    " << PrettyPrint::PrettyPrint(hook_gripper_blocker_upper->halfExtents, true, " ") << std::endl;

                                    assert(false);
                                    collision_map_for_export_.SetValue(x_ind, y_ind, z_ind, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, GENERIC_OBSTACLE));
                                }
                                break;
                            }

                            default:
                                collision_map_for_export_.SetValue(x_ind, y_ind, z_ind, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0, GENERIC_OBSTACLE));
                        }
                    }
                }
            }
        }

        // Move the collision map to world frame
        const Eigen::Isometry3d bullet_to_collision_map_starting_origin = collision_map_for_export_.GetOriginTransform();
        const geometry_msgs::TransformStamped world_to_bullet_tf_as_ros =
                tf_buffer_.lookupTransform(world_frame_name_, bullet_frame_name_, ros::Time::now(), ros::Duration(10.0));
        const Eigen::Isometry3d world_to_bullet_as_eigen = EigenHelpersConversions::GeometryTransformToEigenIsometry3d(world_to_bullet_tf_as_ros.transform);
        const Eigen::Isometry3d world_to_collision_map_origin = world_to_bullet_as_eigen * bullet_to_collision_map_starting_origin;
        collision_map_for_export_.UpdateOriginTransform(world_to_collision_map_origin);
        collision_map_for_export_.SetFrame(world_frame_name_);
        ROS_INFO_STREAM("Finished creating collision map in " << stopwatch(arc_utilities::READ) << " seconds");

        ROS_INFO("Generating SDF");
        stopwatch(arc_utilities::RESET);
        std::vector<uint32_t> obstacle_ids_to_use(ObjectIds::LAST_ID);
        std::iota(obstacle_ids_to_use.begin(), obstacle_ids_to_use.end(), 1);
        // We're setting a negative value here to indicate that we are in collision outisde of the explicit region of the SDF;
        // this is so that when we queury the SDF, we get that out of bounds is "in collision" or "not allowed"
        sdf_for_export_ = collision_map_for_export_ .ExtractSignedDistanceField(-BT_LARGE_FLOAT, obstacle_ids_to_use, false, false).first;
        sdf_for_export_.Lock();
        ROS_INFO_STREAM("Finished SDF in " << stopwatch(arc_utilities::READ) << " seconds");

        ROS_INFO("Generating markers for collision map visualization");
        stopwatch(arc_utilities::RESET);
        collision_map_marker_array_for_export_ = collision_map_for_export_.ExportContourOnlyForDisplayUniqueNs(object_color_map_);
        ROS_INFO_STREAM("Finished generating marker in " << stopwatch(arc_utilities::READ) << " seconds");

        try
        {
            ROS_INFO_STREAM("Serializing CollisionMap, SDF, and RViz Marker and saving to file");
            std::vector<uint8_t> buffer;
            const auto map_bytes_written = collision_map_for_export_.SerializeSelf(buffer, nullptr);
            const auto sdf_bytes_written = sdf_for_export_.SerializeSelf(buffer);
            const auto message_array_bytes_written = arc_utilities::RosMessageSerializationWrapper(collision_map_marker_array_for_export_, buffer);
            assert(buffer.size() == map_bytes_written + sdf_bytes_written + message_array_bytes_written);
            ZlibHelpers::CompressAndWriteToFile(buffer, collision_map_file_location);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("Saving CollsionMap to file failed: " << e.what());
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Internal helper functions
////////////////////////////////////////////////////////////////////////////////

static void SetGripperTransform(
        const std::map<std::string, GripperKinematicObject::Ptr>& grippers_map,
        const std::string& name,
        const btTransform& pose_in_bt_coords)
{
    grippers_map.at(name)->setWorldTransform(pose_in_bt_coords);
}

static std::vector<btTransform> GetGripperTransforms(
        const std::map<std::string, GripperKinematicObject::Ptr>& grippers_map,
        const std::vector<std::string>& names)
{
    std::vector<btTransform> transforms;
    transforms.reserve(names.size());
    for (const std::string& gripper_name: names)
    {
        const auto gripper = grippers_map.at(gripper_name);
        transforms.push_back(gripper->getWorldTransform());
    }
    return transforms;
}

btTransform CustomScene::getTransform(
        const std::string& parent,
        const std::string& child)
{
    const geometry_msgs::TransformStamped tf =
            tf_buffer_.lookupTransform(parent, child, ros::Time(0.0), ros::Duration(0.0));
    return toBulletTransform(tf.transform, METERS);
}

geometry_msgs::PoseStamped CustomScene::transformPoseToBulletFrame(
        const std_msgs::Header& input_header,
        const geometry_msgs::Pose& pose_in_input_frame) const
{
    geometry_msgs::PoseStamped stamped_input;
    stamped_input.header = input_header;
    stamped_input.pose = pose_in_input_frame;

    // Get the latest available transform, whatever that is
    geometry_msgs::PoseStamped pose_in_bullet_frame;
    tf_buffer_.transform(stamped_input, pose_in_bullet_frame, bullet_frame_name_, ros::Time(0), world_frame_name_);

    return pose_in_bullet_frame;
}

dmm::WorldState CustomScene::createSimulatorFbk(
        const CapsuleRope::ConstPtr rope,
        const BulletSoftObject::ConstPtr cloth,
        const std::map<std::string, GripperKinematicObject::Ptr>& grippers) const
{
    assert(num_timesteps_to_execute_per_gripper_cmd_ > 0);

    dmm::WorldState msg;

    // fill out the object configuration data
    msg.object_configuration = toRosPointVector(world_to_bullet_tf_, getDeformableObjectNodes(rope, cloth), METERS);
    if (rope != nullptr)
    {
        msg.rope_node_transforms = toRosPoseVector(world_to_bullet_tf_, rope->getNodesTransforms(), METERS);
    }

    // fill out the gripper data
    for (const std::string &gripper_name: auto_grippers_)
    {
        const GripperKinematicObject::Ptr gripper = grippers.at(gripper_name);
        msg.gripper_names.push_back(gripper_name);
        msg.gripper_poses.push_back(toRosPose(world_to_bullet_tf_, gripper->getWorldTransform(), METERS));

        btPointCollector collision_result = collisionHelper(gripper);

        if (collision_result.m_hasResult)
        {
            msg.gripper_distance_to_obstacle.push_back(collision_result.m_distance / METERS);
            msg.obstacle_surface_normal.push_back(toRosVector3(collision_result.m_normalOnBInWorld, 1.0f));
            msg.gripper_nearest_point_to_obstacle.push_back(toRosPoint(
                        collision_result.m_pointInWorld
                        + collision_result.m_normalOnBInWorld * collision_result.m_distance, METERS));
        }
        else
        {
            msg.gripper_distance_to_obstacle.push_back(std::numeric_limits<double>::infinity());
            msg.obstacle_surface_normal.push_back(toRosVector3(btVector3(1.0f, 0.0f, 0.0f), 1));
            msg.gripper_nearest_point_to_obstacle.push_back(toRosPoint(btVector3(0.0f, 0.0f, 0.0f), 1));
        }
    }

    // fill out the robot configuration data as being invalid
    msg.robot_configuration_valid = false;

    // update the sim_time
    // TODO: in some cumstances, I think this math is wrong
    // sim time won't be updated yet, when using testMicrosteps service or testRobotMotion action server
    msg.sim_time = (simTime - base_sim_time_) / (double)num_timesteps_to_execute_per_gripper_cmd_;

    msg.header.frame_id = world_frame_name_;
    msg.header.stamp = ros::Time::now();

    return msg;
}

std::vector<btVector3> CustomScene::getDeformableObjectNodes(
        const CapsuleRope::ConstPtr rope,
        const BulletSoftObject::ConstPtr cloth) const
{
    std::vector<btVector3> nodes;

    switch (deformable_type_)
    {
        case DeformableType::ROPE:
            nodes = rope->getNodes();
            break;

        case DeformableType::CLOTH:
            nodes = nodeArrayToNodePosVector(cloth->softBody->m_nodes);
            break;
    }

    return nodes;
}

/**
 * @brief Invoke bullet's collision detector to find the points on the gripper
 * and cylinder/table that are nearest each other.
 *
 * @param gripper_name The name of the gripper to check for collision
 *
 * @return The result of the collision check
 */
btPointCollector CustomScene::collisionHelper(const GripperKinematicObject::Ptr& gripper) const
{
    assert(gripper);
    // Note that gjkOutput initializes to hasResult = false and m_distance = BT_LARGE_FLOAT
    btPointCollector gjkOutput_min;

    // find the distance to any objects in the world
    for (auto ittr = world_obstacles_.begin(); ittr != world_obstacles_.end(); ++ittr)
    {
        BulletObject::Ptr obj = ittr->second;
        for (size_t gripper_child_ind = 0; gripper_child_ind < gripper->getChildren().size(); gripper_child_ind++)
        {
            // TODO: how much (if any) of this should be static/class members?
            btGjkEpaPenetrationDepthSolver epaSolver;
            btVoronoiSimplexSolver sGjkSimplexSolver;
            btPointCollector gjkOutput;

            btGjkPairDetector convexConvex(
                        dynamic_cast<btBoxShape*>(gripper->getChildren()[gripper_child_ind]->collisionShape.get()),
                        dynamic_cast<btConvexShape*>(obj->collisionShape.get()),
                        &sGjkSimplexSolver,
                        &epaSolver);

            btGjkPairDetector::ClosestPointInput input;
            gripper->getChildren()[gripper_child_ind]->motionState->getWorldTransform(input.m_transformA);
            obj->motionState->getWorldTransform(input.m_transformB);
            input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);
            convexConvex.getClosestPoints(input, gjkOutput, nullptr);

            if (gjkOutput.m_distance < gjkOutput_min.m_distance)
            {
                gjkOutput_min = gjkOutput;
            }
        }
    }

    gjkOutput_min.m_normalOnBInWorld.normalize();
    return gjkOutput_min;
}

/**
 * @brief CustomScene::collisionHelper
 * @param sphere
 * @return
 */
btPointCollector CustomScene::collisionHelper(const SphereObject::Ptr& sphere) const
{
    assert(sphere);
    // Note that gjkOutput initializes to hasResult = false and m_distance = BT_LARGE_FLOAT
    btPointCollector gjkOutput_min;

    // find the distance to any objects in the world
    for (auto ittr = world_obstacles_.begin(); ittr != world_obstacles_.end(); ++ittr)
    {
        BulletObject::Ptr obj = ittr->second;

        // TODO: how much (if any) of this should be static/class members?
        btGjkEpaPenetrationDepthSolver epaSolver;
        btVoronoiSimplexSolver sGjkSimplexSolver;
        btPointCollector gjkOutput;

        btGjkPairDetector convexConvex(
                    dynamic_cast<btSphereShape*>(sphere->collisionShape.get()),
                    dynamic_cast<btConvexShape*>(obj->collisionShape.get()),
                    &sGjkSimplexSolver,
                    &epaSolver);

        btGjkPairDetector::ClosestPointInput input;
        sphere->motionState->getWorldTransform(input.m_transformA);
        obj->motionState->getWorldTransform(input.m_transformB);
        input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);
        convexConvex.getClosestPoints(input, gjkOutput, nullptr);

        if (gjkOutput.m_distance < gjkOutput_min.m_distance)
        {
            gjkOutput_min = gjkOutput;
        }
    }

    gjkOutput_min.m_normalOnBInWorld.normalize();
    return gjkOutput_min;
}

bool CustomScene::ropeNodeTransformsValid(const std::vector<btTransform>& nodes) const
{
    assert(rope_ != nullptr && "This only makes sense with a valid rope object");
    assert(nodes.size() == rope_->getChildren().size());

    bool valid = true;
    for (size_t node_idx = 0; node_idx < nodes.size(); ++node_idx)
    {
        CapsuleObject::Ptr template_capsule = boost::dynamic_pointer_cast<CapsuleObject>(rope_->getChildren()[node_idx]);
        const btScalar mass = 0.0f;
        CapsuleObject::Ptr test_capsule = boost::make_shared<CapsuleObject>(
                    mass, template_capsule->getRadius(), template_capsule->getHeight(), nodes[node_idx]);


        // find the distance to any objects in the world
        for (auto ittr = world_obstacles_.begin(); ittr != world_obstacles_.end(); ++ittr)
        {
            BulletObject::Ptr obj = ittr->second;

            // TODO: how much (if any) of this should be static/class members?
            btGjkEpaPenetrationDepthSolver epaSolver;
            btVoronoiSimplexSolver sGjkSimplexSolver;
            btPointCollector gjkOutput;

            btGjkPairDetector convexConvex(
                        dynamic_cast<btConvexShape*>(test_capsule->collisionShape.get()),
                        dynamic_cast<btConvexShape*>(obj->collisionShape.get()),
                        &sGjkSimplexSolver,
                        &epaSolver);

            btGjkPairDetector::ClosestPointInput input;
            test_capsule->motionState->getWorldTransform(input.m_transformA);
            obj->motionState->getWorldTransform(input.m_transformB);
            input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);
            convexConvex.getClosestPoints(input, gjkOutput, nullptr);

            if (gjkOutput.m_distance < 0.0f)
            {
                valid = false;
                break;
            }
        }

        if (!valid)
        {
            break;
        }

    }
    return valid;
}

////////////////////////////////////////////////////////////////////////////////
// Fork and Fork-visualization management
////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<ViewerData> CustomScene::createVisualizerForFork(SimForkResult& fork_result)
{
    std::shared_ptr<ViewerData> viewer(new ViewerData(*this, fork_result));

    viewer->viewer_.addEventHandler(new CustomKeyHandler(*this));

    viewer->dbgDraw_.reset(new osgbCollision::GLDebugDrawer());
    viewer->dbgDraw_->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE /*btIDebugDraw::DBG_DrawWireframe*/);
    viewer->dbgDraw_->setEnabled(false);
    fork_result.bullet_->dynamicsWorld->setDebugDrawer(viewer->dbgDraw_.get());
    fork_result.osg_->root->addChild(viewer->dbgDraw_->getSceneGraph());

    viewer->viewer_.setUpViewInWindow(0, 0, ViewerConfig::windowWidth, ViewerConfig::windowHeight);
    viewer->manip_ = new EventHandler(*this);
    viewer->manip_->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition), util::toOSGVector(ViewerConfig::pointCameraLooksAt), osg::Z_AXIS);
    viewer->viewer_.setCameraManipulator(viewer->manip_);
    viewer->viewer_.setSceneData(fork_result.osg_->root.get());
    viewer->viewer_.realize();

    return viewer;
}

SimForkResult CustomScene::createForkWithNoSimulationDone(
        const Environment::Ptr environment_to_fork,
        BulletSoftObject::Ptr cloth_to_fork,
        boost::shared_ptr<CapsuleRope> rope_to_fork,
        std::map<std::string, GripperKinematicObject::Ptr> grippers_to_fork)
{
    assert(task_type_ != TaskType::CLOTH_COLAB_FOLDING && "This does not yet work with colab folding - due to manual gripper path stuff");

    SimForkResult result;
    result.bullet_.reset(new BulletInstance());
    result.bullet_->setGravity(BulletConfig::gravity);
    result.osg_.reset(new OSGInstance());
    result.fork_.reset(new Fork(environment_to_fork, result.bullet_, result.osg_));
    result.cloth_ = boost::static_pointer_cast<BulletSoftObject>(result.fork_->forkOf(cloth_to_fork));
    result.rope_ = boost::static_pointer_cast<CapsuleRope>(result.fork_->forkOf(rope_to_fork));

    for (const auto& gripper: grippers_to_fork)
    {
        GripperKinematicObject::Ptr gripper_copy = boost::static_pointer_cast<GripperKinematicObject>(result.fork_->forkOf(gripper.second));
        assert(gripper_copy);
        result.grippers_[gripper.first] = gripper_copy;
    }

    // TODO: Why do we need to do this with rope
    // If we have a rope, regrasp with the gripper
    if (result.rope_ != nullptr)
    {
        // Single gripper experiments, assumes that the gripper is grasping the "0th" index of the rope
        if (result.grippers_.size() == 1)
        {
            result.grippers_[auto_grippers_[0]]->rigidGrab(result.rope_->getChildren()[0]->rigidBody.get(), 0);
        }
        // Double gripper experiments, assumes that the gripper is grasping the "0th" and last index of the rope
        else if (result.grippers_.size() == 2)
        {
            const size_t object_node_ind = result.rope_->getChildren().size() - 1;
            result.grippers_[auto_grippers_[0]]->rigidGrab(result.rope_->getChildren()[0]->rigidBody.get(), 0);
            result.grippers_[auto_grippers_[1]]->rigidGrab(result.rope_->getChildren()[object_node_ind]->rigidBody.get(), object_node_ind);
        }
        else
        {
            assert(false && "grippers size is neither 1 nor 2 ");
        }
    }

    return result;
}

SimForkResult CustomScene::createForkWithNoSimulationDone(
        const std::vector<std::string>& gripper_names,
        const std::vector<btTransform>& gripper_poses_in_bt_coords)
{
    assert(gripper_names.size() == gripper_poses_in_bt_coords.size());
    SimForkResult result = createForkWithNoSimulationDone(env, cloth_, rope_, grippers_);
    for (size_t gripper_ind = 0; gripper_ind < gripper_names.size(); gripper_ind++)
    {
        SetGripperTransform(result.grippers_, gripper_names[gripper_ind], gripper_poses_in_bt_coords[gripper_ind]);
    }
    return result;
}

SimForkResult CustomScene::simulateInNewFork(
        const std::vector<std::string>& gripper_names,
        const std::vector<btTransform>& gripper_poses_in_bt_coords)
{
    SimForkResult result = createForkWithNoSimulationDone(gripper_names, gripper_poses_in_bt_coords);
    for (size_t timestep = 0; timestep < num_timesteps_to_execute_per_gripper_cmd_; timestep++)
    {
        result.fork_->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks - Getting information
////////////////////////////////////////////////////////////////////////////////

bool CustomScene::getGripperNamesCallback(
        dmm::GetGripperNames::Request& req,
        dmm::GetGripperNames::Response& res)
{
    (void)req;
    res.names = auto_grippers_;
    return true;
}

bool CustomScene::getGripperAttachedNodeIndicesCallback(
        dmm::GetGripperAttachedNodeIndices::Request& req,
        dmm::GetGripperAttachedNodeIndices::Response& res)
{
    GripperKinematicObject::Ptr gripper = grippers_.at(req.name);
    res.indices = gripper->getAttachedNodeIndices();
    return true;
}

bool CustomScene::getGripperStretchingVectorInfoCallback(
        dmm::GetGripperStretchingVectorInfo::Request& req,
        dmm::GetGripperStretchingVectorInfo::Response& res)
{
    if (cloth_ != nullptr)
    {
        GripperKinematicObject::Ptr gripper = grippers_.at(req.name);
        res.to_gripper_name = gripper->getClothGeoInfoToAnotherGripper().to_gripper_name;
        res.attatched_indices = gripper->getClothGeoInfoToAnotherGripper().from_nodes;
        res.neighbor_indices = gripper->getClothGeoInfoToAnotherGripper().to_nodes;
        res.contributions = gripper->getClothGeoInfoToAnotherGripper().node_contribution;
        return true;
    }
    else
    {
        ROS_WARN_ONCE("getGripperStretchingVectorInfo called for non-cloth task, this doesn't make sense");
        return false;
    }
}

bool CustomScene::getGripperPoseCallback(
        dmm::GetGripperPose::Request& req,
        dmm::GetGripperPose::Response& res)
{
    GripperKinematicObject::Ptr gripper = grippers_.at(req.name);
    res.pose = toRosPose(world_to_bullet_tf_, gripper->getWorldTransform(), METERS);
    res.header.frame_id = world_frame_name_;
    res.header.stamp = ros::Time::now();
    return true;
}

bool CustomScene::getRobotConfigurationCallback(
        dmm::GetRobotConfiguration::Request& req,
        dmm::GetRobotConfiguration::Response& res)
{
    (void)req;
    res.valid = false;
    return true;
}

bool CustomScene::gripperCollisionCheckCallback(
        dmm::GetGripperCollisionReport::Request& req,
        dmm::GetGripperCollisionReport::Response& res)
{
    size_t num_checks = req.pose.size();

    res.gripper_distance_to_obstacle.resize(num_checks);
    res.gripper_nearest_point_to_obstacle.resize(num_checks);
    res.obstacle_surface_normal.resize(num_checks);

    for (size_t pose_ind = 0; pose_ind < num_checks; pose_ind++)
    {
        // Convert the pose into bullet coordinates
        const geometry_msgs::PoseStamped pose_in_bt_frame = transformPoseToBulletFrame(req.header, req.pose[pose_ind]);
        const btTransform pose_in_bt_coords = toBulletTransform(pose_in_bt_frame.pose, METERS);

        // Move the collision check gripper to the querry pose and do the querry
        collision_check_gripper_->setWorldTransform(pose_in_bt_coords);
        const btPointCollector collision_result = collisionHelper(collision_check_gripper_);

        if (collision_result.m_hasResult)
        {
            res.gripper_distance_to_obstacle[pose_ind] = collision_result.m_distance / METERS;
        }
        else
        {
            res.gripper_distance_to_obstacle[pose_ind] = std::numeric_limits<double>::infinity();
        }

        // Rotate the vector to the world frame for output - btScale = 1.0 because we want a normalized vector
        res.obstacle_surface_normal[pose_ind] = toRosVector3(world_to_bullet_tf_, collision_result.m_normalOnBInWorld, 1.0);

        // Calculate the nearest point on the gripper in bullet frame coordinates
        const btVector3 gripper_nearest_point_to_obstacle_bt_frame =
                collision_result.m_pointInWorld + collision_result.m_normalOnBInWorld * collision_result.m_distance;

        // Transform the point to the world frame for output
        res.gripper_nearest_point_to_obstacle[pose_ind] =
                toRosPoint(world_to_bullet_tf_, gripper_nearest_point_to_obstacle_bt_frame, METERS);
    }

    res.header.frame_id = world_frame_name_;
    res.header.stamp = ros::Time::now();

    return true;
}

bool CustomScene::getCoverPointsCallback(
        dmm::GetPointSet::Request& req,
        dmm::GetPointSet::Response& res)
{
    (void)req;
    res.points = toRosPointVector(world_to_bullet_tf_, cover_points_, METERS);
    res.header.frame_id = world_frame_name_;
    res.header.stamp = ros::Time::now();
    return true;
}

bool CustomScene::getCoverPointNormalsCallback(
        dmm::GetVector3Set::Request& req,
        dmm::GetVector3Set::Response& res)
{
    (void)req;
    // Because we want normalized vectors, don't rescale to world coords
    res.vectors = toRosVec3Vector(world_to_bullet_tf_, cover_point_normals_, 1.0f);
    res.header.frame_id = world_frame_name_;
    res.header.stamp = ros::Time::now();
    return true;
}

bool CustomScene::getMirrorLineCallback(
        dmm::GetMirrorLine::Request& req,
        dmm::GetMirrorLine::Response& res)
{
    (void)req;
    if (task_type_ == TaskType::CLOTH_COLAB_FOLDING &&
         deformable_type_ == DeformableType::CLOTH)
    {
        // TODO: Update mirror line data with non-bullet frames - currently uses bullet frame
        assert(bullet_frame_name_ == world_frame_name_);
        res = mirror_line_data_;
        return true;
    }
    else
    {
        res.mid_x = std::numeric_limits<double>::infinity();
        res.min_y = std::numeric_limits<double>::infinity();
        res.max_y = std::numeric_limits<double>::infinity();
        return false;
    }
}

bool CustomScene::getFreeSpaceGraphCallback(
        dmm::GetFreeSpaceGraphRequest& req,
        dmm::GetFreeSpaceGraphResponse& res)
{
    (void)req;

    // First serialze the graph itself
    {
        // Create a copy of the graph in world coordinates and world scaling
        auto resized_graph = free_space_graph_;
        for (auto& node: resized_graph.GetNodesMutable())
        {
            const btVector3 point_in_world_frame = world_to_bullet_tf_ * node.GetValueImmutable() / METERS;
            node.GetValueMutable() = point_in_world_frame;

            // Lower the weight by the same distance factor
            for (auto& edge: node.GetInEdgesMutable())
            {
                edge.SetWeight(edge.GetWeight() / METERS);
            }
            for (auto& edge: node.GetOutEdgesMutable())
            {
                edge.SetWeight(edge.GetWeight() / METERS);
            }
        }
        assert(resized_graph.CheckGraphLinkage());

        // Assumes that the data is already in the world frame and distance scale
        const auto value_serializer_fn = [] (const btVector3& value, std::vector<uint8_t>& buffer)
        {
            const uint64_t start_buffer_size = buffer.size();
            uint64_t running_total = 0;

            running_total += arc_utilities::SerializeFixedSizePOD<btScalar>(value.x(), buffer);
            running_total += arc_utilities::SerializeFixedSizePOD<btScalar>(value.y(), buffer);
            running_total += arc_utilities::SerializeFixedSizePOD<btScalar>(value.z(), buffer);

            const uint64_t end_buffer_size = buffer.size();
            const uint64_t bytes_written = end_buffer_size - start_buffer_size;

            assert(running_total == bytes_written);

            return bytes_written;
        };

        // Allocate a data buffer to store the results in
        res.graph_data_buffer.clear();
        size_t expected_data_len = 0;
        expected_data_len += sizeof(size_t); // Graph node vector header
        expected_data_len += free_space_graph_.GetNodesImmutable().size() * 3 * sizeof(btScalar); // Value item for each graph node
        expected_data_len += free_space_graph_.GetNodesImmutable().size() * sizeof(double);       // Distance value for each graph node
        expected_data_len += free_space_graph_.GetNodesImmutable().size() * 2 * sizeof(size_t);   // Edge vector overhead
        expected_data_len += num_graph_edges_ * sizeof(arc_dijkstras::GraphEdge);                 // Total number of edges to store
        res.graph_data_buffer.reserve(expected_data_len);

        // Finally, serialze the graph
        resized_graph.SerializeSelf(res.graph_data_buffer, value_serializer_fn);
    }

    // Next add the mapping between cover indices and graph indices
    res.cover_point_ind_to_graph_ind = cover_ind_to_free_space_graph_ind_;

    // And add the header information
    res.header.frame_id = world_frame_name_;
    res.header.stamp = ros::Time::now();

    return true;
}

bool CustomScene::getSignedDistanceFieldCallback(
        dmm::GetSignedDistanceFieldRequest &req,
        dmm::GetSignedDistanceFieldResponse &res)
{
    (void)req;
    // The SDF should already be in world frame and distances, so no conversion needed
    res.sdf = sdf_tools::SignedDistanceField::GetMessageRepresentation(sdf_for_export_);
    return true;
}

bool CustomScene::getObjectInitialConfigurationCallback(
        dmm::GetPointSet::Request& req,
        dmm::GetPointSet::Response& res)
{
    (void)req;
    res.points = object_initial_configuration_;
    res.header.frame_id = world_frame_name_;
    res.header.stamp = ros::Time::now();
    return true;
}

bool CustomScene::getObjectCurrentConfigurationCallback(
        dmm::GetPointSet::Request& req,
        dmm::GetPointSet::Response& res)
{
    (void)req;
    res.points = toRosPointVector(world_to_bullet_tf_, getDeformableObjectNodes(rope_, cloth_), METERS);
    res.header.frame_id = world_frame_name_;
    res.header.stamp = ros::Time::now();
    return true;
}

bool CustomScene::getRopeCurrentNodeTransforms(
        dmm::GetPoseSet::Request& req,
        dmm::GetPoseSet::Response& res)
{
    (void)req;
    res.poses = toRosPoseVector(world_to_bullet_tf_, rope_->getNodesTransforms(), METERS);
    res.header.frame_id = world_frame_name_;
    res.header.stamp = ros::Time::now();
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks - Simulation actions
////////////////////////////////////////////////////////////////////////////////

bool CustomScene::executeRobotMotionCallback(
        dmm::ExecuteRobotMotion::Request& req,
        dmm::ExecuteRobotMotion::Response& res)
{
    assert(req.grippers_names.size() == req.gripper_poses.size());
    ROS_INFO("Executing gripper command");

    std::lock_guard<std::mutex> lock(sim_mutex_);

    for (size_t gripper_ind = 0; gripper_ind < req.grippers_names.size(); gripper_ind++)
    {
        const geometry_msgs::PoseStamped pose_in_bt_frame = transformPoseToBulletFrame(req.header, req.gripper_poses[gripper_ind]);
        const btTransform pose_in_bt_coords = toBulletTransform(pose_in_bt_frame.pose, METERS);
        SetGripperTransform(grippers_, req.grippers_names[gripper_ind], pose_in_bt_coords);
    }

    res.microstep_state_history.reserve(num_timesteps_to_execute_per_gripper_cmd_);
    double total_time = 0.0;
    for (size_t timestep = 0; timestep < num_timesteps_to_execute_per_gripper_cmd_; timestep++)
    {
        screen_recorder_->snapshot();
        arc_utilities::Stopwatch stopwatch;
        step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
        total_time += stopwatch(arc_utilities::READ);
        const auto world_state = createSimulatorFbk(rope_, cloth_, grippers_);
        res.microstep_state_history.push_back(world_state);
        simulator_fbk_pub_.publish(world_state);
    }

    LOG_STREAM(simulation_time_logger_, num_timesteps_to_execute_per_gripper_cmd_ << " " << total_time);

    res.world_state = res.microstep_state_history.back();
    return true;
}

bool CustomScene::testRobotMotionMicrostepsCallback(
        dmm::TestRobotMotionMicrosteps::Request& req,
        dmm::TestRobotMotionMicrosteps::Response& res)
{
    assert(task_type_ == ROPE_HOOKS_DATA_GENERATION &&
           "This service only makes sense for this single task");

    assert(req.grippers_names.size() == req.starting_gripper_poses.size());
    assert(req.grippers_names.size() == req.target_gripper_poses.size());
    ROS_INFO("Executing test robot motion microsteps");
    std::lock_guard<std::mutex> lock(sim_mutex_);

    const btTransform input_to_bullet_tf_ =
            getTransform(req.header.frame_id, bullet_frame_name_);

    const auto node_transforms_bt_coords =
            toBulletTransformVector(input_to_bullet_tf_, req.starting_object_configuration, METERS);
    if (!ropeNodeTransformsValid(node_transforms_bt_coords))
    {
        return true;
    }

    // Set the a copy of the rope to the desired starting configuration
    CapsuleRope::Ptr test_rope = duplicateRopeAtInitialConfig();
    env->add(test_rope);
    test_rope->setNodesTransforms(node_transforms_bt_coords);
    // Duplicate the existing grippers at the new rope starting configuration
    std::map<std::string, GripperKinematicObject::Ptr> test_grippers;
    for (const auto& gripper_itr : grippers_)
    {
        const std::string gripper_name = gripper_itr.first;
        const GripperKinematicObject::Ptr gripper = gripper_itr.second;
        test_grippers[gripper_name] = boost::make_shared<GripperKinematicObject>(
                    env,
                    gripper_name,
                    gripper->apperture,
                    GRIPPER_COLOR);
        env->add(test_grippers[gripper_name]);
        const auto object_node_indices = gripper->getAttachedNodeIndices();
        assert(object_node_indices.size() == 1);
        const auto object_node_ind = object_node_indices[0];
        test_grippers[gripper_name]->setWorldTransform(test_rope->getChildren()[object_node_ind]->rigidBody->getCenterOfMassTransform());
        test_grippers[gripper_name]->rigidGrab(test_rope->getChildren()[object_node_ind]->rigidBody.get(), object_node_ind);
    }
    // Move the grippers to the specified starting position
    for (size_t gripper_ind = 0; gripper_ind < req.grippers_names.size(); gripper_ind++)
    {
        const btTransform pose_in_bt_coords =
                toBulletTransform(input_to_bullet_tf_, req.starting_gripper_poses[gripper_ind], METERS);
        SetGripperTransform(test_grippers, req.grippers_names[gripper_ind], pose_in_bt_coords);
    }

    // Let the rope settle at the specified stating position
    for (size_t timestep = 0; timestep < num_timesteps_to_execute_per_gripper_cmd_; timestep++)
    {
        step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
    }
    res.start_after_settling = createSimulatorFbk(test_rope, nullptr, test_grippers);

    res.microsteps.reserve(num_timesteps_to_execute_per_gripper_cmd_ * req.num_substeps);
    for (ssize_t macrostep_idx = 0; macrostep_idx < req.num_substeps; ++macrostep_idx)
    {
        // Interpolate along the target gripper movements
        const double ratio = (req.num_substeps != 1) ? (double)macrostep_idx / (double)(req.num_substeps - 1) : 1.0;
        const auto grippers_target = InterpolateVectors(req.starting_gripper_poses, req.target_gripper_poses, ratio);

        // Move the grippers to the specified interpolated poses
        for (size_t gripper_idx = 0; gripper_idx < req.grippers_names.size(); gripper_idx++)
        {
            const btTransform pose_in_bt_coords =
                    toBulletTransform(input_to_bullet_tf_, grippers_target[gripper_idx], METERS);
            SetGripperTransform(test_grippers, req.grippers_names[gripper_idx], pose_in_bt_coords);
        }

        // Simulate the rope movement
        for (size_t timestep = 0; timestep < num_timesteps_to_execute_per_gripper_cmd_; timestep++)
        {
            step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            res.microsteps.push_back(createSimulatorFbk(test_rope, nullptr, test_grippers));
        }
    }

    env->remove(test_rope);
    for (auto& gripper: test_grippers)
    {
        env->remove(gripper.second);
    }

    return true;
}

void CustomScene::testRobotMotionExecuteCallback(
        const dmm::TestRobotMotionGoalConstPtr& goal)
{
    const size_t num_tests = goal->poses_to_test.size();

    std::lock_guard<std::mutex> lock(sim_mutex_);
    #pragma omp parallel for
    for (size_t test_ind = 0; test_ind < num_tests; test_ind++)
    {
        ROS_INFO_STREAM_NAMED("test_gripper_poses", "Testing gripper pose " << test_ind);
        const std::vector<btTransform> gripper_poses_in_bt_frame =
                toBulletTransformVector(world_to_bullet_tf_, goal->poses_to_test[test_ind].poses, METERS);
        const SimForkResult result = simulateInNewFork(goal->gripper_names, gripper_poses_in_bt_frame);

        // Create a feedback message
        dmm::TestRobotMotionFeedback fbk;
        fbk.world_state = createSimulatorFbk(result.rope_, result.cloth_, result.grippers_);
        fbk.test_id = test_ind;
        // Send feedback
        test_grippers_poses_as_.publishFeedback(fbk);
    }

    test_grippers_poses_as_.setSucceeded();
}

void CustomScene::generateTransitionDataExecuteCallback(
        const dmm::GenerateTransitionDataGoalConstPtr& goal)
{
    assert(task_type_ == ROPE_HOOKS_DATA_GENERATION &&
           "This service only makes sense for this single task");

    assert(goal->filenames.size() == 0 || goal->tests.size() == goal->filenames.size());

    std::lock_guard<std::mutex> lock(sim_mutex_);
    #pragma omp parallel for
    for (size_t test_idx = 0; test_idx < goal->tests.size(); test_idx++)
    {
        const auto& test = goal->tests[test_idx];
        SimForkResult forked_sim = createForkWithNoSimulationDone(env, cloth_, rope_, grippers_);
        dmm::GenerateTransitionDataFeedback fbk;
        fbk.test_id = test_idx;
        fbk.test_result.header.frame_id = world_frame_name_;
        fbk.test_result.header.stamp = ros::Time::now();

        const btTransform input_to_bullet_tf_ = getTransform(test.header.frame_id, bullet_frame_name_);

        // Start the rope at the specified coordinates
        const auto node_transforms_bt_coords =
                toBulletTransformVector(input_to_bullet_tf_, test.starting_object_configuration, METERS);
        if (!ropeNodeTransformsValid(node_transforms_bt_coords))
        {
            assert(false && "something wierd here");
        }
        forked_sim.rope_->setNodesTransforms(node_transforms_bt_coords);

        // Start the grippers at the specified coordnates
        for (size_t gripper_idx = 0; gripper_idx < test.gripper_names.size(); gripper_idx++)
        {
            const btTransform pose_in_bt_coords =
                    toBulletTransform(input_to_bullet_tf_, test.starting_gripper_poses[gripper_idx], METERS);
            SetGripperTransform(forked_sim.grippers_, test.gripper_names[gripper_idx], pose_in_bt_coords);
        }

        // Follow the path to the start configuration
        {
            assert(test.path_to_start_of_test.size() == test.path_num_substeps.size());
            const auto path_total_microsteps = std::accumulate(test.path_num_substeps.begin(), test.path_num_substeps.end(), test.final_num_substeps);
            fbk.test_result.microsteps_all.reserve(path_total_microsteps);
            for (size_t path_idx = 0; path_idx < test.path_to_start_of_test.size(); ++path_idx)
            {
                std::vector<btTransform> grippers_interpolate_start_poses_bt_coords =
                        GetGripperTransforms(forked_sim.grippers_, test.gripper_names);
                const std::vector<btTransform> path_grippers_target_bt_coords =
                        toBulletTransformVector(world_to_bullet_tf_, test.path_to_start_of_test[path_idx].poses, METERS);
                const auto num_substeps = test.path_num_substeps[path_idx];

                // Take a single macrostep for this segment of the path
                for (ssize_t macrostep_idx = 0; macrostep_idx < num_substeps; ++macrostep_idx)
                {
                    // Interpolate along the target gripper movements
                    const btScalar ratio = (num_substeps != 1) ? (btScalar)macrostep_idx / (btScalar)(num_substeps - 1) : 1.0f;
                    const auto grippers_interpolated_bt_coords = InterpolateVectors(grippers_interpolate_start_poses_bt_coords, path_grippers_target_bt_coords, ratio);

                    // Move the grippers to the specified interpolated poses
                    for (size_t gripper_idx = 0; gripper_idx < test.gripper_names.size(); gripper_idx++)
                    {
                        SetGripperTransform(forked_sim.grippers_, test.gripper_names[gripper_idx], grippers_interpolated_bt_coords[gripper_idx]);
                    }

                    // Simulate the rope movement
                    for (size_t timestep = 0; timestep < num_timesteps_to_execute_per_gripper_cmd_; timestep++)
                    {
                        forked_sim.fork_->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
                        fbk.test_result.microsteps_all.push_back(createSimulatorFbk(forked_sim.rope_, nullptr, forked_sim.grippers_));
                    }
                }
            }
            fbk.test_result.start_after_following_path = fbk.test_result.microsteps_all.back();
        }

        // Take the last step
        {
            const auto grippers_target_bt_coords = toBulletTransformVector(world_to_bullet_tf_, test.final_gripper_targets, METERS);
            std::vector<btTransform> grippers_interpolate_start_poses_bt_coords = GetGripperTransforms(forked_sim.grippers_, test.gripper_names);
            fbk.test_result.microsteps_last_action.reserve(test.final_num_substeps * num_timesteps_to_execute_per_gripper_cmd_);
            for (ssize_t macrostep_idx = 0; macrostep_idx < test.final_num_substeps; ++macrostep_idx)
            {
                // Interpolate along the target gripper movements
                const btScalar ratio = (test.final_num_substeps != 1) ? (btScalar)macrostep_idx / (btScalar)(test.final_num_substeps - 1) : 1.0f;
                const auto grippers_interpolated_bt_coords = InterpolateVectors(grippers_interpolate_start_poses_bt_coords, grippers_target_bt_coords, ratio);

                // Move the grippers to the specified interpolated poses
                for (size_t gripper_idx = 0; gripper_idx < test.gripper_names.size(); gripper_idx++)
                {
                    SetGripperTransform(forked_sim.grippers_, test.gripper_names[gripper_idx], grippers_interpolated_bt_coords[gripper_idx]);
                }

                // Simulate the rope movement
                for (size_t timestep = 0; timestep < num_timesteps_to_execute_per_gripper_cmd_; timestep++)
                {
                    forked_sim.fork_->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
                    fbk.test_result.microsteps_all.push_back(createSimulatorFbk(forked_sim.rope_, nullptr, forked_sim.grippers_));
                    fbk.test_result.microsteps_last_action.push_back(fbk.test_result.microsteps_all.back());
                }
            }
        }

        generate_transition_data_as_.publishFeedback(fbk);
        std::vector<uint8_t> buffer;
        arc_utilities::RosMessageSerializationWrapper(fbk, buffer);
        ZlibHelpers::CompressAndWriteToFile(buffer, goal->filenames[test_idx].data);
    }
    generate_transition_data_as_.setSucceeded();
}

////////////////////////////////////////////////////////////////////////////////
// Pre-step Callbacks
////////////////////////////////////////////////////////////////////////////////

void CustomScene::drawAxes()
{
    for (auto& axes: gripper_axes_)
    {
        axes.second->setup(grippers_[axes.first]->getWorldTransform(), 1);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Post-step Callbacks
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Key Handler for our Custom Scene
////////////////////////////////////////////////////////////////////////////////

CustomScene::CustomKeyHandler::CustomKeyHandler(CustomScene &scene)
    : scene_(scene)
    , current_gripper_(nullptr)
    , translate_gripper_(false)
    , rotate_gripper_(false)
{}

bool CustomScene::CustomKeyHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    (void)aa;
    bool suppress_default_handler = false;

    switch (ea.getEventType())
    {
        case osgGA::GUIEventAdapter::KEYDOWN:
        {
            switch (ea.getKey())
            {
                // Gripper translate/rotate selection keybinds
                {
                    case '1':
                    {
                        current_gripper_ = getGripper(0);
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'q':
                    {
                        current_gripper_ = getGripper(0);
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }

                    case '2':
                    {
                        current_gripper_ = getGripper(1);
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'w':
                    {
                        current_gripper_ = getGripper(1);
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }

                    case '3':
                    {
                        current_gripper_ = getGripper(2);
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'e':
                    {
                        current_gripper_ = getGripper(2);
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }

                    case '4':
                    {
                        current_gripper_ = getGripper(3);
                        translate_gripper_ = true;
                        rotate_gripper_ = false;
                        break;
                    }
                    case 'r':
                    {
                        current_gripper_ = getGripper(3);
                        translate_gripper_ = false;
                        rotate_gripper_ = true;
                        break;
                    }
                }

                // Simulation behaviour keybinds
                {
                    case 'a':
                    {
                        scene_.advance_grippers_.store(!scene_.advance_grippers_.load());
                        break;
                    }
                }

                // Visualization keybonds
                {
//                    case 'c':
//                    {
//                        std_srvs::Empty::Request req;
//                        std_srvs::Empty::Response res;
//                        scene_.clearVisualizationsCallback(req, res);
                        // The scene is already locked in the main sim loop,
                        // thus we call the internall handler, not the external interface
//                        scene_.rviz_marker_manager_->clearVisualizationsInternalHandler();
//                        break;
//                    }
                }

/*
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
*/
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


                case 'c':
                    scene.regraspWithOneGripper(scene.left_gripper1,scene.left_gripper2);
                    break;

                case 'v':
                    scene.regraspWithOneGripper(scene.right_gripper1,scene.right_gripper2);
                    break;
*/
/*
                case 'f':
                    // scene.regraspWithOneGripper(scene.right_gripper1,scene.left_gripper1);
                    // scene.left_gripper1->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,100)));
                    scene.testRelease(scene.left_gripper1);
                    break;

                case 't':
                    scene.testRelease2(scene.left_gripper2);
                    break;scene.inputState.startDragging = true;

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
/*
                 case 'k':
                     scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*
                         scene.right_gripper1->getWorldTransform().getRotation(),
                         scene.right_gripper1->getWorldTransform().getOrigin()));
                     break;

                 case ',':
                     scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*
                         scene.right_gripper1->getWorldTransform().getRotation(),
                         scene.right_gripper1->getWorldTransform().getOrigin()));
                     break;


                 case 'l':
                     scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),0.2)*
                         scene.right_gripper2->getWorldTransform().getRotation(),
                         scene.right_gripper2->getWorldTransform().getOrigin()));
                     break;

                case '.':
                    scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(1,0,0),-0.2)*
                        scene.right_gripper2->getWorldTransform().getRotation(),
                        scene.right_gripper2->getWorldTransform().getOrigin()));
                    break;


                 case 'y':
                     scene.right_gripper1->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*
                         scene.right_gripper1->getWorldTransform().getRotation(),
                         scene.right_gripper1->getWorldTransform().getOrigin()));
                     break;


                case 'u':
                    scene.right_gripper2->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),-0.2)*
                        scene.right_gripper2->getWorldTransform().getRotation(),
                        scene.right_gripper2->getWorldTransform().getOrigin()));
                    break;
*/
/*
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
*/
            }
            break;
        }

        case osgGA::GUIEventAdapter::KEYUP:
        {
            switch (ea.getKey())
            {
                case '1':
                case 'q':
                case '2':
                case 'w':
                case '3':
                case 'e':
                case '4':
                case 'r':
                {
                    current_gripper_ = nullptr;
                    translate_gripper_ = false;
                    rotate_gripper_ = false;
                }
            }
            break;
        }

        case osgGA::GUIEventAdapter::PUSH:
        {
            start_dragging_ = true;
            break;
        }

        case osgGA::GUIEventAdapter::DRAG:
        {
            // drag the active manipulator in the plane of view
            if ((ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) && current_gripper_)
            {
                // if we've just started moving, reset our internal position trackers
                if (start_dragging_)
                {
                    mouse_last_x_ = ea.getXnormalized();
                    mouse_last_y_ = ea.getYnormalized();
                    start_dragging_ = false;
                }
                float dx = mouse_last_x_ - ea.getXnormalized();
                float dy = ea.getYnormalized() - mouse_last_y_;

                mouse_last_x_ = ea.getXnormalized();
                mouse_last_y_ = ea.getYnormalized();

                // get our current view
                osg::Vec3d osgCenter, osgEye, osgUp;
                scene_.manip->getTransformation(osgCenter, osgEye, osgUp);
                btVector3 from(util::toBtVector(osgEye));
                btVector3 to(util::toBtVector(osgCenter));
                btVector3 up(util::toBtVector(osgUp)); up.normalize();

                // compute basis vectors for the plane of view
                // (the plane normal to the ray from the camera to the center of the scene)
                btVector3 camera_normal_vector = (to - from).normalized();
                btVector3 camera_y_axis = (up - (up.dot(camera_normal_vector))*camera_normal_vector).normalized(); //TODO: FIXME: is this necessary with osg?
                btVector3 camera_x_axis = camera_normal_vector.cross(camera_y_axis);
                btVector3 drag_vector = SceneConfig::mouseDragScale * (dx*camera_x_axis + dy*camera_y_axis);

                btTransform current_transform = current_gripper_->getWorldTransform();
                btTransform next_transform(current_transform);

                if (translate_gripper_)
                {
                    // if moving the manip, just set the origin appropriately
                    next_transform.setOrigin(drag_vector + current_transform.getOrigin());
                }
                else if (rotate_gripper_)
                {
                    // if we're rotating, the axis is perpendicular to the
                    // direction the mouse is dragging
                    btVector3 axis = camera_normal_vector.cross(drag_vector);
                    btScalar angle = drag_vector.length();
                    btQuaternion rot(axis, angle);
                    // we must ensure that we never get a bad rotation quaternion
                    // due to really small (effectively zero) mouse movements
                    // this is the easiest way to do this:
                    if (rot.length() > 0.99f && rot.length() < 1.01f)
                    {
                        next_transform.setRotation(rot * current_transform.getRotation());
                    }
                }

                // We don't need to lock here, as this is called inside of draw()
                // at which point we've already locked
                current_gripper_->setWorldTransform(next_transform);
                suppress_default_handler = true;
            }
            break;
        }

        default:
        {
            break;
        }
    }
    return suppress_default_handler;
}

GripperKinematicObject::Ptr CustomScene::CustomKeyHandler::getGripper(size_t gripper_num)
{
    if (gripper_num < scene_.auto_grippers_.size())
    {
        return scene_.grippers_[scene_.auto_grippers_[gripper_num]];
    }
    else if (gripper_num - scene_.auto_grippers_.size() < scene_.manual_grippers_.size())
    {
        gripper_num -= scene_.auto_grippers_.size();
        return scene_.grippers_[scene_.manual_grippers_[gripper_num]];
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("deform_simulator", "Invalid gripper number: " << gripper_num);
        ROS_ERROR_STREAM_NAMED("deform_simulator", "Existing auto grippers:   " << PrettyPrint::PrettyPrint(scene_.auto_grippers_));
        ROS_ERROR_STREAM_NAMED("deform_simulator", "Existing manual grippers: " << PrettyPrint::PrettyPrint(scene_.manual_grippers_));
        return nullptr;
    }
}
