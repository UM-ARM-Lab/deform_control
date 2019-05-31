#ifndef BULLET_ROS_CONVERSIONS_HPP
#define BULLET_ROS_CONVERSIONS_HPP

#include <btBulletDynamicsCommon.h>
#include <osg/Geometry>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>

// NOTE: The notation that I'm using here to denote "world_to_bullet_tf" is such that the following holds:
// point_in_world_coords = world_to_bullet_tf * point_in_bullet_coords

namespace BulletHelpers
{
    //// ROS to Bullet Conversions /////////////////////////////////////////////////////////////////////////////////////

    inline btQuaternion toBulletQuaternion(
            const geometry_msgs::Quaternion& quat)
    {
        return btQuaternion(quat.x, quat.y, quat.z, quat.w);
    }

    inline btQuaternion toBulletQuaternion(
            const btTransform& world_to_bullet_tf,
            const geometry_msgs::Quaternion& quat)
    {
        return world_to_bullet_tf.inverse() * toBulletQuaternion(quat);
    }

    inline btVector3 toBulletVector3(
            const geometry_msgs::Point& pos,
            const float bt_scale)
    {
        return btVector3(pos.x, pos.y, pos.z) * bt_scale;
    }

    inline btVector3 toBulletVector3(
            const btTransform& world_to_bullet_tf,
            const geometry_msgs::Point& pos,
            const float bt_scale)
    {
        return world_to_bullet_tf.inverse() * toBulletVector3(pos, bt_scale);
    }

    inline btVector3 toBulletVector3(
            const geometry_msgs::Vector3& vec,
            const float bt_scale)
    {
        return btVector3(vec.x, vec.y, vec.z) * bt_scale;
    }

    // Vectors do not change length on transformation, only orientation
    inline btVector3 toBulletVector3(
            const btTransform& world_to_bullet_tf,
            const geometry_msgs::Vector3& vec,
            const float bt_scale)
    {
        return world_to_bullet_tf.getBasis().inverse() * toBulletVector3(vec, bt_scale);
    }

    inline btTransform toBulletTransform(
            const geometry_msgs::Pose& geom_pose,
            const float bt_scale)
    {
        const btQuaternion rot = toBulletQuaternion(geom_pose.orientation);
        const btVector3 trans = toBulletVector3(geom_pose.position, bt_scale);
        const btTransform bullet_tf(rot, trans);
        return bullet_tf;
    }

    inline btTransform toBulletTransform(
            const btTransform& world_to_bullet_tf,
            const geometry_msgs::Pose& geom_pose,
            const float bt_scale)
    {
        const btTransform bullet_tf = toBulletTransform(geom_pose, bt_scale);
        return world_to_bullet_tf.inverseTimes(bullet_tf);
    }

    inline btTransform toBulletTransform(
            const geometry_msgs::Transform& geom_tf,
            const float bt_scale)
    {
        const btQuaternion rot = toBulletQuaternion(geom_tf.rotation);
        const btVector3 trans = toBulletVector3(geom_tf.translation, bt_scale);
        const btTransform bullet_tf(rot, trans);

        // Sanity check the output
        assert(-100.0f < bullet_tf.getOrigin().x() && bullet_tf.getOrigin().x() < 100.0f);
        assert(-100.0f < bullet_tf.getOrigin().y() && bullet_tf.getOrigin().y() < 100.0f);
        assert(-100.0f < bullet_tf.getOrigin().z() && bullet_tf.getOrigin().z() < 100.0f);

        return bullet_tf;
    }

    inline btTransform toBulletTransform(
            const btTransform& world_to_bullet_tf,
            const geometry_msgs::Transform& geom_tf,
            const float bt_scale)
    {
        const btTransform bullet_tf = toBulletTransform(geom_tf, bt_scale);
        return world_to_bullet_tf.inverseTimes(bullet_tf);
    }

    inline std::vector<btTransform> toBulletTransformVector(
            const btTransform& world_to_bullet_tf,
            const std::vector<geometry_msgs::Pose>& geom_poses,
            const float bt_scale)
    {
        std::vector<btTransform> bt_transforms;
        bt_transforms.reserve(geom_poses.size());
        for (size_t pose_ind = 0; pose_ind < geom_poses.size(); ++pose_ind)
        {
            bt_transforms.push_back(toBulletTransform(world_to_bullet_tf, geom_poses[pose_ind], bt_scale));
        }
        return bt_transforms;
    }

    inline std::vector<btVector3> toBulletPointVector(
            const btTransform& bullet_tf,
            const std::vector<geometry_msgs::Point>& ros,
            const float bt_scale)
    {
        std::vector<btVector3> bt(ros.size());
        for (size_t i = 0; i < ros.size(); ++i)
        {
            bt[i] = toBulletVector3(bullet_tf, ros[i], bt_scale);
        }
        return bt;
    }

    inline std::vector<btVector3> toBulletPointVector(
            const geometry_msgs::Pose& tf,
            const std::vector<geometry_msgs::Point>& ros,
            const float bt_scale)
    {
        const btTransform bullet_tf = toBulletTransform(tf, bt_scale);
        return toBulletPointVector(bullet_tf, ros, bt_scale);
    }

    inline std::vector<btVector3> toBulletPointVector(
            const btTransform& world_to_bullet_tf,
            const geometry_msgs::Pose& tf,
            const std::vector<geometry_msgs::Point>& ros,
            const float bt_scale)
    {
        const btTransform bullet_tf = toBulletTransform(world_to_bullet_tf, tf, bt_scale);
        return toBulletPointVector(bullet_tf, ros, bt_scale);
    }

    inline btVector4 toBulletColor(const std_msgs::ColorRGBA& ros)
    {
        return btVector4(ros.r, ros.g, ros.b, ros.a);
    }

    inline std::vector<btVector4> toBulletColorArray(
            const std::vector<std_msgs::ColorRGBA>& ros)
    {
        std::vector<btVector4> bt(ros.size());
        for (size_t i = 0; i < ros.size() ; i++)
        {
            bt[i] = toBulletColor(ros[i]);
        }
        return bt;
    }

    inline std::vector<btVector4> toBulletColorArray(
            const std_msgs::ColorRGBA& ros,
            size_t num_copies)
    {
        return std::vector<btVector4>(num_copies, toBulletColor(ros));
    }

    //// Bullet to ROS Conversions /////////////////////////////////////////////////////////////////////////////////////

    inline geometry_msgs::Quaternion toRosQuaternion(
            const btQuaternion& quat)
    {
        geometry_msgs::Quaternion ros;
        ros.x = quat.x();
        ros.y = quat.y();
        ros.z = quat.z();
        ros.w = quat.w();
        return ros;
    }

    inline geometry_msgs::Quaternion toRosQuaternion(
            const btTransform& world_to_bullet_tf,
            const btQuaternion& quat)
    {
        const btQuaternion final_orientation = world_to_bullet_tf.getRotation() * quat;
        return toRosQuaternion(final_orientation);
    }

    inline geometry_msgs::Point toRosPoint(
            const btVector3& point,
            const float bt_scale)
    {
        assert(-100.0f < point.x() && point.x() < 100.0f && "Data sanity check failed in bullet_ros_conversions.hpp");
        assert(-100.0f < point.y() && point.y() < 100.0f && "Data sanity check failed in bullet_ros_conversions.hpp");
        assert(-100.0f < point.z() && point.z() < 100.0f && "Data sanity check failed in bullet_ros_conversions.hpp");
        geometry_msgs::Point ros;
        ros.x = point.x() / bt_scale;
        ros.y = point.y() / bt_scale;
        ros.z = point.z() / bt_scale;
        return ros;
    }

    inline geometry_msgs::Point toRosPoint(
            const btTransform& world_to_bullet_tf,
            const btVector3& point,
            const float bt_scale)
    {
        const btVector3 final_point = world_to_bullet_tf * point;
        return toRosPoint(final_point, bt_scale);
    }

    inline geometry_msgs::Vector3 toRosVector3(
            const btVector3& vec,
            const float bt_scale)
    {
        geometry_msgs::Vector3 ros;
        ros.x = vec.x() * bt_scale;
        ros.y = vec.y() * bt_scale;
        ros.z = vec.z() * bt_scale;
        return ros;
    }

    // Vectors do not change length on transformation, only orientation, thus
    // we only use the basis portion of the transform
    inline geometry_msgs::Vector3 toRosVector3(
            const btTransform& world_to_bullet_tf,
            const btVector3& vec,
            const float bt_scale)
    {
        const btVector3 final_vec = world_to_bullet_tf.getBasis() * vec;
        return toRosVector3(final_vec, bt_scale);
    }

    inline geometry_msgs::Pose toRosPose(
            const btTransform& world_to_bullet_tf,
            const btTransform& pose,
            const float bt_scale)
    {
        const btTransform final_pose = world_to_bullet_tf * pose;
        geometry_msgs::Pose ros;
        ros.position = toRosPoint(final_pose.getOrigin(), bt_scale);
        ros.orientation = toRosQuaternion(final_pose.getRotation());
        return ros;
    }

    inline std::vector<geometry_msgs::Pose> toRosPoseVector(
            const btTransform& world_to_bullet_tf,
            const std::vector<btTransform>& bt,
            const float bt_scale)
    {
        std::vector<geometry_msgs::Pose> ros(bt.size());
        for (size_t i = 0; i < bt.size() ; ++i)
        {
            ros[i] = toRosPose(world_to_bullet_tf, bt[i], bt_scale);
        }
        return ros;
    }

    inline geometry_msgs::Transform toRosTransform(
            const btTransform& world_to_bullet_tf,
            const btTransform& tf,
            const float bt_scale)
    {
        const btTransform final_tf = world_to_bullet_tf * tf;
        geometry_msgs::Transform ros;
        ros.translation = toRosVector3(final_tf.getOrigin(), bt_scale);
        ros.rotation = toRosQuaternion(final_tf.getRotation());
        return ros;
    }

    inline std::vector<geometry_msgs::Transform> toRosTransformVector(
            const btTransform& world_to_bullet_tf,
            const std::vector<btTransform>& bt,
            const float bt_scale)
    {
        std::vector<geometry_msgs::Transform> ros(bt.size());
        for (size_t i = 0; i < bt.size() ; ++i)
        {
            ros[i] = toRosTransform(world_to_bullet_tf, bt[i], bt_scale);
        }
        return ros;
    }

    inline std::vector<geometry_msgs::Point> toRosPointVector(
            const btTransform& world_to_bullet_tf,
            const std::vector<btVector3>& bt,
            const float bt_scale)
    {
        std::vector<geometry_msgs::Point> ros(bt.size());
        for (size_t i = 0; i < bt.size() ; ++i)
        {
            ros[i] = toRosPoint(world_to_bullet_tf * bt[i], bt_scale);
        }
        return ros;
    }

    // When transforming vectors (as opposed to points), we only rotate
    inline std::vector<geometry_msgs::Vector3> toRosVec3Vector(
            const btTransform& world_to_bullet_tf,
            const std::vector<btVector3>& bt,
            const float bt_scale)
    {
        std::vector<geometry_msgs::Vector3> ros(bt.size());
        for (size_t i = 0; i < bt.size() ; ++i)
        {
            ros[i] = toRosVector3(world_to_bullet_tf, bt[i], bt_scale);
        }
        return ros;
    }

    inline sensor_msgs::PointCloud2 toPointCloud2(
            const btTransform& world_to_bullet_tf,
            const std::vector<btVector3>& bt,
            const float bt_scale)
    {
        sensor_msgs::PointCloud2 output;
        // Mark the output as being un-ordered (i.e., not comming from a 2D image style sensor)
        output.width = bt.size();
        output.height = 1;
        output.is_bigendian = false;
        output.is_dense = false;

        // Mark the output as having exactly 3 fields, each field consuming 1 btScalar
        static_assert(std::is_same<btScalar, float>::value || std::is_same<btScalar, double>::value,
                      "btScalar is neither a float nor a double, this is very strange");
        const auto datatype = std::is_same<btScalar, float>::value ?
                    sensor_msgs::PointField::FLOAT32 : sensor_msgs::PointField::FLOAT64;
        output.fields.resize(3);
        output.fields[0].name = "x";
        output.fields[0].offset = 0;
        output.fields[0].datatype = datatype;
        output.fields[0].count = 1;
        output.fields[1].name = "y";
        output.fields[1].offset = sizeof(btScalar);
        output.fields[1].datatype = datatype;
        output.fields[1].count = 1;
        output.fields[2].name = "z";
        output.fields[2].offset = 2 * sizeof(btScalar);;
        output.fields[2].datatype = datatype;
        output.fields[2].count = 1;

        output.point_step = 3 * sizeof(btScalar);
        output.row_step = output.point_step * output.width;

        output.data.resize(bt.size() * output.point_step);
        for (size_t i = 0; i < bt.size() ; ++i)
        {
            const btVector3 world_point = (world_to_bullet_tf * bt[i]) * bt_scale;
            // Note that this copy is specialized to the particular format of btVector3
            std::memcpy(&output.data[i * output.point_step], world_point.m_floats, output.point_step);
        }

        return output;
    }

    //// ROS to OSG Conversions ////////////////////////////////////////////////////////////////////////////////////////

    inline osg::ref_ptr<osg::Vec3Array> toOsgRefVec3Array(
            const btTransform& world_to_bullet_tf,
            const geometry_msgs::Pose& pose,
            const std::vector<geometry_msgs::Point>& ros,
            const float bt_scale)
    {
        // Converting through Eigen as I am not familiar with any bullet <---> osg conversions that exist
        const btTransform final_bt_tf = world_to_bullet_tf.inverseTimes(toBulletTransform(pose, bt_scale));

        osg::ref_ptr<osg::Vec3Array> out = new osg::Vec3Array();
        out->reserve(ros.size());
        for (const auto& point: ros)
        {
            const btVector3 bt_point = final_bt_tf * toBulletVector3(point, bt_scale);
            out->push_back(osg::Vec3(bt_point.x(), bt_point.y(), bt_point.z()));
        }
        return out;
    }

    inline osg::Quat toOsgQuat(const geometry_msgs::Quaternion& ros_quat)
    {
        return osg::Quat(ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w);
    }

    inline osg::Vec3 toOsgVec3(const geometry_msgs::Vector3& ros, const float bt_scale)
    {
        return osg::Vec3(ros.x, ros.y, ros.z) * bt_scale;
    }

    inline osg::Vec3 toOsgVec3(const geometry_msgs::Point& ros, const float bt_scale)
    {
        return osg::Vec3(ros.x, ros.y, ros.z) * bt_scale;
    }

    inline osg::ref_ptr<osg::Vec3Array> toOsgRefVec3Array(const std::vector<geometry_msgs::Point>& ros, const float bt_scale)
    {
        osg::ref_ptr<osg::Vec3Array> out = new osg::Vec3Array();
        out->reserve(ros.size());
        for(const auto& point: ros)
        {
            out->push_back(toOsgVec3(point, bt_scale));
        }
        return out;
    }

    inline osg::ref_ptr<osg::Vec4Array> toOsgRefVec4Array(
            const std::vector<std_msgs::ColorRGBA>& ros)
    {
        osg::ref_ptr<osg::Vec4Array> out = new osg::Vec4Array();
        out->reserve(ros.size());
        for (const auto& color: ros)
        {
            out->push_back(osg::Vec4(color.r, color.g, color.b, color.a));
        }
        return out;
    }

    //// Misc //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // We have to essentially double the entries for every internal point of marker.points and marker.colors
    inline void convertLineStripToLineList(
            visualization_msgs::Marker& marker)
    {
        const size_t num_lines = marker.points.size();

        // Make a copy so that we can mutate the incomming list
        const auto points = marker.points;
        const auto colors = marker.colors;

        assert(points.size() >= 2);
        assert(points.size() == colors.size());

        marker.points.resize(num_lines * 2 - 2);
        marker.colors.resize(num_lines * 2 - 2);
        for (size_t ind = 1; ind < num_lines * 2 - 2; ++ind)
        {
            marker.points[ind] = points[(ind + 1) / 2];
            marker.colors[ind] = colors[(ind + 1) / 2];
        }
    }
}

#endif // BULLET_ROS_CONVERSIONS_HPP
