#ifndef BULLET_ROS_CONVERSIONS_HPP
#define BULLET_ROS_CONVERSIONS_HPP

#include <btBulletDynamicsCommon.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

namespace BulletHelpers
{
    inline btQuaternion toBulletQuaternion( const geometry_msgs::Quaternion& quat )
    {
        return btQuaternion( quat.x, quat.y, quat.z, quat.w );
    }

    inline btVector3 toBulletVector3( const geometry_msgs::Point& pos, float bt_scale )
    {
        return btVector3( pos.x, pos.y, pos.z )*bt_scale;
    }

    inline btTransform toBulletTransform( const geometry_msgs::Pose& pose, float bt_scale )
    {
        btQuaternion rot = toBulletQuaternion( pose.orientation );
        btVector3 trans = toBulletVector3( pose.position, bt_scale );
        btTransform tf( rot, trans );
        return tf;
    }


    inline geometry_msgs::Quaternion toRosQuaternion( const btQuaternion& quat )
    {
        geometry_msgs::Quaternion ros;
        ros.x = quat.x();
        ros.y = quat.y();
        ros.z = quat.z();
        ros.w = quat.w();
        return ros;
    }

    inline geometry_msgs::Point toRosPoint( const btVector3& vec, float bt_scale )
    {
        geometry_msgs::Point point;
        point.x = vec.x()/bt_scale;
        point.y = vec.y()/bt_scale;
        point.z = vec.z()/bt_scale;
        return point;
    }

    inline geometry_msgs::Pose toRosPose( const btTransform& tf, float bt_scale )
    {
        geometry_msgs::Pose pose;
        pose.position = toRosPoint( tf.getOrigin(), bt_scale );
        pose.orientation = toRosQuaternion( tf.getRotation() );
        return pose;
    }

    inline std::vector< geometry_msgs::Point > toRosPointVector( const std::vector< btVector3 >& bt, float bt_scale )
    {
        std::vector< geometry_msgs::Point > ros( bt.size() );
        for ( size_t i = 0; i < bt.size() ; i++ )
        {
            ros[i] = toRosPoint( bt[i], bt_scale );
        }
        return ros;
    }
}

#endif // BULLET_ROS_CONVERSIONS_HPP
