#ifndef BULLET_EIGEN_CONVERSIONS_HPP
#define BULLET_EIGEN_CONVERSIONS_HPP

#include <btBulletDynamicsCommon.h>
#include <Eigen/Dense>

namespace BulletHelpers
{
    inline Eigen::Isometry3d toEigenIsometry3d(btTransform&& bt)
    {
        const btVector3& bt_origin = bt.getOrigin();
        const btQuaternion& bt_rot = bt.getRotation();
        const Eigen::Translation3d trans(bt_origin.getX(), bt_origin.getY(), bt_origin.getZ());
        const Eigen::Quaterniond rot(bt_rot.getW(), bt_rot.getX(), bt_rot.getY(), bt_rot.getZ());

        return trans*rot;
    }

    inline Eigen::Isometry3d toEigenIsometry3d(const btTransform& bt)
    {
        const btVector3& bt_origin = bt.getOrigin();
        const btQuaternion& bt_rot = bt.getRotation();
        const Eigen::Translation3d trans(bt_origin.getX(), bt_origin.getY(), bt_origin.getZ());
        const Eigen::Quaterniond rot(bt_rot.getW(), bt_rot.getX(), bt_rot.getY(), bt_rot.getZ());

        return trans*rot;
    }

    inline btVector3 toBtVector3(const Eigen::Vector3d& eigen)
    {
        return btVector3(eigen.x(), eigen.y(), eigen.z());
    }

    inline btQuaternion toBtQuaternion(const Eigen::Quaterniond& eigen)
    {
        return btQuaternion(eigen.x(), eigen.y(), eigen.z(), eigen.w());
    }

    inline btTransform toBtTransform(const Eigen::Isometry3d& eigen)
    {
        const Eigen::Vector3d eigen_origin = eigen.translation();
        const Eigen::Quaterniond eigen_rot(eigen.rotation());
        return btTransform(toBtQuaternion(eigen_rot), toBtVector3(eigen_origin));
    }
}

#endif // BULLET_EIGEN_CONVERSIONS_HPP
