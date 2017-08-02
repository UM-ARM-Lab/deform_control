#ifndef BULLET_INTERNAL_CONVERSIONS_HPP
#define BULLET_INTERNAL_CONVERSIONS_HPP

#include <vector>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>

#include "bullet_helpers/bullet_math_helpers.hpp"

namespace BulletHelpers
{
    inline void nodeArrayToNodePosVector(const btSoftBody::tNodeArray &m_nodes, std::vector<btVector3> &nodeposvec)
    {
        nodeposvec.resize(m_nodes.size());
        for(int i =0; i < m_nodes.size(); i++)
        {
            nodeposvec[i] = m_nodes[i].m_x;
        }
    }

    inline std::vector<btVector3> nodeArrayToNodePosVector(const btSoftBody::tNodeArray &m_nodes)
    {
        std::vector<btVector3> nodeposvec(m_nodes.size());
        for(int i = 0; i < m_nodes.size(); i++)
        {
            nodeposvec[i] = m_nodes[i].m_x;
        }
        return nodeposvec;
    }

    // Helper to convert contact info array into vector (for forces data) --- Added by Mengyao
    inline std::vector<btVector3> nodeArrayToNodeForceAccumulatorVector(const btSoftBody::tNodeArray &m_nodes)
    {
        std::vector<btVector3> nodeforcevec(m_nodes.size());
        for(int i = 0; i < m_nodes.size(); i++)
        {
            nodeforcevec[i] = m_nodes[i].m_f;
        }
        return nodeforcevec;
    }

    template <typename T>
    inline btVector3 stdVectorToBtVector3(const std::vector<T>& std_vector)
    {
        assert(std_vector.size() == 3);
        return btVector3((btScalar)std_vector[0], (btScalar)std_vector[1], (btScalar)std_vector[2]);
    }
}

#endif // BULLET_INTERNAL_CONVERSIONS_HPP
