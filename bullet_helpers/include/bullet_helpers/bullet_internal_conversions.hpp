#ifndef BULLET_INTERNAL_CONVERSIONS_HPP
#define BULLET_INTERNAL_CONVERSIONS_HPP

#include <vector>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>

namespace BulletHelpers
{
    inline void nodeArrayToNodePosVector(const btAlignedObjectArray<btSoftBody::Node> &m_nodes, std::vector<btVector3> &nodeposvec)
    {
        nodeposvec.resize(m_nodes.size());
        for(int i =0; i < m_nodes.size(); i++)
        {
            nodeposvec[i] = m_nodes[i].m_x;
        }
    }

    inline std::vector<btVector3> nodeArrayToNodePosVector(const btAlignedObjectArray<btSoftBody::Node> &m_nodes)
    {
        std::vector<btVector3> nodeposvec(m_nodes.size());
        for(int i = 0; i < m_nodes.size(); i++)
        {
            nodeposvec[i] = m_nodes[i].m_x;
        }
        return nodeposvec;
    }
}

#endif // BULLET_INTERNAL_CONVERSIONS_HPP
