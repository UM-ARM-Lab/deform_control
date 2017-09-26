#ifndef BULLET_CLOTH_HELPERS_HPP
#define BULLET_CLOTH_HELPERS_HPP

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

namespace BulletHelpers
{
    /*
     *  returns a vector of strains between all links of a soft body
     */
    inline std::vector<btScalar> getStrain(const btSoftBody* psb){
        int num_links = psb->m_links.size();
        std::vector<btScalar> strains;
        strains.resize(num_links);
            
        for(int i=0; i<num_links; i++)
        {
            const btSoftBody::Link& l = psb->m_links[i];
            const btSoftBody::Node& a = *l.m_n[0];
            const btSoftBody::Node& b = *l.m_n[1];
            const btVector3	del=b.m_x-a.m_x;
            const btScalar	len=del.length();
            strains[i] = (len-l.m_rl)/l.m_rl;
        }
        return strains;
    }

    /*
     *  returns the maximum strain for all links of a soft body
     */
    inline btScalar getMaximumStrain(const btSoftBody* psb){
        std::vector<btScalar> strains = getStrain(psb);
        return *std::max_element(std::begin(strains), std::end(strains));
    }

    
    /**
     * Sets cloth_corner_ind to the extremal points of the mesh
     * Conditions: cloth must be X-Y axis-aligned
     *
     * cloth_corner_ind[0] = minx_miny
     * cloth_corner_ind[1] = minx_maxy
     * cloth_corner_ind[2] = maxx_miny
     * cloth_corner_ind[3] = maxx_maxy
     */
    inline void findClothCornerNodes(const btSoftBody* psb,
                                std::vector<int>& cloth_corner_ind)
    {
        const btSoftBody::tNodeArray cloth_nodes = psb->m_nodes;
                
        cloth_corner_ind.resize(4, 0);
        // Setup defaults for doing the max and min operations inside of the loop
        std::vector<btVector3> corner_node_positions(4);

        // min_x, min_y
        corner_node_positions[0] = btVector3(
            std::numeric_limits<btScalar>::infinity(),
            std::numeric_limits<btScalar>::infinity(),
            0);

        // min_x, max_y
        corner_node_positions[1] = btVector3(
            std::numeric_limits<btScalar>::infinity(),
            -std::numeric_limits<btScalar>::infinity(),
            0);

        // max_x, min_y
        corner_node_positions[2] = btVector3(
            -std::numeric_limits<btScalar>::infinity(),
            std::numeric_limits<btScalar>::infinity(),
            0);

        // max_x, max_y
        corner_node_positions[3] = btVector3(
            -std::numeric_limits<btScalar>::infinity(),
            -std::numeric_limits<btScalar>::infinity(),
            0);


        // Itterate through the nodes in the cloth, finding the extremal points
        for (int ind = 0; ind < cloth_nodes.size(); ind++)
        {
            if (cloth_nodes[ind].m_x.x() <= corner_node_positions[0].x() &&
                cloth_nodes[ind].m_x.y() <= corner_node_positions[0].y())
            {
                cloth_corner_ind[0] = ind;
                corner_node_positions[0] = cloth_nodes[ind].m_x;
            }
            else if (cloth_nodes[ind].m_x.x() <= corner_node_positions[1].x() &&
                     cloth_nodes[ind].m_x.y() >= corner_node_positions[1].y())
            {
                cloth_corner_ind[1] = ind;
                corner_node_positions[1] = cloth_nodes[ind].m_x;
            }
            else if (cloth_nodes[ind].m_x.x() >= corner_node_positions[2].x() &&
                     cloth_nodes[ind].m_x.y() <= corner_node_positions[2].y())
            {
                cloth_corner_ind[2] = ind;
                corner_node_positions[2] = cloth_nodes[ind].m_x;
            }
            else if (cloth_nodes[ind].m_x.x() >= corner_node_positions[3].x() &&
                     cloth_nodes[ind].m_x.y() >= corner_node_positions[3].y())
            {
                cloth_corner_ind[3] = ind;
                corner_node_positions[3] = cloth_nodes[ind].m_x;
            }
        }
    }
    
}

#endif // BULLET_MATH_HELPERS_HPP
