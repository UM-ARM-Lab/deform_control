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

    
}

#endif // BULLET_MATH_HELPERS_HPP
