#ifndef BULLET_MATH_HELPERS_HPP
#define BULLET_MATH_HELPERS_HPP

#include <btBulletDynamicsCommon.h>
#include <vector>

namespace BulletHelpers
{
    inline std::vector<btVector3> AverageObjectNodes(const std::vector<std::vector<btVector3>>& data)
    {
        assert(data.size() > 0);

        if (data.size() == 1)
        {
            return data[0];
        }

        const size_t num_items_to_average = data.size();
        const size_t num_nodes_per_item = data[0].size();

        std::vector<btVector3> average(num_nodes_per_item, btVector3(0, 0, 0));

        for (size_t data_ind = 0; data_ind < num_items_to_average; data_ind++)
        {
            for (size_t node_ind = 0; node_ind < num_nodes_per_item; node_ind++)
            {
                assert(num_nodes_per_item == data[data_ind].size());

                average[node_ind] += data[data_ind][node_ind] / num_items_to_average;
            }
        }

        return average;
    }
}

#endif // BULLET_MATH_HELPERS_HPP
