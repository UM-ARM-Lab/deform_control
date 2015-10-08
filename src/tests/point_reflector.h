#ifndef POINT_REFLECTOR_H
#define POINT_REFLECTOR_H

#include "simulation/simplescene.h"

class PointReflector
{
public:
    PointReflector(float _mid_x, float _min_y, float _max_y)
    {
        mid_x = _mid_x;
        min_y = _min_y;
        max_y = _max_y;
    }

    btVector3 reflect(btVector3& vector_in)
    {
        btVector3 v_out = vector_in;
        v_out[0] = vector_in[0] - 2*(vector_in[0] - mid_x);
        return v_out;
    }

    float mid_x, min_y, max_y;

    typedef boost::shared_ptr<PointReflector> Ptr;
};

#endif
