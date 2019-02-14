//
// Created by asphox on 16/10/18.
//

#ifndef LIDAR_UST_10LX_DATAPOINT_H
#define LIDAR_UST_10LX_DATAPOINT_H

#include <stdint-gcc.h>

/**
* \struct
* \brief Structure representing a data to compute (angle+distance)
*/
struct DataPoint
{
    float angle = 0;
    int16_t distance = 0;
};

#endif //LIDAR_UST_10LX_DATAPOINT_H
