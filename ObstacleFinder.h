//
// Created by Teo-CD on 15/02/19.
//

#ifndef LIDAR_UST_10LX_OBSTACLEDETECTOR_H
#define LIDAR_UST_10LX_OBSTACLEDETECTOR_H

#include <inttypes.h>
#include <vector>
#include <string>

#include "DataPoint.h"


class ObstacleFinder
{
private:
    int16_t invalidDistance;
    uint16_t maxObstacleWidth;

    std::vector<DataPoint> obstacles;

public:
    explicit ObstacleFinder(int16_t,uint16_t = 0);

    const std::vector<DataPoint>& findObstacles(const std::vector<DataPoint>&);
};


#endif //LIDAR_UST_10LX_OBSTACLEDETECTOR_H
