//
// Created by Teo-CD on 15/02/19.
//

#ifndef LIDAR_UST_10LX_OBSTACLEDETECTOR_H
#define LIDAR_UST_10LX_OBSTACLEDETECTOR_H

#include <inttypes.h>
#include <vector>
#include <string>
#include <math.h>

#include "DataPoint.h"

constexpr float PI = 3.14159265359;

class ObstacleFinder
{
private:
    int16_t invalidDistance;
    uint16_t maxDistanceBetweenObstacles;
    float_t maxLocalSlope;
    uint16_t minDistance;

    std::vector<DataPoint> obstacles;
    std::vector<DataPoint> filteredData;

public:
    explicit ObstacleFinder(int16_t,uint16_t = 0,float_t = 0, uint16_t = 0);

    const std::vector<DataPoint>& findObstacles(const std::vector<DataPoint>&);
    const float distanceSq(const DataPoint &a, const DataPoint &b);
};


#endif //LIDAR_UST_10LX_OBSTACLEDETECTOR_H
