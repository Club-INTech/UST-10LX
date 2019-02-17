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

    std::vector<DataPoint> obstacles;

    /// Separates coordinates of two points
    static constexpr char pointSeparator = ';';
    /// Separates two sets of coordinates
    static constexpr char coordinatesSeparator = ':';

public:
    explicit ObstacleFinder(int16_t);

    const std::vector<DataPoint>& findObstacles(const std::vector<DataPoint>&);

    static void toString(std::string&,const std::vector<DataPoint>&);
    static std::string toString(const std::vector<DataPoint>&);
};


#endif //LIDAR_UST_10LX_OBSTACLEDETECTOR_H
