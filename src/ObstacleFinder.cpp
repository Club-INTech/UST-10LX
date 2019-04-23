//
// Created by Teo-CD on 15/02/19.
//

#include <iostream>
#include "ObstacleFinder.h"


ObstacleFinder::ObstacleFinder(int16_t invalidDistance, uint16_t maxWidth, float_t maxSlope, uint16_t minDistance)
{
    this->invalidDistance = invalidDistance;
    maxDistanceBetweenObstacles = maxWidth;
    maxLocalSlope = maxSlope;
    this->minDistance = minDistance;
}

/**
 * Detects edges between invalidDistance and other data and with local slope computation.
 * Obstacles that are too large can be split in multiple smaller obstacles.
 * <br>Returned obstacles are polar points with DataPoint.distance being the mean distance of all points in the obstacle
 * and DataPoint.angle being the middle of the obstacle.
 * @brief Finds obstacles in a data set by edge detection
 * @param unfilteredData points to analyze
 * @returns Reference to vector of found obstacles
 */
const std::vector<DataPoint>& ObstacleFinder::findObstacles(const std::vector<DataPoint>& unfilteredData)
{
    obstacles.clear();
    filteredData.clear();
    float maxDistanceSq = maxDistanceBetweenObstacles*maxDistanceBetweenObstacles;
    for(int pos = 0;pos<unfilteredData.size();pos++)
    {
        const DataPoint& point = unfilteredData.at(pos);
        if(point.distance < minDistance)
        {
            continue;
        }
        DataPoint* closest = nullptr;
        // find closest
        for (int j = 0; j < obstacles.size(); ++j) {
            if(closest) {
                if(distanceSq(*closest, point) >= distanceSq(obstacles.at(j), point)) { // on a trouvé un point plus proche
                    closest = &obstacles.at(j);
                }
            } else {
                if(distanceSq(obstacles.at(j), point) <= maxDistanceSq) { // on a trouvé un point plus proche
                    closest = &obstacles.at(j);
                }
            }
        }

        if(!closest || distanceSq(*closest, point) >= maxDistanceSq) { // no point to link to OR closest is too far away
            bool toAdd = true;
            for(int i = std::max(0,pos-3);i<std::min((int)unfilteredData.size(),pos+3);i++)
            {
                if(distanceSq(unfilteredData.at(i),point) >= maxDistanceSq)
                {
                    toAdd = false;
                    break;
                }
            }
            if(toAdd)
            {
                obstacles.push_back(point);
            }
        }
    }
    return(obstacles);
}

const float ObstacleFinder::distanceSq(const DataPoint &a, const DataPoint &b) {
    return abs(a.distance*a.distance + b.distance*b.distance - 2 * a.distance * b.distance * cos(b.angle-a.angle));
}
