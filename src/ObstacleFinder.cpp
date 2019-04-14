//
// Created by Teo-CD on 15/02/19.
//

#include "ObstacleFinder.h"


ObstacleFinder::ObstacleFinder(int16_t invalidDistance, uint16_t maxWidth, float_t maxSlope)
{
    this->invalidDistance = invalidDistance;
    maxDistanceBetweenObstacles = maxWidth;
    maxLocalSlope = maxSlope;
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
    obstacles.push_back(unfilteredData.at(0));
    uint16_t countPerObstacle[1081] = {0};
    uint16_t obstacleIndex = 0;
    for(const DataPoint& point: unfilteredData)
    {
        DataPoint* closest = &obstacles.at(0);
        // find closest
        for (int j = 0; j < obstacles.size(); ++j) {
            if(distance(*closest, point) >= distance(obstacles.at(j), point)) { // on a trouvé un point plus proche
                closest = &obstacles.at(j);
            }
        }

        if(!closest || distance(*closest, point) >= maxDistanceSq) { // no point to link to OR closest is too far away
            obstacles.push_back(point);
            countPerObstacle[obstacleIndex++] = 1;
        } else {
            countPerObstacle[obstacleIndex]++;
        }
    }

/*
    uint16_t filteredCount = 0;

    for(int i = 0;i<obstacleIndex;i++) {
        if(countPerObstacle[obstacleIndex] >= 2) {
            filteredData.push_back(obstacles.at(i));
            filteredCount++;
        }
    }

    filteredData.resize(filteredCount);
*/
    return(obstacles);
}

const float ObstacleFinder::distance(const DataPoint& a, const DataPoint& b) {
    return a.distance*a.distance + b.distance*b.distance - 2 * a.distance * b.distance * cos(b.angle-a.angle);
}
