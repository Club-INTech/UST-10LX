//
// Created by Teo-CD on 15/02/19.
//

#include "ObstacleFinder.h"


ObstacleFinder::ObstacleFinder(int16_t invalidDistance, uint16_t maxWidth, float_t maxSlope)
{
    this->invalidDistance = invalidDistance;
    maxObstacleWidth = maxWidth;
    maxLocalSlope = maxSlope;
}

/**
 * Detects edges between invalidDistance and other data and with local slope computation.
 * Obstacles that are too large can be split in multiple smaller obstacles.
 * <br>Returned obstacles are polar points with DataPoint.distance being the mean distance of all points in the obstacle
 * and DataPoint.angle being the middle of the obstacle.
 * @brief Finds obstacles in a data set by edge detection
 * @param dataPoints to analyze
 * @returns Reference to vector of found obstacles
 */
const std::vector<DataPoint>& ObstacleFinder::findObstacles(const std::vector<DataPoint>& dataPoints)
{
    obstacles.clear();

    bool inObstacle = false;
    uint16_t startIndex = 0;
    uint16_t endIndex = 0;
    uint32_t totalDistance = 0;

    for(uint16_t i = 0;i<dataPoints.size();i++)
    {
        if(dataPoints.at(i).distance != invalidDistance && !inObstacle)         // If there is valid data and we are not in an obstacle...
        {
            startIndex = i;                                                     // ... set the new beginning of the obstacle ...
            endIndex = i;
            inObstacle = true;

            totalDistance = dataPoints.at(i).distance;                          // ... count the first point of the obstacle;
        }
        else if(dataPoints.at(i).distance != invalidDistance && inObstacle)     // If we are in an obstacle and there is valid data
        {
            // Compute the local slope in order to detect obstacle edges
            float_t localSlope = std::abs((int)(dataPoints.at(i).distance - dataPoints.at(i - 1).distance))/
                                                          (dataPoints.at(i).angle - dataPoints.at(i-1).angle);

            // If the obstacle is too large or if there is an obstacle edge, split it
            if(i-startIndex == maxObstacleWidth || (maxLocalSlope != 0 && localSlope >= maxLocalSlope))
            {
                endIndex = i;

                // Compute the mean of the start and end angles
                float meanAngle = (dataPoints.at(startIndex).angle+dataPoints.at(endIndex).angle)/2.0f;

                // Compute the mean of the distances
                int16_t meanDistance = totalDistance/(endIndex-startIndex);

                // Add them to distance vector
                obstacles.push_back(DataPoint{meanAngle,meanDistance});

                // Start a new obstacle right away
                startIndex = i;
                totalDistance = 0;
            }
            else                                                                // If there is no reason to split
            {
                totalDistance += dataPoints.at(i).distance;                     // Add the distance to the current sum
            }
        }
        else if(dataPoints.at(i).distance == invalidDistance && inObstacle)     // If there is no more valid data ...
        {
            endIndex = i;                                                       // ... Save the end of the obstacle
            inObstacle = false;

            // Compute the mean of the start and end angles
            float meanAngle = (dataPoints.at(startIndex).angle+dataPoints.at(endIndex-1).angle)/2.0f;

            // Compute the mean of the distances
            int16_t meanDistance = totalDistance/(endIndex-startIndex);

            // Add them to distance vector
            obstacles.push_back(DataPoint{meanAngle,meanDistance});
        }
    }

    // If we were in an obstacle at the end of the data set, add it
    if(inObstacle)
    {
        endIndex = dataPoints.size();
        float meanAngle = (dataPoints.at(startIndex).angle+dataPoints.at(endIndex-1).angle)/2.0f;
        int16_t meanDistance = totalDistance/(endIndex-startIndex);
        obstacles.push_back(DataPoint{meanAngle,meanDistance});
    }

    return(obstacles);
}
