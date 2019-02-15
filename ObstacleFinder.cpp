//
// Created by Teo-CD on 15/02/19.
//

#include "ObstacleFinder.h"

ObstacleFinder::ObstacleFinder(int16_t invalidDistance)
{
    this->invalidDistance = invalidDistance;
}

/**
 * Detects edges between invalidDistance and other data. Returned obstacles are polar points with DataPoint.distance
 * being the mean distance of all points in the obstacle and DataPoint.angle being the middle of the obstacle.
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
            totalDistance += dataPoints.at(i).distance;                         // Add the distance to the current sum
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

/**
 * String is formed by concatenation of all similar coordinates separated by a ObstacleFinder::pointSeparator.
 * Different sets of coordinates are separated by a ObstacleFinder::coordinatesSeparator
 * @brief Converts obstacle vector to a string which can be sent to the High Level
 * @param dataToConvert
 * @return Data converted to a string which can be sent
 */
std::string ObstacleFinder::toString(const std::vector<DataPoint>& dataToConvert)
{
    std::string radii;
    std::string angles;
    for(const DataPoint& point: dataToConvert)
    {
        radii += std::to_string(point.distance);
        radii += pointSeparator;
        angles += std::to_string(point.angle);
        angles += pointSeparator;
    }

    radii.pop_back();
    angles.pop_back();

    radii += coordinatesSeparator;
    radii += angles;
    return (radii);
}

