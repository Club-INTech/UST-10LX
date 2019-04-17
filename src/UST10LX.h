//
// Created by Teo-CD on 13/02/19.
//

#ifndef LIDAR_UST_10LX_UST10LX_H
#define LIDAR_UST_10LX_UST10LX_H

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <math.h>
#include <string.h>

#include "DataPoint.h"

constexpr float degreeToRadian = (float)M_PI/180;

class UST10LX
{

public:
    /// Number of data points transmitted
    static constexpr uint16_t dataSize = 1081;

    /// Erroneous data point value
    static constexpr int16_t dataError = -1;

    explicit UST10LX(int16_t = 0);
    ~UST10LX();

    void connect(const std::string&);
    bool scan();

    /// True if a connection to the LiDAR is established
    explicit operator bool();

    const std::vector<DataPoint>& getDataPoints();
    const std::array<int16_t,dataSize>& getScan();
    const uint32_t getDataPointCount();

private:
    int16_t charDecode(uint16_t,uint8_t);
    bool write(const std::string&);
    uint16_t read(uint16_t = 0);

    /// LiDAR connection port
    static constexpr uint16_t port = 10940;

    /// In mm ; Distance under which a point is considered to be an error
    static constexpr uint16_t minDistance = 21;

    /// In mm ; Distance above which a point is considered to be at infinity
    static constexpr uint16_t maxDistance = 3000;

    /// In ° : Offset applied to the angle detected by the LiDAR
    const int16_t m_angleOffset;

    int m_socketID;
    sockaddr_in m_socketDescriptor;

    std::string m_receiveBuffer;
    std::array<int16_t,dataSize> m_lastScan;

    std::vector<DataPoint> m_dataPointScan;

    uint32_t m_dataPointCount;
};


#endif //LIDAR_UST_10LX_UST10LX_H
