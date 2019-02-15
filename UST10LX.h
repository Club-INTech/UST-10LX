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
#include <chrono>
#include <thread>


class UST10LX
{

public:
    /// Number of data points transmitted
    static constexpr uint16_t dataSize = 1081;

    UST10LX();
    ~UST10LX();

    void connect(const std::string&);
    bool scan();

    const std::array<int16_t,dataSize>& getScan();

    /// True if a connection to the LiDAR is established
    explicit operator bool();

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

    /// Erroneous data point value
    static constexpr int16_t dataError = -1;

    int m_socketID;
    sockaddr_in m_socketDescriptor;

    std::string m_recieveBuffer;
    std::array<int16_t,dataSize> m_lastScan;
};


#endif //LIDAR_UST_10LX_UST10LX_H
