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
private:
    int16_t charDecode(uint16_t,uint8_t);
    bool write(const std::string&);
    uint16_t read(uint16_t = 0);

    static constexpr uint16_t m_dataSize = 1081;
    static constexpr uint16_t m_port = 10940;

    std::array<uint16_t,m_dataSize> m_lastScan;

    int m_socketID;
    sockaddr_in m_socketDescriptor;
    std::string m_recieveBuffer;

public:
    UST10LX();
    ~UST10LX();

    void connect(const std::string&);
    bool scan();

    const std::array<uint16_t,m_dataSize>& getScan();
};


#endif //LIDAR_UST_10LX_UST10LX_H
